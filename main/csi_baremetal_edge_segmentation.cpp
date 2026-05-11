/**
 * @file csi_baremetal_edge_segmentation.cpp
 * @brief CSI Edge Segmentador — Semana 3/4
 *
 * Solución para ESP-IDF v6 + ESP32-D0WD-V3:
 * El ESP32 clásico NO permite esp_wifi_set_csi_config en modo STA puro
 * mientras está conectado a un AP. La solución es WIFI_MODE_APSTA:
 *
 *  • STA → se conecta al router local configurado (para uso futuro MQTT)
 *  • AP  → crea "ESP32_CSI_AP" (el CSI se captura de este enlace)
 *  • CSI → activado en modo promiscuo sobre la interfaz AP
 *
 * FSM de segmentación con parámetros optimizados de Semana 3:
 *   T_start=2.60, T_stop=2.50, hysteresis=10, ventana=50
 */

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

extern "C" {
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
}

#if __has_include("wifi_credentials.h")
#include "wifi_credentials.h"
#else
#define STA_SSID        "CHANGE_ME_WIFI_SSID"
#define STA_PASS        "CHANGE_ME_WIFI_PASSWORD"
#endif

// ─── Etiqueta de diagnóstico ──────────────────────────────────────────────────
static const char *TAG = "EDGE_HAR_DSP";

/* ==============================================================================
 * CREDENCIALES WiFi
 * ============================================================================== */

// Red a la que el ESP32 se conecta como cliente.
// Para uso local, crear main/wifi_credentials.h a partir de
// main/wifi_credentials.example.h. Ese archivo queda ignorado por Git.
#define WIFI_MAX_RETRY  10

// AP propio que crea el ESP32 (necesario para habilitar CSI en v6)
#define AP_SSID         "ESP32_CSI_AP"
#define AP_PASS         "csi12345"          // mínimo 8 caracteres
#define AP_CHANNEL      6                   // canal fijo para CSI estable
#define AP_MAX_CONN     4

#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT       BIT1

static EventGroupHandle_t s_wifi_event_group = nullptr;
static int                s_retry_count      = 0;

/* ==============================================================================
 * PARÁMETROS CSI / SEGMENTACIÓN  (Semana 3 optimizados)
 * ============================================================================== */

#define NUM_SUBCARRIERS     64
#define WINDOW_SIZE         50
#define VARIANCE_T_START    2.60f
#define VARIANCE_T_STOP     2.50f
#define HYSTERESIS_SAMPLES  10
#define MAX_SEGMENT_LENGTH  100U

typedef enum { ENV_STATIC = 0, ENV_DYNAMIC = 1 } env_state_t;

static uint32_t   s_frames_received = 0;
static float      s_last_variance   = 0.0f;
static uint32_t   s_segments_closed = 0;
static uint32_t   s_timeout_closed  = 0;

/* ==============================================================================
 * MEMORIA ESTÁTICA — DSP
 * ============================================================================== */

static float phase_history[WINDOW_SIZE][NUM_SUBCARRIERS];
static float mean_vector[NUM_SUBCARRIERS];
static float m2_vector[NUM_SUBCARRIERS];
static float variance_vector[NUM_SUBCARRIERS];

static uint16_t    current_head       = 0;
static uint16_t    samples_collected  = 0;
static bool        buffer_primed      = false;
static env_state_t current_state      = ENV_STATIC;
static uint16_t    hysteresis_counter = 0;
static uint32_t    current_segment_start = 0;

/* ==============================================================================
 * DSP — Varianza deslizante Welford O(1)
 * ============================================================================== */

static inline void process_welford_sliding_variance(uint8_t sub, float new_phase)
{
    const float old_phase = phase_history[current_head][sub];
    phase_history[current_head][sub] = new_phase;

    if (!buffer_primed) {
        const float n      = (float)(samples_collected + 1U);
        const float delta  = new_phase - mean_vector[sub];
        mean_vector[sub]  += delta / n;
        const float delta2 = new_phase - mean_vector[sub];
        m2_vector[sub]    += delta * delta2;
        variance_vector[sub] = (n > 1.0f) ? (m2_vector[sub] / (n - 1.0f)) : 0.0f;
    } else {
        const float old_mean   = mean_vector[sub];
        const float new_mean   = old_mean + (new_phase - old_phase) / (float)WINDOW_SIZE;
        mean_vector[sub]       = new_mean;
        m2_vector[sub]        += (new_phase - old_phase) *
                                 (new_phase - new_mean + old_phase - old_mean);
        variance_vector[sub]   = m2_vector[sub] / (float)(WINDOW_SIZE - 1);
    }
}

static inline float compute_environment_variance(void)
{
    float acc = 0.0f;
    for (uint8_t k = 0; k < NUM_SUBCARRIERS; ++k) acc += variance_vector[k];
    return acc / (float)NUM_SUBCARRIERS;
}

/* ==============================================================================
 * CALLBACK CSI
 * ============================================================================== */

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;
    if (!info || !info->buf)             return;
    if (info->len < (NUM_SUBCARRIERS*2)) return;

    const int8_t *iq = (const int8_t *)info->buf;
    for (uint8_t k = 0; k < NUM_SUBCARRIERS; ++k) {
        process_welford_sliding_variance(k, atan2f((float)iq[k*2], (float)iq[k*2+1]));
    }

    const float env_var = compute_environment_variance();

    current_head = (uint16_t)((current_head + 1U) % WINDOW_SIZE);
    if (samples_collected < WINDOW_SIZE) {
        samples_collected++;
        if (samples_collected >= WINDOW_SIZE) buffer_primed = true;
    }

    s_frames_received = s_frames_received + 1u;
    s_last_variance   = env_var;

    if (!buffer_primed) return;

    const uint32_t frame_idx = s_frames_received - 1U;

    // ── FSM de segmentación ────────────────────────────────────────────────
    if (current_state == ENV_STATIC) {
        if (env_var > VARIANCE_T_START) {
            current_state      = ENV_DYNAMIC;
            hysteresis_counter = 0;
            current_segment_start = frame_idx;
            ESP_LOGI(TAG, ">>> EVENTO DETECTADO   var=%.4f > %.2f <<<",
                     env_var, VARIANCE_T_START);
        }
    } else {
        const uint32_t segment_len = (frame_idx >= current_segment_start)
            ? (frame_idx - current_segment_start + 1U)
            : 0U;

        if (segment_len >= MAX_SEGMENT_LENGTH) {
            current_state      = ENV_STATIC;
            hysteresis_counter = 0;
            s_segments_closed++;
            s_timeout_closed++;
            ESP_LOGI(TAG,
                     ">>> FIN POR TIMEOUT   frames=%lu start=%lu end=%lu <<<",
                     (unsigned long)segment_len,
                     (unsigned long)current_segment_start,
                     (unsigned long)frame_idx);
        } else if (env_var < VARIANCE_T_STOP) {
            if (++hysteresis_counter >= HYSTERESIS_SAMPLES) {
                current_state      = ENV_STATIC;
                hysteresis_counter = 0;
                s_segments_closed++;
                ESP_LOGI(TAG, ">>> FIN DE EVENTO      var=%.4f < %.2f <<<",
                         env_var, VARIANCE_T_STOP);
            }
        } else {
            hysteresis_counter = 0;
        }
    }
}

/* ==============================================================================
 * HANDLERS WiFi / IP
 * ============================================================================== */

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        // NO llamamos esp_wifi_connect() aquí.
        // La conexión STA se iniciará DESPUÉS de configurar CSI.
        ESP_LOGI(TAG, "STA iniciado (esperando orden de conexión)");

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < WIFI_MAX_RETRY) {
            s_retry_count++;
            ESP_LOGW(TAG, "Reintento %d/%d...", s_retry_count, WIFI_MAX_RETRY);
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "No se pudo conectar a '%s'.", STA_SSID);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *ev = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "IP obtenida: " IPSTR, IP2STR(&ev->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_START) {
        ESP_LOGI(TAG, "AP '%s' activo en canal %d", AP_SSID, AP_CHANNEL);

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *ev = (wifi_event_ap_staconnected_t *)data;
        ESP_LOGI(TAG, "Cliente conectado al AP — MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 ev->mac[0], ev->mac[1], ev->mac[2],
                 ev->mac[3], ev->mac[4], ev->mac[5]);
    }
}

/* ==============================================================================
 * INIT ESTADO DSP
 * ============================================================================== */

static void init_static_state(void)
{
    memset(phase_history,   0, sizeof(phase_history));
    memset(mean_vector,     0, sizeof(mean_vector));
    memset(m2_vector,       0, sizeof(m2_vector));
    memset(variance_vector, 0, sizeof(variance_vector));
    current_head       = 0;
    samples_collected  = 0;
    buffer_primed      = false;
    current_state      = ENV_STATIC;
    hysteresis_counter = 0;
    current_segment_start = 0;
    s_frames_received = 0;
    s_last_variance = 0.0f;
    s_segments_closed = 0;
    s_timeout_closed = 0;
}

/* ==============================================================================
 * INIT WiFi APSTA + CSI
 * ============================================================================== */

static bool wifi_apsta_and_enable_csi(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    // Configuración STA
    wifi_config_t sta_cfg = {};
    strncpy((char *)sta_cfg.sta.ssid,     STA_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, STA_PASS, sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));

    // Configuración AP
    wifi_config_t ap_cfg = {};
    strncpy((char *)ap_cfg.ap.ssid,     AP_SSID, sizeof(ap_cfg.ap.ssid) - 1);
    strncpy((char *)ap_cfg.ap.password, AP_PASS,  sizeof(ap_cfg.ap.password) - 1);
    ap_cfg.ap.ssid_len        = (uint8_t)strlen(AP_SSID);
    ap_cfg.ap.channel         = AP_CHANNEL;
    ap_cfg.ap.authmode        = WIFI_AUTH_WPA2_PSK;
    ap_cfg.ap.max_connection  = AP_MAX_CONN;
    ap_cfg.ap.beacon_interval = 100;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));

    ESP_ERROR_CHECK(esp_wifi_start());

    // ── CLAVE ESP-IDF v6: configurar CSI INMEDIATAMENTE tras start(),
    //    ANTES de que la asociación STA arranque (evento STA_START dispara
    //    esp_wifi_connect() en el handler, pero todavía no hay asociación).
    ESP_LOGI(TAG, "Configurando CSI antes de asociar STA...");
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, nullptr));

    wifi_csi_config_t csi_cfg = {};
    csi_cfg.lltf_en           = true;
    csi_cfg.htltf_en          = true;
    csi_cfg.stbc_htltf2_en    = true;
    csi_cfg.ltf_merge_en      = true;
    csi_cfg.channel_filter_en = false;
    csi_cfg.manu_scale        = false;
    csi_cfg.shift             = 0;
    csi_cfg.dump_ack_en       = false;

    esp_err_t err = esp_wifi_set_csi_config(&csi_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_csi_config: %s (0x%x)", esp_err_to_name(err), err);
        ESP_LOGE(TAG, "DIAGNÓSTICO: promiscuous activo, STA en auth state=%d",
                 (int)err);
        return false;
    }

    err = esp_wifi_set_csi(true);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_wifi_set_csi(true): %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "✓ CSI ACTIVO — esperando tramas WiFi en canal %d", AP_CHANNEL);

    // CSI ya está activo. Ahora SÍ conectamos la STA al router.
    ESP_LOGI(TAG, "Iniciando conexión STA a '%s'...", STA_SSID);
    esp_wifi_connect();

    EventBits_t bits = xEventGroupWaitBits(
        s_wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE, pdFALSE,
        pdMS_TO_TICKS(30000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "STA conectado a '%s' ✓", STA_SSID);
    } else {
        ESP_LOGW(TAG, "STA sin IP — CSI sigue activo con AP propio");
    }

    ESP_LOGI(TAG, "Conecta cualquier dispositivo WiFi a '%s' (pass: '%s')",
             AP_SSID, AP_PASS);
    return true;
}

/* ==============================================================================
 * TASK DE TELEMETRÍA
 * ============================================================================== */

static void telemetry_task(void *pv)
{
    (void)pv;
    uint32_t prev = 0;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint32_t now   = s_frames_received;
        float    var   = s_last_variance;
        uint32_t delta = now - prev;
        prev           = now;

        ESP_LOGI(TAG,
            "[TELEM] total=%lu (+%lu/2s) var=%.4f estado=%s hist=%u buffer=%s seg=%lu timeout=%lu",
            (unsigned long)now,
            (unsigned long)delta,
            var,
            (current_state == ENV_DYNAMIC) ? "DYNAMIC" : "STATIC",
            hysteresis_counter,
            buffer_primed ? "LISTO" : "llenando",
            (unsigned long)s_segments_closed,
            (unsigned long)s_timeout_closed);
    }
}

/* ==============================================================================
 * MAIN
 * ============================================================================== */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  CSI Edge Segmentador — Semana 3/4");
    ESP_LOGI(TAG, "  ESP-IDF v6.0.1 | ESP32-D0WD-V3 r3.1");
    ESP_LOGI(TAG, "  Modo: APSTA | STA='%s' AP='%s'", STA_SSID, AP_SSID);
    ESP_LOGI(TAG, "  T_start=%.2f T_stop=%.2f hist=%d ventana=%d max_seg=%lu",
             VARIANCE_T_START,
             VARIANCE_T_STOP,
             HYSTERESIS_SAMPLES,
             WINDOW_SIZE,
             (unsigned long)MAX_SEGMENT_LENGTH);
    ESP_LOGI(TAG, "========================================");

    // NVS requerido por WiFi
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    init_static_state();

    bool csi_ok = wifi_apsta_and_enable_csi();

    xTaskCreate(telemetry_task, "telemetry", 3072, nullptr, 1, nullptr);

    if (csi_ok) {
        ESP_LOGI(TAG, "Sistema OPERACIONAL. Segmentando CSI en tiempo real.");
    } else {
        ESP_LOGW(TAG, "CSI no pudo activarse. Revisa los logs anteriores.");
    }

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
