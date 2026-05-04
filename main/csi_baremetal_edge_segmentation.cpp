/**
 * @file csi_baremetal_edge_segmentation.cpp
 * @brief Adquisición Wi-Fi CSI y segmentación adaptativa dinámica para Semana 1.
 *
 * Cambios aplicados:
 * - Arreglos correctamente dimensionados.
 * - Inicialización explícita de estados y memoria.
 * - Búfer circular consistente con Welford deslizante.
 * - Validación mínima del buffer CSI.
 * - Habilitación de promiscuous mode antes de CSI.
 * - Manejo más robusto de errores.
 *
 * Nota:
 * - Este archivo sigue siendo una base de Semana 1.
 * - No implementa aún MQTT, WebSockets ni cola RTOS para enviar segmentos.
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
}

// Etiqueta global de diagnóstico
static const char *TAG = "EDGE_HAR_DSP";

/* ==============================================================================
 * PARÁMETROS
 * ============================================================================== */

#define NUM_SUBCARRIERS     64
#define WINDOW_SIZE         50

// Umbrales de partida; deben calibrarse experimentalmente en validaciones futuras
#define VARIANCE_T_START    15.0f
#define VARIANCE_T_STOP      5.0f
#define HYSTERESIS_SAMPLES  20

typedef enum {
    ENV_STATIC = 0,
    ENV_DYNAMIC = 1
} env_state_t;

/* ==============================================================================
 * MEMORIA ESTÁTICA
 * ============================================================================== */

// [ventana temporal][subportadora]
static float phase_history[WINDOW_SIZE][NUM_SUBCARRIERS];

// Estadística por subportadora
static float mean_vector[NUM_SUBCARRIERS];
static float m2_vector[NUM_SUBCARRIERS];
static float variance_vector[NUM_SUBCARRIERS];

// Estado de la ventana / segmentación
static uint16_t current_head = 0;         // posición de escritura en el anillo
static uint16_t samples_collected = 0;    // cuántas muestras reales se han acumulado
static bool buffer_primed = false;
static env_state_t current_state = ENV_STATIC;
static uint16_t hysteresis_counter = 0;

/* ==============================================================================
 * DSP
 * ============================================================================== */

static inline void process_welford_sliding_variance(uint8_t subcarrier, float new_phase)
{
    // Valor que sale del búfer circular
    const float old_phase = phase_history[current_head][subcarrier];

    // Sobrescritura del slot actual
    phase_history[current_head][subcarrier] = new_phase;

    if (!buffer_primed) {
        // Fase de llenado inicial: Welford estándar
        const float count = (float)(samples_collected + 1U);
        const float delta = new_phase - mean_vector[subcarrier];
        mean_vector[subcarrier] += delta / count;
        const float delta2 = new_phase - mean_vector[subcarrier];
        m2_vector[subcarrier] += delta * delta2;
        variance_vector[subcarrier] = (count > 1.0f) ? (m2_vector[subcarrier] / (count - 1.0f)) : 0.0f;
    } else {
        // Fase estable: actualización deslizante O(1)
        const float old_mean = mean_vector[subcarrier];
        const float delta_mean = (new_phase - old_phase) / (float)WINDOW_SIZE;
        const float new_mean = old_mean + delta_mean;

        mean_vector[subcarrier] = new_mean;

        m2_vector[subcarrier] += (new_phase - old_phase) *
                                 (new_phase - new_mean + old_phase - old_mean);

        variance_vector[subcarrier] = m2_vector[subcarrier] / (float)(WINDOW_SIZE - 1);
    }
}

static inline float compute_environment_variance(void)
{
    float acc = 0.0f;
    for (uint8_t k = 0; k < NUM_SUBCARRIERS; ++k) {
        acc += variance_vector[k];
    }
    return acc / (float)NUM_SUBCARRIERS;
}

/* ==============================================================================
 * CALLBACK CSI
 * ============================================================================== */

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    (void)ctx;

    // Validación mínima
    if (info == nullptr || info->buf == nullptr) {
        return;
    }

    // Se espera un bloque mínimo suficiente para 64 subportadoras en esta fase
    if (info->len < (NUM_SUBCARRIERS * 2)) {
        return;
    }

    const int8_t *raw_iq_buffer = (const int8_t *)info->buf;

    for (uint8_t k = 0; k < NUM_SUBCARRIERS; ++k) {
        // Ajusta este orden si tu captura real de CSI viene invertida en tu build.
        const float imag_Q = (float)raw_iq_buffer[(k * 2)];
        const float real_I = (float)raw_iq_buffer[(k * 2) + 1];

        const float phase_radian = atan2f(imag_Q, real_I);
        process_welford_sliding_variance(k, phase_radian);
    }

    const float environment_macro_variance = compute_environment_variance();

    // Avance del anillo: una vez por trama CSI
    current_head = (uint16_t)((current_head + 1U) % WINDOW_SIZE);
    if (samples_collected < WINDOW_SIZE) {
        samples_collected++;
        if (samples_collected >= WINDOW_SIZE) {
            buffer_primed = true;
        }
    }

    // FSM simple de detección
    if (!buffer_primed) {
        return;
    }

    if (current_state == ENV_STATIC) {
        if (environment_macro_variance > VARIANCE_T_START) {
            current_state = ENV_DYNAMIC;
            hysteresis_counter = 0;
            ESP_LOGI(TAG, "UMBRAL DETONADO: inicio de evento.");
        }
    } else {
        if (environment_macro_variance < VARIANCE_T_STOP) {
            hysteresis_counter++;
            if (hysteresis_counter >= HYSTERESIS_SAMPLES) {
                current_state = ENV_STATIC;
                hysteresis_counter = 0;
                ESP_LOGI(TAG, "UMBRAL CAÍDO: fin de evento.");
            }
        } else {
            hysteresis_counter = 0;
        }
    }
}

/* ==============================================================================
 * UTILIDAD DE INICIALIZACIÓN
 * ============================================================================== */

static void init_static_state(void)
{
    memset(phase_history, 0, sizeof(phase_history));
    memset(mean_vector, 0, sizeof(mean_vector));
    memset(m2_vector, 0, sizeof(m2_vector));
    memset(variance_vector, 0, sizeof(variance_vector));

    current_head = 0;
    samples_collected = 0;
    buffer_primed = false;
    current_state = ENV_STATIC;
    hysteresis_counter = 0;
}

static void wifi_init_csi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // Para que CSI funcione de forma consistente en ESP-IDF
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

    wifi_csi_config_t csi_cfg = {};
    csi_cfg.lltf_en = true;
    csi_cfg.htltf_en = true;
    csi_cfg.stbc_htltf2_en = true;
    csi_cfg.ltf_merge_en = true;
    csi_cfg.channel_filter_en = false;
    csi_cfg.manu_scale = false;
    csi_cfg.shift = 0;

    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, nullptr));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

/* ==============================================================================
 * MAIN
 * ============================================================================== */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Inicializando módulo CSI Edge de Semana 1");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    init_static_state();
    wifi_init_csi();

    ESP_LOGI(TAG, "Recepción CSI habilitada. Segmentación base en ejecución.");

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}