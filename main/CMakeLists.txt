# Firmware principal del ESP32. El harness portable queda fuera del build normal
# para evitar un segundo app_main.
idf_component_register(
    SRCS "csi_baremetal_edge_segmentation.cpp"
    INCLUDE_DIRS "."
)
