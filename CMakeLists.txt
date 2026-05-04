# El orden de las líneas es importante
cmake_minimum_required(VERSION 3.16)

# Incluye las herramientas de construcción del ESP-IDF
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Nombre del proyecto (coincide con el nombre de tu carpeta principal)
project(csi_segmentation)