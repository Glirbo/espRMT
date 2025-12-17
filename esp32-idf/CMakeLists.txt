# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Exclude unnecessary components to save DRAM (6 motors use ~393KB for RMT buffers)
# This is a motor controller - no WiFi, Bluetooth, or networking needed
set(EXCLUDE_COMPONENTS
    esp_http_client esp_http_server esp_https_ota esp_https_server
    esp-tls tcp_transport esp_netif esp_eth esp_wifi wpa_supplicant
    wifi_provisioning protocomm mdns bt esp_hid esp_lcd
    fatfs wear_levelling sdmmc spiffs mqtt json protobuf-c
    perfmon cmock unity console esp_gdbstub espcoredump nvs_sec_provider
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rmtarm)
