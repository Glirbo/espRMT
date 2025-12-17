# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# Exclude unnecessary components to save RAM/flash for ESP32-WROOM
set(EXCLUDE_COMPONENTS
    # Networking stack (not needed - no Wi-Fi/Ethernet)
    esp_http_client
    esp_http_server
    esp_https_ota
    esp_https_server
    esp-tls
    tcp_transport
    esp_netif
    esp_eth
    esp_wifi
    wpa_supplicant
    esp_local_ctrl
    wifi_provisioning
    protocomm
    mdns

    # Bluetooth (not needed)
    bt

    # Other unused components
    esp_hid
    esp_lcd
    fatfs
    wear_levelling
    sdmmc
    spiffs
    mqtt
    json
    protobuf-c
    perfmon
    cmock
    unity
    console
    esp_gdbstub
    espcoredump
    nvs_sec_provider
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rmtarm)
