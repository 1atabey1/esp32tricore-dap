set(sources
    main.c
    storage.c
    ice40up5k/ice.c
    network/network.c
    network/network_adapter.c
    network/network_mngr.c
    network/web_server.c
    network/ctrl_server.c
    network/flash_web.c
    network/dap_web.c
    network/lz4_fast.c
    network/mcds_web.c
    network/network_mngr_ota.c
    network/descriptors.c
    network/uart_websocket.c
    boards/board_profile.c
    gpio_loopback_test.c
)

set(dependencies
    fatfs
    driver
    esp_driver_gpio
    esp_driver_ledc
    esp_driver_spi
    esp_driver_uart
    nvs_flash
    esp_netif
    console
    esptool_py
    esp_gdbstub
    esp_psram
    esp_http_server
    esp_http_client
    esp_https_ota
    esp_wifi
    app_update
    network_provisioning
    compat
    platform_include
    blackmagic_esp32
    lcd
    CherryDAP
    dap_probe
)

# ESP-IDF v6 dropped the built-in json component; cJSON then comes from the
# espressif/cjson registry component (main/idf_component.yml).
if(IDF_VERSION_MAJOR LESS 6)
    list(APPEND dependencies json)
endif()

idf_component_register(
    SRCS
        ${sources}
    INCLUDE_DIRS
        . network ice40up5k boards
    PRIV_REQUIRES
        ${dependencies}
        esp_https_server
    EMBED_FILES
        "web/website.html"
        "web/logic_analyzer.html"
        "web/trace.html"
        "web/datatrace.html"
        "web/flash.html"
        "web/help.html"
        "web/ez32.svg"
        "web/favicon.ico"
        "ice40up5k/bitstream.bin"
        # Built from fpga/dap_master; copied in rather than embedded from the
        # build tree, which is not in the repository, so a fresh clone builds.
        "ice40up5k/dap_master.bin"
)

set(host "esp-idf")
set(PLATFORM_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/components/platform_include)
