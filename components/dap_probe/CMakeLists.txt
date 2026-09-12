# Infineon DAP master for AURIX TC3xx targets.
#
# dap_frame.c is pure computation and is also built by
# components/dap_probe/test/Makefile as a host binary, so keep it free of
# ESP-IDF dependencies.

idf_component_register(
    SRCS "dap_frame.c" "dap_phy.c" "dap_phy_spi.c" "dap_probe.c" "dap_phy_fpga.c" "dap_fpga_route.c" "dap_trace.c"
         "tricore.c" "tricore_bmp.c" "tricore_flash_probe.c" "tricore_flash.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_spi esp_hw_support esp_timer platform_include blackmagic_esp32
)
