idf_component_register(SRCS "main.c" "oft_board.c" "oft_ui.c" "oft_ui_geometry.c" "oft_touch.c" "oft_peripherals.c" "oft_gauge.c" "oft_ble.c" "oft_network.c" "oft_udp.c" "oft_rx_queue.c" "oft_motion.c" "oft_video.cpp"
    "../components/oft_protocol/test/test_duml.c"
    "../components/oft_protocol/test/test_wifi_profile.c"
    "../components/oft_protocol/test/test_capture.c"
    "../components/oft_protocol/test/test_settings.c"
    "../components/oft_protocol/test/test_control.c"
    "../components/oft_protocol/test/test_orientation.c"
    "../components/oft_protocol/test/test_head_target.c"
    "../components/oft_protocol/test/test_media.c" INCLUDE_DIRS "."
    REQUIRES esp_system esp_timer heap soc esp_driver_gpio esp_driver_spi esp_driver_i2c esp_lcd unity oft_protocol oft_sensors oft_openh264 mbedtls bt nvs_flash esp_wifi esp_event esp_netif)
# Route Unity through the same USB VFS as application logs, without patching IDF.
set_source_files_properties(oft_video.cpp PROPERTIES COMPILE_OPTIONS "-O3")
# Main calls USB/display initialization and the on-board tests on a6144-byte
# task stack. Refuse another oversized parent frame before anything is flashed.
set_source_files_properties(main.c PROPERTIES COMPILE_OPTIONS "-Wframe-larger-than=2048")
if("$ENV{OFT_CABAC_IRAM}" STREQUAL "1")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OFT_CABAC_IRAM=1)
endif()
if("$ENV{OFT_RESIDUAL_IRAM}" STREQUAL "1")
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OFT_RESIDUAL_IRAM=1)
endif()
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=unity_putc" "-Wl,--wrap=unity_flush")
target_link_options(${COMPONENT_LIB} INTERFACE "-Wl,--wrap=sys_mbox_new" "-Wl,--wrap=sys_mbox_trypost" "-Wl,--wrap=sys_mbox_free")
if(NOT CMAKE_BUILD_EARLY_EXPANSION AND DEFINED ENV{OFT_BENCH_FIXTURE})
    set(fixture "$ENV{OFT_BENCH_FIXTURE}")
    if(NOT EXISTS "${fixture}")
        message(FATAL_ERROR "Private benchmark fixture missing")
    endif()
    file(SIZE "${fixture}" fixture_size)
    if(fixture_size GREATER 204800 OR fixture_size LESS 16)
        message(FATAL_ERROR "Private benchmark fixture exceeds bounded sample size")
    endif()
    target_add_binary_data(${COMPONENT_LIB} "${fixture}" BINARY RENAME_TO oft_bench_fixture)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE OFT_BENCH_EMBEDDED=1)
endif()
