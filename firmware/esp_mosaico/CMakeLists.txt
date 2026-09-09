cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
project(openframetap_mosaico)
# Preserve managed/SDK sources. Scope the minimum-delay adapter to BMI270 only.
idf_component_get_property(bmi_lib espressif__bmi270 COMPONENT_LIB)
idf_component_get_property(bmi_dir espressif__bmi270 COMPONENT_DIR)
set_property(SOURCE "${bmi_dir}/src/bmi270.c" TARGET_DIRECTORY ${bmi_lib}
    APPEND PROPERTY COMPILE_OPTIONS "-include" "${CMAKE_CURRENT_SOURCE_DIR}/main/oft_bmi_delay.h" "-DOFT_BMI_DELAY_OVERRIDE=1")
