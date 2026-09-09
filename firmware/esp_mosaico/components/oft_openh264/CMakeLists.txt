file(GLOB common "vendor/codec/common/src/*.cpp")
file(GLOB decoder "vendor/codec/decoder/core/src/*.cpp" "vendor/codec/decoder/plus/src/*.cpp")
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    include("${CMAKE_CURRENT_LIST_DIR}/compat/profile_sources.cmake")
endif()
idf_component_register(SRCS ${common} ${decoder} "compat/length_bridge.cpp" "compat/psram_alloc.cpp" "compat/oft_profile.cpp"
    INCLUDE_DIRS "vendor/codec/api" "compat" PRIV_INCLUDE_DIRS "vendor/codec/common/inc"
    "vendor/codec/decoder/core/inc" "vendor/codec/decoder/plus/inc" "vendor/codec/api/wels"
    REQUIRES pthread heap esp_timer)
if("$ENV{OFT_CABAC32}" STREQUAL "compact")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC OFT_CABAC32=2)
elseif("$ENV{OFT_CABAC32}" STREQUAL "1")
    target_compile_definitions(${COMPONENT_LIB} PUBLIC OFT_CABAC32=1)
endif()
set_source_files_properties("${CMAKE_CURRENT_SOURCE_DIR}/vendor/codec/common/src/memory_align.cpp"
    PROPERTIES COMPILE_OPTIONS "-include;${CMAKE_CURRENT_SOURCE_DIR}/compat/psram_alloc.h")
# Upstream C/C++ fallback only; no x86/ARM assembly or encoder. Keep upstream
# source pinned/unmodified. GCC15 warnings in upstream do not relax app warnings.
target_compile_definitions(${COMPONENT_LIB} PRIVATE NDEBUG GENERATED_VERSION_HEADER)
target_compile_options(${COMPONENT_LIB} PRIVATE -O3 -fno-strict-aliasing -std=gnu++11 -Wno-error
    -include "${CMAKE_CURRENT_SOURCE_DIR}/compat/oft_posix.h")
