idf_component_register(SRCS "vendor/bmm150/bmm150.c" INCLUDE_DIRS "vendor/bmm150")
target_compile_definitions(${COMPONENT_LIB} PUBLIC BMM150_USE_FLOATING_POINT)
