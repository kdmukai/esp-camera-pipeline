idf_component_register(
    SRCS
        "src/cam_pipeline_entropy.c"
    INCLUDE_DIRS "include"
    REQUIRES esp-camera-pipeline freertos mbedtls
)
