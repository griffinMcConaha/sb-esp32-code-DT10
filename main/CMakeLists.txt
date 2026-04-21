# Register the single controller component used by this ESP-IDF app. The real
# runtime split lives in main.c (entrypoint) and dispersion_controller.c.
idf_component_register(SRCS "main.c" "dispersion_controller.c"
                    INCLUDE_DIRS ".")
