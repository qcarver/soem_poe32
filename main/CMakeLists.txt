idf_component_register(SRCS "main.cpp" "acquire.cpp" "power_sensor.cpp" "motor_control.cpp" "scan_patterns.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES nvs_flash esp_event driver esp_timer soem_port)

# Define PROJECT_NAME for use in code
target_compile_definitions(${COMPONENT_LIB} PRIVATE PROJECT_NAME="${PROJECT_NAME}")
