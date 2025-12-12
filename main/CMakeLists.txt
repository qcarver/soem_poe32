idf_component_register(SRCS "main.c" "acquire.cpp" "power_sensor.cpp" "motor_control.cpp" "scan_patterns.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ethernet_init esp_netif esp_eth esp_event driver esp_timer)

# Add SOEM and Xeryon library paths
target_include_directories(${COMPONENT_LIB} PUBLIC
    "/home/qcarver/.local/include"
    "/home/qcarver/.local/include/soem"
    "/home/qcarver/.local/include/xeryon")

# Define PROJECT_NAME for use in code
target_compile_definitions(${COMPONENT_LIB} PRIVATE PROJECT_NAME="${PROJECT_NAME}")

# Link against SOEM and Xeryon libraries
target_link_libraries(${COMPONENT_LIB} PUBLIC
    "/home/qcarver/.local/lib/libsoem.a"
    "/home/qcarver/.local/lib/libxeryon.a")

# If SOEM is a shared library (.so), use:
# target_link_libraries(${COMPONENT_LIB} PUBLIC
#     "/home/qcarver/.local/lib/libsoem.so")
