idf_component_register(SRCS "ethernet_example_main.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES ethernet_init esp_netif esp_eth esp_event driver)

# Add SOEM library paths
target_include_directories(${COMPONENT_LIB} PUBLIC
    "/home/qcarver/.local/include"
    "/home/qcarver/.local/include/soem")

# Link against SOEM library
target_link_libraries(${COMPONENT_LIB} PUBLIC
    "/home/qcarver/.local/lib/libsoem.a")

# If SOEM is a shared library (.so), use:
# target_link_libraries(${COMPONENT_LIB} PUBLIC
#     "/home/qcarver/.local/lib/libsoem.so")
