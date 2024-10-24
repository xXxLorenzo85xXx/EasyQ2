zephyr_include_directories(.)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/max86178.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/max86178_ss_instance.c)
