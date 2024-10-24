zephyr_include_directories(.)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/easyq.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/easyq_helper.c)
target_sources(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/cmd_manager.c)
