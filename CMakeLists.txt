cmake_minimum_required(VERSION 3.20.0)


find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(EasyQ2)
target_sources(app PRIVATE main.c)
target_include_directories(app PRIVATE common)
add_subdirectory(MAX86178)
add_subdirectory(drivers)
add_subdirectory(utilities)
add_subdirectory(drivers/EASYQ)