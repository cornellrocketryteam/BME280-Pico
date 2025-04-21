add_library(BME280-Pico INTERFACE)

target_sources(BME280-Pico INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bme280.cpp
    ${CMAKE_CURRENT_LIST_DIR}/bme280/bme280.c
)

target_link_libraries(BME280-Pico INTERFACE
    pico_stdlib
    hardware_i2c
)