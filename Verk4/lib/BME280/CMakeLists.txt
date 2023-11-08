add_library(BME280 INTERFACE)

target_sources(BME280 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/BME280.cpp
)

target_include_directories(BME280 INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(BME280 INTERFACE hardware_i2c)