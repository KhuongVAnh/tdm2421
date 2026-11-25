cmake_minimum_required(VERSION 3.16)
project(tdm2421_c2_bridge VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Network Test)

add_library(c2_bridge_core STATIC
    src/MqttCodec.cpp
    src/MqttCodec.h
    src/C2BridgeWorker.cpp
    src/C2BridgeWorker.h
)

target_link_libraries(c2_bridge_core PUBLIC
    Qt6::Core
    Qt6::Network
)

target_include_directories(c2_bridge_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_executable(tdm2421_c2_bridge
    src/main.cpp
    src/MainWindow.cpp
    src/MainWindow.h
)

target_link_libraries(tdm2421_c2_bridge PRIVATE
    c2_bridge_core
    Qt6::Widgets
)

enable_testing()

add_executable(test_mqtt_codec
    tests/test_mqtt_codec.cpp
)

target_link_libraries(test_mqtt_codec PRIVATE
    c2_bridge_core
    Qt6::Test
)

add_test(NAME test_mqtt_codec COMMAND test_mqtt_codec)
