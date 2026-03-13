cmake_minimum_required(VERSION 3.16)
project(QtImuClient LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets Network)
qt_standard_project_setup()

qt_add_executable(qt_imu_client
    src/main.cpp
    src/main_window.cpp
    src/main_window.h
    src/orientation_widget.cpp
    src/orientation_widget.h
    src/quaternion.h
)

target_link_libraries(qt_imu_client PRIVATE Qt6::Widgets Qt6::Network)
