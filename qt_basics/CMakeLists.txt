cmake_minimum_required(VERSION 3.16)
project(QtBasicsMouseTracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Qt6 REQUIRED COMPONENTS Widgets)
qt_standard_project_setup()

qt_add_executable(qt_basics_mouse_tracker
    src/main.cpp
    src/mouse_tracker_window.cpp
    src/mouse_tracker_window.h
    src/tracker_canvas.cpp
    src/tracker_canvas.h
)

target_link_libraries(qt_basics_mouse_tracker PRIVATE Qt6::Widgets)
