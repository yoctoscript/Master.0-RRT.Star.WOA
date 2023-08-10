cmake_minimum_required(VERSION 3.0)

project(your_project_name)

set(CMAKE_CXX_STANDARD 17)

add_executable(out Main.cpp MapProcessor.cpp PathPlanner.cpp SteerOptimizer.cpp ControlGenerator.cpp)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(out ${OpenCV_LIBS})