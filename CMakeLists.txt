cmake_minimum_required(VERSION 3.12)
project(VisualPositioning)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_LIBS})
set(CMAKE_CXX_STANDARD 14)

add_executable(VisualPositioning 3dPosition.cpp positioning.cpp positioning.h calibration.cpp calibration.h compute.cpp compute.h aStarSearch.cpp aStarSearch.h)
# 3dPosition.cpp positioning.cpp positioning.h calibration.cpp calibration.h compute.cpp compute.h
target_link_libraries(VisualPositioning ${OpenCV_LIBS})