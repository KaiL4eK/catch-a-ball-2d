cmake_minimum_required(VERSION 3.0)
project(catch-a-ball-2d)

set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

add_executable(${PROJECT_NAME} 
                    main.cpp
                    simulator.cpp
                    simulator.hpp
                )

target_include_directories(${PROJECT_NAME} PRIVATE .)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
