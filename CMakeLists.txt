cmake_minimum_required(VERSION 3.0)
project(catch-a-ball-2d)

set (CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)

find_package (Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} 
                    src/main.cpp
                    src/simulator.cpp
                    include/simulator.hpp
                    src/control.cpp
                    include/control.hpp
                    src/objects.cpp
                    include/objects.hpp
                )

target_include_directories(${PROJECT_NAME} PRIVATE include)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES} Eigen3::Eigen)
