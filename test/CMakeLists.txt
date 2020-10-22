cmake_minimum_required(VERSION 2.8)

set(cmake_build_type debug)

project(test)

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}")
include_directories("/usr/include")
include_directories("/usr/local/include")
include_directories(${OpenCV_INCLUDE_DIRS})

link_directories("/usr/lib")
link_directories("/usr/local/lib")

add_executable(sample src/main.cpp)

target_link_libraries(sample ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})