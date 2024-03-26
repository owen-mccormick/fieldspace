cmake_minimum_required(VERSION 3.20)
project(fieldspace VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
include_directories(/usr/local/include/apriltag/)
enable_testing()

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fieldspace src/main.cpp)
target_link_libraries(fieldspace ${OpenCV_LIBS})
target_link_libraries(fieldspace /usr/local/lib64/libapriltag.so)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
