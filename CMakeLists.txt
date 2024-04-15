cmake_minimum_required(VERSION 3.20)
project(fieldspace VERSION 0.1.0 LANGUAGES C CXX)

include(CTest)
include_directories(/usr/local/include/apriltag/)
enable_testing()

# Glib resource compile (see https://stackoverflow.com/questions/63697778/how-to-use-glib-compile-resources-with-cmake)
# find_program(GLIB_COMPILE_RESOURCES NAMES glib-compile-resources REQUIRED)
# set(GRESOURCE_C resources.gresource.c)
# set(GRESOURCE_XML ../resources/resources.gresource.xml)
# add_custom_command(
  # OUTPUT ${GRESOURCE_C}
  # WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  # COMMAND ${GLIB_COMPILE_RESOURCES}
  # ARGS
    # --target=${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_C}
    # ${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_XML}
  # VERBATIM
  # MAIN_DEPENDENCY ${GRESOURCE_XML}
# )

# add_custom_target(
  # resources
  # DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE_C}
# )

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-4.0)

include_directories(${GTKMM_INCLUDE_DIRS})
link_directories(${GTKMM_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
# find_package(nlohmann_json 3.2.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(fieldspace src/main.cpp)
# set_source_files_properties(
  # ${CMAKE_CURRENT_BINARY_DIR}/${GRESOURCE}
  # PROPERTIES GENERATED TRUE
# )
# add_dependencies(${PROJECT_NAME} resources)
target_link_libraries(fieldspace ${OpenCV_LIBS})
target_link_libraries(fieldspace /usr/local/lib64/libapriltag.so)
target_link_libraries(${CMAKE_PROJECT_NAME} ${GTKMM_LIBRARIES})
# target_link_libraries(fieldspace nlohmann_json::nlohmann_json)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
