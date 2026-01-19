cmake_minimum_required(VERSION 3.22)

project(payload-test LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(
  $<$<COMPILE_LANGUAGE:CXX>:-Wall>
  $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
  $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/deps/lib/cmake")
find_package(MAVLink REQUIRED)
message(STATUS "Found MAVLink: ${MAVLINK_CMAKE_DIR}")
get_target_property(MAVLink_IID MAVLink::mavlink INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "MAVLink include dirs: ${MAVLink_IID}")

include_directories($<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

add_library(port src/port.cpp)
target_link_libraries(port PUBLIC MAVLink::mavlink)

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE port)
