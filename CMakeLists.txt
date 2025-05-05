cmake_minimum_required(VERSION 3.10)

# Project identification
project(libserial
    VERSION 2.4
    DESCRIPTION "Serial port management library"
    LANGUAGES CXX
)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Generate version file
configure_file(
    ${PROJECT_SOURCE_DIR}/src/version.cpp.in
    ${PROJECT_BINARY_DIR}/src/version.cpp
)

# Source list
add_library(serial SHARED
    src/libserial.cpp
    ${PROJECT_BINARY_DIR}/src/version.cpp
)

# Public API
set_target_properties(serial PROPERTIES PUBLIC_HEADER include/libserial.h)

# Include dir
target_include_directories(serial PUBLIC ${PROJECT_SOURCE_DIR}/include)

# Library version
set_target_properties(serial PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(serial PROPERTIES SOVERSION ${PROJECT_VERSION_MAJOR})

# Test programs
add_executable(test_libserial test/test_libserial.cpp)
target_link_libraries(test_libserial serial)

add_executable(nanoterm test/nanoterm.cpp)
target_link_libraries(nanoterm serial)
