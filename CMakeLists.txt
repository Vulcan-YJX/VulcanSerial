cmake_minimum_required(VERSION 3.0)

project(vulcanserial)


option(_BUILD_EXAMPLE "build the example on linux " OFF)

add_definitions("-D_LINUX") 

add_subdirectory(src)


if(_BUILD_EXAMPLE)
    add_subdirectory(example)
endif()

