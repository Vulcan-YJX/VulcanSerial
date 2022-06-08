cmake_minimum_required(VERSION 3.0)

project(vulcanserial)


option(_BUILD_EXAMPLE "build the example on linux " OFF)

add_definitions("-D_LINUX") 

add_subdirectory(src)


if(_BUILD_EXAMPLE)
    add_subdirectory(example)
endif()

# FILE(GLOB VulcanSerial_LIB_SRCS
#         src/*.cpp
#     )
# 
# add_library(
#     ${PROJECT_NAME}
#     STATIC
#     ${VulcanSerial_LIB_SRCS}
# )


# add_executable(vulcanserial ${OSDK_LIB_SRCS} main.cpp)
# 
# install(TARGETS vulcanserial RUNTIME DESTINATION bin)
