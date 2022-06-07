cmake_minimum_required(VERSION 3.0)

project(vulcanserial)

add_executable(vulcanserial main.cpp)

install(TARGETS vulcanserial RUNTIME DESTINATION bin)
