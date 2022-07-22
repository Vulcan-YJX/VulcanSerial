cmake_minimum_required(VERSION 3.0)

project(RWCommond)

message(STATUS "VulcanSerial lib : ${VulcanSerial_INCLUDE_DIR}")

find_package(VulcanSerial QUIET)

if (VulcanSerial_FOUND)
  add_executable( ${PROJECT_NAME} main.cpp)
  include_directories(${VulcanSerial_INCLUDE_DIR})
  target_link_libraries (${PROJECT_NAME} ${VulcanSerial_LIBRARY})
else()
  # compile with source code
  message(STATUS "Not found system VulcanSerial, compile example with source code")

  set(VulcanSerialRootPath "${CMAKE_CURRENT_SOURCE_DIR}/../..")
  include_directories(${VulcanSerialRootPath}/include/VulcanSerial)
  list(APPEND VulcanSerialSourceFiles ${VulcanSerialRootPath}/src/SerialPort.cpp)
  add_executable( ${PROJECT_NAME} main.cpp ${VulcanSerialSourceFiles})
  target_link_libraries( ${PROJECT_NAME} pthread)

endif ()
