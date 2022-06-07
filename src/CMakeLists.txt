

file(GLOB_RECURSE VulcanSerial_SRC
        "*.cpp")

message(STATUS "prOJECT_NAME source :${PROJECT_NAME}/include/VulcanSerial/*.hpp")

file(GLOB_RECURSE VulcanSerial_HEADERS
        "${CMAKE_SOURCE_DIR}/include/VulcanSerial/*.hpp")
        
# include_directories(
#     ../include/VulcanSerial
# )
option(SERIAL_BUILD_SHARED_LIBS "Build VulcanSerial_HEADERS shared library" OFF)

if (SERIAL_BUILD_SHARED_LIBS)
  set(LibType SHARED)
else()
  set(LibType STATIC)
endif()

add_library(VulcanSerial ${LibType} ${VulcanSerial_SRC} ${VulcanSerial_HEADERS})


target_include_directories(VulcanSerial PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include/VulcanSerial>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>")                                                  

# On Linux, "sudo make install" will typically copy the library
# into the folder /usr/local/bin
install(TARGETS VulcanSerial DESTINATION lib)

# On Linux, "sudo make install" will typically copy the
# folder into /usr/local/include
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/VulcanSerial DESTINATION include)
