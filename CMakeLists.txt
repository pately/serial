project(serial_comm)
cmake_minimum_required(VERSION 2.8)

set(ENV{WINDOWS_BUILD} TRUE)

if(DEFINED ENV{WINDOWS_BUILD})
    MESSAGE("BUILD FOR WINDOWS")
    set(HELPER_LIB
       serial_windows/serial_windows.cc
    )
else()
    MESSAGE("BUILD FOR LINUX")
    set(HELPER_LIB
       serial_linux/serial_linux.cc
       raii.h
    )
endif()

add_definitions(
  -std=c++11
  -Wall
)

aux_source_directory(. SRC_LIST)
add_executable(${PROJECT_NAME} ${SRC_LIST} ${HELPER_LIB})
