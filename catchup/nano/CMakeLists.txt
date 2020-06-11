cmake_minimum_required(VERSION 2.8)

find_package(NanoGUI REQUIRED)
if(NanoGUI_INCLUDE_DIRS)
    include_directories(${NanoGUI_INCLUDE_DIRS})
endif(NanoGUI_INCLUDE_DIRS)

ADD_SUBDIRECTORY(GUI1)
