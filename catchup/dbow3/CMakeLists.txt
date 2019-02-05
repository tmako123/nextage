cmake_minimum_required(VERSION 2.8)

find_package(DBoW3 REQUIRED)
if(DBoW3_INCLUDE_DIR)
    include_directories(${DBoW3_INCLUDE_DIR})
endif(DBoW3_INCLUDE_DIR)

ADD_SUBDIRECTORY(ImageMatching)
