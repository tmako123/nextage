cmake_minimum_required(VERSION 2.8)

find_package(FBoW REQUIRED)
if(FBoW_INCLUDE_DIR)
    include_directories(${FBoW_INCLUDE_DIR})
endif(FBoW_INCLUDE_DIR)

ADD_SUBDIRECTORY(ImageMatching)
