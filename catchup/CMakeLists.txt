cmake_minimum_required(VERSION 2.8)
SET(execName nextage_catchup)
PROJECT(${execName})

#exeの出力フォルダ
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#####################################################################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

find_package( OpenMP )
if (OPENMP_FOUND) 
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

## Eigen3
find_package(Eigen3 REQUIRED)
if(EIGEN3_INCLUDE_DIR)
    include_directories(${EIGEN3_INCLUDE_DIR})
endif(EIGEN3_INCLUDE_DIR)

find_package(OpenCV REQUIRED)
if(OpenCV_INCLUDE_DIRS)
    include_directories(${OpenCV_INCLUDE_DIRS})
    link_directories(${OpenCV_LIB_DIRS})
endif(OpenCV_INCLUDE_DIRS)

#####################################################################

ADD_SUBDIRECTORY(g2o)
