cmake_minimum_required(VERSION 2.8)
SET(execName nextage_catchup)
PROJECT(${execName})

#exeの出力フォルダ
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

##
#option
option(USE_STD "catchup STD" OFF)
option(USE_G2O "catchup G2O" OFF)
option(USE_FBoW "catchup FBoW" OFF)
option(USE_DBoW3 "catchup DBoW3" OFF)
option(USE_GTSAM "catchup GTSAM" OFF)
option(USE_SIMD "catchup SIMD" OFF)
option(USE_PLY "catchup PLY" OFF)
option(USE_ZEROMQ "catchup ZEROMQ" OFF)
option(USE_CV "catchup CV" OFF)
option(USE_NANO "catchup NANO" OFF)
option(USE_CERES "catchup CERES" OFF)
##

#####################################################################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

set(CMAKE_CXX_STANDARD 17)

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
if(USE_STD)
ADD_SUBDIRECTORY(std)
endif(USE_STD)

if(USE_G2O)
ADD_SUBDIRECTORY(g2o)
endif(USE_G2O)

if(USE_FBoW)
ADD_SUBDIRECTORY(fbow)
endif(USE_FBoW)

if(USE_DBoW3)
ADD_SUBDIRECTORY(dbow3)
endif(USE_DBoW3)

if(USE_GTSAM)
ADD_SUBDIRECTORY(gtsam)
endif(USE_GTSAM)

if(USE_SIMD)
ADD_SUBDIRECTORY(simd)
endif(USE_SIMD)

if(USE_PLY)
ADD_SUBDIRECTORY(ply)
endif(USE_PLY)

if(USE_ZEROMQ)
ADD_SUBDIRECTORY(zeromq)
endif(USE_ZEROMQ)

if(USE_CV)
ADD_SUBDIRECTORY(cv)
endif(USE_CV)

if(USE_NANO)
ADD_SUBDIRECTORY(nano)
endif(USE_NANO)

if(USE_CERES)
ADD_SUBDIRECTORY(ceres)
endif(USE_CERES)
