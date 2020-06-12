cmake_minimum_required(VERSION 2.8)

#Ceres
find_package(CERES REQUIRED)
if(CERES_INCLUDE_DIR)
    include_directories(${CERES_INCLUDE_DIRS})
endif(CERES_INCLUDE_DIR)

find_package(GLOG REQUIRED)
if(GLOG_DIR)
    include_directories(${GLOG_DIR}/../../../include)
endif(GLOG_DIR)

find_package(GFLAGS REQUIRED)
if(GFLAGS_INCLUDE_DIRS)
    include_directories(${GFLAGS_INCLUDE_DIRS})
endif(GFLAGS_INCLUDE_DIRS)


#if(WIN32)
    include_directories($ENV{PYTHONHOME}/Lib/site-packages/numpy/core/include)
#endif()

ADD_SUBDIRECTORY(LinearFunction)
ADD_SUBDIRECTORY(QuadraticFunction)
ADD_SUBDIRECTORY(CubicFunction)
ADD_SUBDIRECTORY(PowellFunction)