cmake_minimum_required(VERSION 2.8)

find_package(G2O REQUIRED)
if(G2O_INCLUDE_DIR)
    include_directories(${G2O_INCLUDE_DIR})
endif(G2O_INCLUDE_DIR)

ADD_SUBDIRECTORY(PoseGraph)
ADD_SUBDIRECTORY(BundleAdjustment)
ADD_SUBDIRECTORY(LinearFunction)
ADD_SUBDIRECTORY(CubicFunction)
ADD_SUBDIRECTORY(QuadraticFunction)
