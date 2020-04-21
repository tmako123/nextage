cmake_minimum_required(VERSION 2.8)

find_package(ZeroMQ REQUIRED)
if(ZeroMQ_INCLUDE_DIRS)
    include_directories(${ZeroMQ_INCLUDE_DIRS})
endif(ZeroMQ_INCLUDE_DIRS)

ADD_SUBDIRECTORY(socket_1_server)
ADD_SUBDIRECTORY(socket_1_client)
ADD_SUBDIRECTORY(socket_2_server)
ADD_SUBDIRECTORY(socket_2_client)
ADD_SUBDIRECTORY(socket_2_client_pose)
