#プロジェクト名
SET(execName zeromq_2_server)
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)
add_executable(${execName} ${HDR_FILES} ${SRC_FILES})

TARGET_LINK_LIBRARIES(${execName} 
    ${ZeroMQ_LIBRARIES}
    )

