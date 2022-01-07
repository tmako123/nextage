#プロジェクト名
SET(execName g2o_QuadraticFunction)
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.h)
add_executable(${execName} ${HDR_FILES} ${SRC_FILES})

TARGET_LINK_LIBRARIES(${execName} 
    ${G2O_LIBRARIES}
    )
