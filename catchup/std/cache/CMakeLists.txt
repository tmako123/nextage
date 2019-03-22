#プロジェクト名
SET(execName std_cache)
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)
add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
