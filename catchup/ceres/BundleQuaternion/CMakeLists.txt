#プロジェクト名
SET(execName ceres_Bundle_quaternion )
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.h)

add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
target_link_libraries(${execName} 
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS})