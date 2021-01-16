#プロジェクト名

## yaml-cpp
find_package(yaml-cpp REQUIRED)
if(YAML_CPP_INCLUDE_DIR)
    include_directories(${YAML_CPP_INCLUDE_DIR})
endif(YAML_CPP_INCLUDE_DIRS)

SET(execName ceres_VisualInertial )
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.h)

add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
target_link_libraries(${execName} 
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS}
    ${YAML_CPP_LIBRARIES})