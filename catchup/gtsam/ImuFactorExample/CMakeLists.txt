#プロジェクト名
SET(execName gtsam_ImuFactorExample)
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.h)
add_executable(${execName} ${HDR_FILES} ${SRC_FILES})

TARGET_LINK_LIBRARIES(${execName} 
    ${OpenCV_LIBS}
    debug ${GTSAM_LIBRARIES_d} optimized ${GTSAM_LIBRARIES} 
    debug ${YAML_CPP_LIBRARIES_d} optimized ${YAML_CPP_LIBRARIES} 
    debug ${GTSAM_BOOST_LIBRARIES_d} optimized ${GTSAM_BOOST_LIBRARIES}
    )

