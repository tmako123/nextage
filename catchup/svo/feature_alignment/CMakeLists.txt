#プロジェクト名
SET(execName svo_feature_alignment)
cmake_minimum_required(VERSION 2.8)
PROJECT(${execName})

#ソースファイルの追加
FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)

set(SVO_SOURCES
  ../ThirdParty/svo_pro/types.h
  ../ThirdParty/svo_pro/feature_alignment.h
  ../ThirdParty/svo_pro/feature_alignment.cpp
  ../ThirdParty/svo_pro/patch_utils.h
  ../ThirdParty/svo_pro/patch_warp.h
  ../ThirdParty/svo_pro/patch_warp.cpp
  )

add_executable(${execName} ${HDR_FILES} ${SRC_FILES} ${SVO_SOURCES})
target_link_libraries(${execName} ${OpenCV_LIBS})