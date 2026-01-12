SET(execName cv_3d-3dCorrespondence)
PROJECT(${execName})

FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.hpp)
add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
target_link_libraries(${execName} 
    ${CERES_LIBRARIES}
    ${OpenCV_LIBS})