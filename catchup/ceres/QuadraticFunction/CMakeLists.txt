SET(execName ceres_QuadraticFunction)
PROJECT(${execName})

FILE(GLOB_RECURSE SRC_FILES *.cpp)
FILE(GLOB_RECURSE HDR_FILES *.h)
add_executable(${execName} ${HDR_FILES} ${SRC_FILES})

TARGET_LINK_LIBRARIES(${execName} 
    ${CERES_LIBRARIES}  Ceres::ceres ${OpenCV_LIBS}
    )
