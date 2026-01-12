SET(execName ceres_Bundle_Adjustment )
PROJECT(${execName})

FILE(GLOB SRC_FILES *.cpp)
FILE(GLOB HDR_FILES *.h)

add_executable(${execName} ${HDR_FILES} ${SRC_FILES})
target_link_libraries(${execName} 
    ${CERES_LIBRARIES} Ceres::ceres ${OpenCV_LIBS}
    )