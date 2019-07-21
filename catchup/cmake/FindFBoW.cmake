set(FBoW_DIR "" CACHE PATH "Folder contains FBoW_DIR")

if (FBoW_LIBRARIES AND FBoW_INCLUDE_DIRS)
# in cache already
    set(FBoW_FOUND TRUE)
else (FBoW_LIBRARIES AND FBoW_INCLUDE_DIRS)
    find_path(FBoW_INCLUDE_DIR
            NAMES
		fbow.h
            PATHS
                ${FBoW_DIR}/../src
            )

    find_library(FBoW_LIBRARY
            NAMES
		fbow001
            PATHS
                ${FBoW_DIR}/bin/Release
            )
    
    set(FBoW_INCLUDE_DIRS
            ${FBoW_INCLUDE_DIR}
       )
    set(FBoW_LIBRARIES
            ${FBoW_LIBRARY}
       )

    if (FBoW_INCLUDE_DIRS AND FBoW_LIBRARIES)
        set(FBoW_FOUND TRUE)
    endif (FBoW_INCLUDE_DIRS AND FBoW_LIBRARIES)

    if (FBoW_FOUND)
        message(STATUS "Found libFBoW:")
        message(STATUS " - Includes: ${FBoW_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${FBoW_LIBRARIES}")
    else (FBoW_FOUND)
        message(FATAL_ERROR "Could not find libFBoW")
    endif (FBoW_FOUND)

    mark_as_advanced(FBoW_INCLUDE_DIRS FBoW_LIBRARIES)

endif (FBoW_LIBRARIES AND FBoW_INCLUDE_DIRS)