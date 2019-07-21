set(DBoW3_DIR "" CACHE PATH "Folder contains DBoW3_DIR")

if (DBoW3_LIBRARIES AND DBoW3_INCLUDE_DIRS)
# in cache already
    set(DBoW3_FOUND TRUE)
else (DBoW3_LIBRARIES AND DBoW3_INCLUDE_DIRS)
    find_path(DBoW3_INCLUDE_DIR
            NAMES
		DBoW3.h
            PATHS
                ${DBoW3_DIR}/../src
            )

    find_library(DBoW3_LIBRARY
            NAMES
		DBoW3001
            PATHS
                ${DBoW3_DIR}/bin/Release
            )
    
    set(DBoW3_INCLUDE_DIRS
            ${DBoW3_INCLUDE_DIR}
       )
    set(DBoW3_LIBRARIES
            ${DBoW3_LIBRARY}
       )

    if (DBoW3_INCLUDE_DIRS AND DBoW3_LIBRARIES)
        set(DBoW3_FOUND TRUE)
    endif (DBoW3_INCLUDE_DIRS AND DBoW3_LIBRARIES)

    if (DBoW3_FOUND)
        message(STATUS "Found libDBoW3:")
        message(STATUS " - Includes: ${DBoW3_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${DBoW3_LIBRARIES}")
    else (DBoW3_FOUND)
        message(FATAL_ERROR "Could not find libDBoW3")
    endif (DBoW3_FOUND)

    mark_as_advanced(DBoW3_INCLUDE_DIRS DBoW3_LIBRARIES)

endif (DBoW3_LIBRARIES AND DBoW3_INCLUDE_DIRS)