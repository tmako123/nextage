set(GTSAM_DIR "" CACHE PATH "Folder contains GTSAM_DIR")

if (GTSAM_LIBRARIES AND GTSAM_INCLUDE_DIRS)
# in cache already
    set(GTSAM_FOUND TRUE)
else (GTSAM_LIBRARIES AND GTSAM_INCLUDE_DIRS)
    find_path(GTSAM_INCLUDE_DIR
            NAMES
                gtsam/global_includes.h
            PATHS
                ${GTSAM_DIR}
            )
    find_library(GTSAM_LIBRARY
            NAMES
		gtsam.lib
            PATHS
                ${GTSAM_DIR}/build64/lib/Release
            )

    find_library(GTSAM_LIBRARY_d
            NAMES
		gtsamDebug.lib
            PATHS
                ${GTSAM_DIR}/build64/lib/Debug
            )
    
    set(GTSAM_INCLUDE_DIRS
            ${GTSAM_INCLUDE_DIR}
       )
    set(GTSAM_LIBRARIES
            ${GTSAM_LIBRARY}
       )

    set(GTSAM_LIBRARIES_d
            ${GTSAM_LIBRARY_d}
       )

    if (GTSAM_INCLUDE_DIRS AND GTSAM_LIBRARIES)
        set(GTSAM_FOUND TRUE)
    endif (GTSAM_INCLUDE_DIRS AND GTSAM_LIBRARIES)

    if (GTSAM_FOUND)
        message(STATUS "Found libGTSAM:")
        message(STATUS " - Includes: ${GTSAM_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${GTSAM_LIBRARIES}")
        message(STATUS " - DebugLibraries: ${GTSAM_LIBRARIES_d}")
    else (GTSAM_FOUND)
        message(FATAL_ERROR "Could not find libGTSAM")
    endif (GTSAM_FOUND)

    mark_as_advanced(GTSAM_INCLUDE_DIRS GTSAM_LIBRARIES)

endif (GTSAM_LIBRARIES AND GTSAM_INCLUDE_DIRS)