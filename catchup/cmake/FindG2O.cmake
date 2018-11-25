set(G2O_DIR "" CACHE PATH "Folder contains G2O_DIR")

if (G2O_LIBRARIES AND G2O_INCLUDE_DIRS)
# in cache already
    set(G2_FOUND TRUE)
else (G2O_LIBRARIES AND G2O_INCLUDE_DIRS)
    find_path(G2O_INCLUDE_DIR
            NAMES
                g2o/core/base_vertex.h
            PATHS
                ${G2O_DIR}
            )
    find_library(G2O_LIBRARY
            NAMES
		g2o.lib
            PATHS
                ${G2O_DIR}/lib/Release
            )
    
    set(G2O_INCLUDE_DIRS
            ${G2O_INCLUDE_DIR}
       )
    set(G2O_LIBRARIES
            ${G2O_LIBRARY}
       )

    if (G2O_INCLUDE_DIRS AND G2O_LIBRARIES)
        set(G2O_FOUND TRUE)
    endif (G2O_INCLUDE_DIRS AND G2O_LIBRARIES)

    if (G2O_FOUND)
        message(STATUS "Found libG2O:")
        message(STATUS " - Includes: ${G2O_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${G2O_LIBRARIES}")
    else (G2O_FOUND)
        message(FATAL_ERROR "Could not find libG2O")
    endif (G2O_FOUND)

    mark_as_advanced(G2O_INCLUDE_DIRS G2O_LIBRARIES)

endif (G2O_LIBRARIES AND G2O_INCLUDE_DIRS)