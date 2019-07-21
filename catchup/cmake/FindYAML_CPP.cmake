set(YAML_CPP_DIR "" CACHE PATH "Folder contains YAML_CPP_DIR")

if (YAML_CPP_LIBRARIES AND YAML_CPP_INCLUDE_DIRS)
# in cache already
    set(G2_FOUND TRUE)
else (YAML_CPP_LIBRARIES AND YAML_CPP_INCLUDE_DIRS)
    find_path(YAML_CPP_INCLUDE_DIR
            NAMES
                yaml-cpp/yaml.h
            PATHS
                ${YAML_CPP_DIR}/include
            )
    find_library(YAML_CPP_LIBRARY
            NAMES
		libyaml-cppmd.lib
            PATHS
                ${YAML_CPP_DIR}/build64/Release
            )
    find_library(YAML_CPP_LIBRARY_d
            NAMES
		libyaml-cppmdd.lib
            PATHS
                ${YAML_CPP_DIR}/build64/Debug
            )
    
    set(YAML_CPP_INCLUDE_DIRS
            ${YAML_CPP_INCLUDE_DIR}
       )
    set(YAML_CPP_LIBRARIES
            ${YAML_CPP_LIBRARY}
       )
    set(YAML_CPP_LIBRARIES_d
            ${YAML_CPP_LIBRARY_d}
       )

    if (YAML_CPP_INCLUDE_DIRS AND YAML_CPP_LIBRARIES)
        set(YAML_CPP_FOUND TRUE)
    endif (YAML_CPP_INCLUDE_DIRS AND YAML_CPP_LIBRARIES)

    if (YAML_CPP_FOUND)
        message(STATUS "Found libYAML_CPP:")
        message(STATUS " - Includes: ${YAML_CPP_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${YAML_CPP_LIBRARIES}")
    else (YAML_CPP_FOUND)
        message(FATAL_ERROR "Could not find libYAML_CPP")
    endif (YAML_CPP_FOUND)

    mark_as_advanced(YAML_CPP_INCLUDE_DIRS YAML_CPP_LIBRARIES)

endif (YAML_CPP_LIBRARIES AND YAML_CPP_INCLUDE_DIRS)