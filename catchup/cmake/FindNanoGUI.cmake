set(NanoGUI "" CACHE PATH "Folder contains NanoGUI")

if (WIN32)
  set(NANOGUI_USE_GLAD_DEFAULT ON)
else()
  set(NANOGUI_USE_GLAD_DEFAULT OFF)
endif()
option(NANOGUI_USE_GLAD      "Use Glad OpenGL loader library?" ${NANOGUI_USE_GLAD_DEFAULT})

if (NanoGUI_LIBRARIES AND NanoGUI_INCLUDE_DIRS)
# in cache already
    set(NanoGUI_FOUND TRUE)
else (NanoGUI_LIBRARIES AND NanoGUI_INCLUDE_DIRS)
    find_path(NanoGUI_INCLUDE_DIR
            NAMES
		nanogui/button.h
            PATHS
                ${NanoGUI}/../include
            )
    find_path(NanoGUI_nanovg_INCLUDE_DIR
            NAMES
		nanovg.h
            PATHS
                ${NanoGUI}/../ext/nanovg/src
            )
if (${NANOGUI_USE_GLAD})
    find_path(NanoGUI_glad_INCLUDE_DIR
            NAMES
		glad/glad.h
            PATHS
                ${NanoGUI}/../ext/glad/include
            )
    add_definitions(-DNANOGUI_GLAD)  
    list(APPEND NANOGUI_EXTRA_LIBS opengl32)
endif(${NANOGUI_USE_GLAD})
    find_library(NanoGUI_LIBRARIE
            NAMES
		nanogui
            PATHS
                ${NanoGUI}/Release
            )
    find_library(NanoGUI_obj_LIBRARIE
            NAMES
		nanogui-obj
            PATHS
                ${NanoGUI}/nanogui-obj.dir/Release
            )

    set(NanoGUI_INCLUDE_DIRS
            ${NanoGUI_INCLUDE_DIR}
            ${NanoGUI_nanovg_INCLUDE_DIR}
            ${NanoGUI_glad_INCLUDE_DIR}
       )
    set(NanoGUI_LIBRARIES
            ${NanoGUI_LIBRARIE}
            ${NanoGUI_obj_LIBRARIE}
       )

    if (NanoGUI_INCLUDE_DIRS AND NanoGUI_LIBRARIES)
        set(NanoGUI_FOUND TRUE)
    endif (NanoGUI_INCLUDE_DIRS AND NanoGUI_LIBRARIES)

    if (NanoGUI_FOUND)
        message(STATUS "Found libNanoGUI")
        message(STATUS " - Includes: ${NanoGUI_INCLUDE_DIRS}")
        message(STATUS " - Libraries: ${NanoGUI_LIBRARIES}")
    else (NanoGUI_FOUND)
        message(FATAL_ERROR "Could not find libNanoGUI")
    endif (NanoGUI_FOUND)

    mark_as_advanced(NanoGUI_INCLUDE_DIRS NanoGUI_LIBRARIES)

endif (NanoGUI_LIBRARIES AND NanoGUI_INCLUDE_DIRS)