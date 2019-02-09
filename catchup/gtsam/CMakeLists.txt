cmake_minimum_required(VERSION 2.8)

#boost
# Find boost

# To change the path for boost, you will need to set:
# BOOST_ROOT: path to install prefix for boost
# Boost_NO_SYSTEM_PATHS: set to true to keep the find script from ignoring BOOST_ROOT

if(MSVC)
	# By default, boost only builds static libraries on windows
	set(Boost_USE_STATIC_LIBS ON)  # only find static libs
	# If we ever reset above on windows and, ...
	# If we use Boost shared libs, disable auto linking.
	# Some libraries, at least Boost Program Options, rely on this to export DLL symbols.
	if(NOT Boost_USE_STATIC_LIBS)
		add_definitions(-DBOOST_ALL_NO_LIB)
	  add_definitions(-DBOOST_ALL_DYN_LINK)
	endif()
	# Virtual memory range for PCH exceeded on VS2015
	if(MSVC_VERSION LESS 1910) # older than VS2017
	  add_definitions(-Zm295)
	endif()
endif()

find_package(Boost 1.43 COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

# Required components
if(NOT Boost_SERIALIZATION_LIBRARY OR NOT Boost_SYSTEM_LIBRARY OR NOT Boost_FILESYSTEM_LIBRARY OR
    NOT Boost_THREAD_LIBRARY OR NOT Boost_DATE_TIME_LIBRARY)
  message(FATAL_ERROR "Missing required Boost components >= v1.43, please install/upgrade Boost or configure your search paths.")
endif()

option(GTSAM_DISABLE_NEW_TIMERS "Disables using Boost.chrono for timing" OFF)
# Allow for not using the timer libraries on boost < 1.48 (GTSAM timing code falls back to old timer library)
set(GTSAM_BOOST_LIBRARIES
    ${Boost_SERIALIZATION_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY}
	${Boost_THREAD_LIBRARY} ${Boost_DATE_TIME_LIBRARY} ${Boost_REGEX_LIBRARY})
if (GTSAM_DISABLE_NEW_TIMERS)
    message("WARNING:  GTSAM timing instrumentation manually disabled")
    add_definitions(-DGTSAM_DISABLE_NEW_TIMERS)
else()
    if(Boost_TIMER_LIBRARY)
      list(APPEND GTSAM_BOOST_LIBRARIES ${Boost_TIMER_LIBRARY} ${Boost_CHRONO_LIBRARY})
    else()
      list(APPEND GTSAM_BOOST_LIBRARIES rt) # When using the header-only boost timer library, need -lrt
      message("WARNING:  GTSAM timing instrumentation will use the older, less accurate, Boost timer library because boost older than 1.48 was found.")
    endif()
endif()


if(NOT (${Boost_VERSION} LESS 105600))
	message("Ignoring Boost restriction on optional lvalue assignment from rvalues")
	add_definitions(-DBOOST_OPTIONAL_ALLOW_BINDING_TO_RVALUES -DBOOST_OPTIONAL_CONFIG_ALLOW_BINDING_TO_RVALUES)
endif()

include_directories(BEFORE SYSTEM ${Boost_INCLUDE_DIR})

#gtsam
find_package(GTSAM REQUIRED)
if(GTSAM_INCLUDE_DIR)
    include_directories(${GTSAM_INCLUDE_DIR})
endif(GTSAM_INCLUDE_DIR)

#yaml_cpp
find_package(YAML_CPP REQUIRED)
if(YAML_CPP_INCLUDE_DIR)
    include_directories(${YAML_CPP_INCLUDE_DIR})
endif(YAML_CPP_INCLUDE_DIR)

ADD_SUBDIRECTORY(ImuFactorEuRoC)
ADD_SUBDIRECTORY(ImuFactorExample)
ADD_SUBDIRECTORY(ImuFactorExample2)
ADD_SUBDIRECTORY(ISAM2_SmartFactorStereo_IMU)

