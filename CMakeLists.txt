#=============================================================================
# CMake configuration file for Chrono-Wheel Projects
#=============================================================================

cmake_minimum_required(VERSION 3.1)

project(ISAEprojects)

# Set location of executable
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)

#-----------------------------------------------------------------------------
# Disable some warnings
#-----------------------------------------------------------------------------

if(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
    if(MSVC)
        add_definitions( "-D_CRT_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-D_SCL_SECURE_NO_DEPRECATE" )  # avoids deprecation warnings
        add_definitions( "-DNOMINMAX" ) # do not use MSVC's min/max macros
        add_definitions( "-MP" ) # perform parallel builds
        add_compile_options(/wd4275)                   # disable warnings triggered by Irrlicht
        add_compile_options(/wd4251)                   # class needs to have dll-interface
    endif(MSVC)
  if(MSVC AND ${MSVC_VERSION} GREATER_EQUAL 1915)
        add_definitions( "-D_ENABLE_EXTENDED_ALIGNED_STORAGE" )
  endif()
endif()

#--------------------------------------------------------------
# Enable creation of "application bundles" on MacOSX.
#--------------------------------------------------------------

# This is necessary for any Irrlicht-based project (like the example here).
# For OpenGL-based or non-graphics projects, this is optional and the block
# below can be removed (or else explcitly set CMAKE_MACOSX_BUNDLE to 'OFF').
#
# If creating application bundles, the build output will be named 'myexe.app'.
# Use the convenience script 'run_app.sh' available under 'contrib/appbundle-macosx/'
# to run:
#     start_demo.sh myexe.app

if(APPLE)
    set(CMAKE_MACOSX_BUNDLE ON)
endif()

#-----------------------------------------------------------------------------
# Invoke CMake in subdirectories
#-----------------------------------------------------------------------------

LIST(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}/../Chrono/lib")
find_package(Chrono
             COMPONENTS Irrlicht Multicore
             CONFIG)

#--------------------------------------------------------------
# Return now if Chrono or a required component was not found.
#--------------------------------------------------------------

if (NOT Chrono_FOUND)
  message("Could not find Chrono or one of its required modules")
  return()
endif()

#--------------------------------------------------------------
# Set output data path. Return now if directory does not exist.
#--------------------------------------------------------------
set(PROJECT_DATA_DIR "" CACHE PATH "Path to data output directory")

if(NOT PROJECT_DATA_DIR)
  message(SEND_ERROR "Could not find PROJECT_DATA_DIR")
endif()

#--------------------------------------------------------------
# Set the path to the configuration files. Return now if directory does not exist.
#--------------------------------------------------------------
# set(PROJECT_CONFIG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/config CACHE PATH "Path to data output directory")

#if(NOT PROJECT_CONFIG_DIR)
#  message(SEND_ERROR "Could not find PROJECT_DATA_DIR")
#endif()

#--------------------------------------------------------------
# Add subdirectories
#--------------------------------------------------------------

add_subdirectory(shared)

# Options for configuring/building individual sets of programs
option(ENABLE_SIMS "Build simulation projects" ON)
option(ENABLE_POSTPROCESS "Build post-process scripts" ON)

# Propagate configuration to submodules.
if(ENABLE_SIMS)
  message(STATUS "==== Simulation projects ====")
  add_subdirectory(sims)
endif()

if(ENABLE_POSTPROCESS)
  message(STATUS "==== POSTPROCESS scripts ====")
  add_subdirectory(postprocess)
endif()

add_DLL_copy_command("${CHRONO_DLLS}")