#=============================================================================
#
# CMake configuration file for the main ChronoWheel library.
#
#=============================================================================

message(STATUS "==== Shared Library ====")

#-----------------------------------------------------------------------------
# Collect files in the main WheelSim library
#-----------------------------------------------------------------------------

set(SHARED_SOURCES
    ProjWriteData.cpp
    ProjConfig.cpp
    ProjCalcs.cpp
    ProjProc.cpp
    )

set(SHARED_HEADERS
    ProjWriteData.h
    ProjConfig.h
    ProjCalcs.h
    ProjProc.h
    )

source_group(SHARED FILES
    ${SHARED_SOURCES}
    ${SHARED_HEADERS})

# Collect all source and header files for the ChronoWheel library
set(SHARED_FILES
    ${SHARED_SOURCES}
    ${SHARED_HEADERS}
    )

#-----------------------------------------------------------------------------
# Includes for the main library
#-----------------------------------------------------------------------------

# Make PROJ_INCLUDES visible in parent scope
set(PROJ_SHARED_INCLUDES "${CMAKE_CURRENT_SOURCE_DIR}" PARENT_SCOPE)

#-----------------------------------------------------------------------------
# Add the ChronoWheel_shared library
#-----------------------------------------------------------------------------

message(STATUS "${CHRONO_LIBRARIES}")

add_library(Shared ${SHARED_FILES})

set_target_properties(Shared PROPERTIES
    COMPILE_FLAGS "${COMPILE_FLAGS} ${EXTRA_COMPILE_FLAGS}"
    COMPILE_DEFINITIONS "CHRONO_DATA_DIR=\"${CHRONO_DATA_DIR}\"; SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}\"; PROJECT_DATA_DIR=\"${PROJECT_DATA_DIR}\"; PROJECT_CONFIG_DIR=\"${PROJECT_CONFIG_DIR}\""
    LINK_FLAGS "${COMPILE_FLAGS} ${CHRONO_LINKER_FLAGS}"
  )

target_include_directories(Shared PRIVATE ${CHRONO_INCLUDE_DIRS})
target_link_libraries(Shared ${CHRONO_LIBRARIES})

if(UNIX)
    target_link_libraries(Shared ${CHRONO_LIBRARIES} stdc++fs)
endif()