cmake_minimum_required(VERSION 3.10)

project(esmini_drive_chrono_demo)

# Source files
set(SOURCES
    main.cpp
    FmuHelper.cpp
    FmuHelper.h
    OsiHelper.h
    DemoConfiguration.h
)

add_executable(esmini_drive_chrono_demo ${SOURCES})

# FMILib configuration
set(FMILIB_ROOT "${CMAKE_SOURCE_DIR}/thirdparty/fmi-library")

set(FMILIB_INCLUDE_DIR 
    "${FMILIB_ROOT}/build/fmilib_include"
    "${FMILIB_ROOT}/build/fmilib_config_include"
    "${FMILIB_ROOT}/src/CAPI/include"
    "${FMILIB_ROOT}/src/Import/include"
    "${FMILIB_ROOT}/src/Util/include"
    "${FMILIB_ROOT}/src/XML/include"
    "${FMILIB_ROOT}/src/ZIP/include"
    "${FMILIB_ROOT}/ThirdParty/FMI/default"
)
set(FMILIB_LIBRARY "${FMILIB_ROOT}/build/Release/fmilib.lib")

add_definitions(-DFMILIB_STATIC_LIB_ONLY)

target_include_directories(esmini_drive_chrono_demo PRIVATE ${FMILIB_INCLUDE_DIR})
target_link_libraries(esmini_drive_chrono_demo PRIVATE ${FMILIB_LIBRARY} Shlwapi)

# Copy config file to build directory
add_custom_command(TARGET esmini_drive_chrono_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/demo_config.json"
    "$<TARGET_FILE_DIR:esmini_drive_chrono_demo>/demo_config.json"
)
