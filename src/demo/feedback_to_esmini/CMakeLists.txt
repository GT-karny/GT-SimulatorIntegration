cmake_minimum_required(VERSION 3.10)
project(esmini_drive_chrono_demo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SOURCES
    main.cpp
    FmuHelper.cpp
    FmuHelper.h
    OsiHelper.h
    DemoConfiguration.h
)
add_executable(esmini_drive_chrono_feedback ${SOURCES})

# FMILib configuration
# Resolve Repo Root
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
message(STATUS "Repo Root: ${REPO_ROOT}")

# FMILIB configuration
set(FMILIB_ROOT "${REPO_ROOT}/thirdparty/fmi-library")
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

target_compile_definitions(esmini_drive_chrono_feedback PRIVATE
    FMILIB_STATIC_LIB_ONLY
    NOMINMAX
    WIN32_LEAN_AND_MEAN
)

if(MSVC)
    target_compile_options(esmini_drive_chrono_feedback PRIVATE /utf-8 /bigobj)
endif()

# Protobuf (via vcpkg)
find_package(Protobuf REQUIRED)

# Undefine VERSION_MAJOR to avoid conflicts with OSI
add_definitions(-UVERSION_MAJOR)

# Use pre-generated OSI files
set(OSI_GEN_DIR "${REPO_ROOT}/generated/osi3")
file(GLOB OSI_SRCS "${OSI_GEN_DIR}/*.pb.cc")
file(GLOB OSI_HDRS "${OSI_GEN_DIR}/*.pb.h")

target_sources(esmini_drive_chrono_feedback PRIVATE ${OSI_SRCS} ${OSI_HDRS})

target_include_directories(esmini_drive_chrono_feedback PRIVATE 
    "${REPO_ROOT}/generated"
    "${REPO_ROOT}/generated/osi3"
    ${FMILIB_INCLUDE_DIR}
)

target_link_libraries(esmini_drive_chrono_feedback PRIVATE 
    ${FMILIB_LIBRARY} 
    Shlwapi
    protobuf::libprotobuf
)

# Copy config file to build directory
add_custom_command(TARGET esmini_drive_chrono_feedback POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/demo_config.json"
    "$<TARGET_FILE_DIR:esmini_drive_chrono_feedback>/demo_config.json"
)

# Automated FMU Distribution Logic
# Define Source Paths
set(SRC_FMU_DIR "${REPO_ROOT}/FMU")
set(SRC_ESMINI_FMU "${SRC_FMU_DIR}/gt_esmini/esmini.fmu")
set(SRC_DC_FMU "${SRC_FMU_DIR}/gt_drivecontroller/GT-DriveController.fmu")
set(SRC_CHRONO_VEHICLE "${SRC_FMU_DIR}/chrono/FMU2cs_WheeledVehicle/FMU2cs_WheeledVehicle.fmu")
set(SRC_CHRONO_POWERTRAIN "${SRC_FMU_DIR}/chrono/FMU2cs_Powertrain/FMU2cs_Powertrain.fmu")
set(SRC_CHRONO_TIRE "${SRC_FMU_DIR}/chrono/FMU2cs_ForceElementTire/FMU2cs_ForceElementTire.fmu")
set(SRC_CHRONO_TERRAIN "${SRC_FMU_DIR}/chrono/FMU2cs_Terrain/FMU2cs_Terrain.fmu")

# Define Destination Directory
set(DIST_FMU_DIR "$<TARGET_FILE_DIR:esmini_drive_chrono_feedback>/FMU")

add_custom_command(TARGET esmini_drive_chrono_feedback POST_BUILD
    # Create Directory
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DIST_FMU_DIR}"
    
    # Copy FMUs
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_ESMINI_FMU}" "${DIST_FMU_DIR}/esmini.fmu"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_DC_FMU}" "${DIST_FMU_DIR}/GT-DriveController.fmu"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_CHRONO_VEHICLE}" "${DIST_FMU_DIR}/FMU2cs_WheeledVehicle.fmu"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_CHRONO_POWERTRAIN}" "${DIST_FMU_DIR}/FMU2cs_Powertrain.fmu"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_CHRONO_TIRE}" "${DIST_FMU_DIR}/FMU2cs_ForceElementTire.fmu"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SRC_CHRONO_TERRAIN}" "${DIST_FMU_DIR}/FMU2cs_Terrain.fmu"
    
    COMMENT "Copying FMUs to release directory..."
)
