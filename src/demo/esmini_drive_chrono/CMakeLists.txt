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

target_compile_definitions(esmini_drive_chrono_demo PRIVATE
    FMILIB_STATIC_LIB_ONLY
    NOMINMAX
    WIN32_LEAN_AND_MEAN
)

if(MSVC)
    target_compile_options(esmini_drive_chrono_demo PRIVATE /utf-8 /bigobj)
endif()

# Protobuf (via vcpkg)  ※Changed to Protobuf (capitalized) to ensure protobuf_generate_cpp is available via wrapper/module
find_package(Protobuf REQUIRED)

# Undefine VERSION_MAJOR to avoid conflicts with OSI
add_definitions(-UVERSION_MAJOR)

# Use pre-generated OSI files
set(OSI_GEN_DIR "${CMAKE_SOURCE_DIR}/generated/osi3")
file(GLOB OSI_SRCS "${OSI_GEN_DIR}/*.pb.cc")
file(GLOB OSI_HDRS "${OSI_GEN_DIR}/*.pb.h")

target_sources(esmini_drive_chrono_demo PRIVATE ${OSI_SRCS} ${OSI_HDRS})

target_include_directories(esmini_drive_chrono_demo PRIVATE 
    "${CMAKE_SOURCE_DIR}/generated"
    "${CMAKE_SOURCE_DIR}/generated/osi3"
    ${FMILIB_INCLUDE_DIR}
)

target_link_libraries(esmini_drive_chrono_demo PRIVATE 
    ${FMILIB_LIBRARY} 
    Shlwapi
    protobuf::libprotobuf
)

# Copy config file to build directory
add_custom_command(TARGET esmini_drive_chrono_demo POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${CMAKE_CURRENT_SOURCE_DIR}/demo_config.json"
    "$<TARGET_FILE_DIR:esmini_drive_chrono_demo>/demo_config.json"
)
