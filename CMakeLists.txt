cmake_minimum_required(VERSION 3.10)
project(GT_SimulatorIntegration)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add subdirectory for demos
add_subdirectory(src/demo/chrono)
add_subdirectory(src/demo/esmini_drive_chrono)
add_subdirectory(src/demo/feedback_to_esmini)
