cmake_minimum_required(VERSION 3.20)
project(VICON_TRACKER)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_library(VRPN_LIB vrpn HINTS /opt/homebrew/lib /usr/local/lib)
find_library(QUAT_LIB quat HINTS /opt/homebrew/lib /usr/local/lib)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

if (NOT VRPN_LIB)
    message(FATAL_ERROR "VRPN library not found. Please install VRPN.")
endif()
if (NOT QUAT_LIB)
    message(FATAL_ERROR "Quat library not found. Please install quatlib.")
endif()

# Include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${zenohc_INCLUDE_DIRS})

# Source files for vicon_tracker library
set(vicon_src src/vicon2pose.cpp src/vicon.cpp)

# Create static library
add_library(vicon_tracker_static STATIC ${vicon_src})
target_include_directories(vicon_tracker_static PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(vicon_tracker_static PUBLIC
    ${VRPN_LIB}
    ${QUAT_LIB}
    zenohcxx::zenohc
    nlohmann_json::nlohmann_json
    Eigen3::Eigen
)

# Add executable
add_executable(test_vicon_tracker src/main.cpp)
target_link_libraries(test_vicon_tracker PRIVATE
    vicon_tracker_static
    ${VRPN_LIB}
    ${QUAT_LIB}
    zenohcxx::zenohc
    nlohmann_json::nlohmann_json
)

# Conditionally set compile options based on compiler
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    target_compile_options(test_vicon_tracker PRIVATE -Wall -O3 -Wno-class-memaccess)
else()
    target_compile_options(test_vicon_tracker PRIVATE -Wall -O3)
endif()