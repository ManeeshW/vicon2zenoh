cmake_minimum_required(VERSION 3.20)
project(VICON_TRACKER)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(Python REQUIRED COMPONENTS Interpreter Development)
find_package(Eigen3 REQUIRED)
find_library(VRPN_LIB vrpn HINTS /opt/homebrew/lib /usr/local/lib)
find_library(QUAT_LIB quat HINTS /opt/homebrew/lib /usr/local/lib)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)
find_package(nlohmann_json 3.2.0 REQUIRED)

# Qt6 (optional — only the GUI target requires it)
find_package(Qt6 COMPONENTS Core Widgets Charts QUIET
    PATHS /usr/lib/x86_64-linux-gnu/cmake)

# ── Vicon tracker (requires VRPN + Quat) ────────────────────────────────────
if (VRPN_LIB AND QUAT_LIB)
    include_directories(${PROJECT_SOURCE_DIR}/include)
    include_directories(${EIGEN3_INCLUDE_DIRS})
    include_directories(${zenohc_INCLUDE_DIRS})

    set(vicon_src src/vicon2pose.cpp src/vicon.cpp)

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

    add_executable(test_vicon_tracker src/main.cpp)
    target_link_libraries(test_vicon_tracker PRIVATE
        vicon_tracker_static
        ${VRPN_LIB}
        ${QUAT_LIB}
        zenohcxx::zenohc
        nlohmann_json::nlohmann_json
    )

    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        target_compile_options(test_vicon_tracker PRIVATE -Wall -O3 -Wno-class-memaccess)
    else()
        target_compile_options(test_vicon_tracker PRIVATE -Wall -O3)
    endif()

    message(STATUS "VRPN found — test_vicon_tracker target enabled.")
else()
    message(WARNING "VRPN or Quat library not found — test_vicon_tracker will NOT be built.")
endif()

# ── GUI target (requires Qt6 Core + Widgets + Charts) ────────────────────────
if (Qt6_FOUND)
    set(CMAKE_AUTOMOC ON)
    set(CMAKE_AUTORCC ON)
    set(CMAKE_AUTOUIC ON)

    add_executable(vicon_gui
        src/gui_main.cpp
        src/vicon_gui.cpp
        include/vicon_gui.hpp
    )

    target_include_directories(vicon_gui PRIVATE
        ${PROJECT_SOURCE_DIR}/include
    )

    target_link_libraries(vicon_gui PRIVATE
        Qt6::Core
        Qt6::Widgets
        Qt6::Charts
        zenohcxx::zenohc
        nlohmann_json::nlohmann_json
        Eigen3::Eigen
    )

    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        target_compile_options(vicon_gui PRIVATE -Wall -O2 -Wno-class-memaccess)
    else()
        target_compile_options(vicon_gui PRIVATE -Wall -O2)
    endif()

    message(STATUS "Qt6 found (${Qt6_VERSION}) — vicon_gui target enabled.")
else()
    message(WARNING "Qt6 not found — vicon_gui target will NOT be built.")
endif()