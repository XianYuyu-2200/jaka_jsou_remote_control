cmake_minimum_required(VERSION 3.20)
project(windows_jaka LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4 /EHsc /utf-8 /wd4005 /wd4819)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(WINDOWS_JAKA_TEST_ONLY "Build host-side tests without the vendor SDK" OFF)

add_compile_definitions(_WIN32_WINNT=0x0601 WIN32_LEAN_AND_MEAN NOMINMAX _CRT_SECURE_NO_WARNINGS)

set(WINDOWS_JAKA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include")

if(NOT WINDOWS_JAKA_TEST_ONLY)
    if(NOT JAKA_SDK_ROOT)
        message(FATAL_ERROR
            "Set -DJAKA_SDK_ROOT=<.../WINDOWS_MSVC_V142_x64> or use "
            "-DWINDOWS_JAKA_TEST_ONLY=ON for host-only tests")
    endif()

    find_path(JAKA_SDK_INCLUDE_DIR
        NAMES JAKAZuRobot.h
        HINTS
            "${JAKA_SDK_ROOT}/Windows/c&c++/inc_of_c++"
            "${JAKA_SDK_ROOT}/Windows/c&c++/inc_of_c"
            "${JAKA_SDK_ROOT}/Windows/WINDOWS_MSVC_V142_x64/Windows/c&c++/inc_of_c++"
            "${JAKA_SDK_ROOT}/Windows/WINDOWS_MSVC_V142_x64/Windows/c&c++/inc_of_c"
            "${JAKA_SDK_ROOT}/include"
            "${JAKA_SDK_ROOT}/inc_of_c++")

    find_library(JAKA_SDK_LIBRARY
        NAMES jakaAPI
        HINTS
            "${JAKA_SDK_ROOT}/Windows/c&c++/x64"
            "${JAKA_SDK_ROOT}/Windows/WINDOWS_MSVC_V142_x64/Windows/c&c++/x64"
            "${JAKA_SDK_ROOT}/x64"
            "${JAKA_SDK_ROOT}/lib")

    if(NOT JAKA_SDK_INCLUDE_DIR OR NOT JAKA_SDK_LIBRARY)
        message(FATAL_ERROR
            "JAKA Windows SDK not found. Expected JAKAZuRobot.h and jakaAPI.lib "
            "under ${JAKA_SDK_ROOT}")
    endif()

    get_filename_component(JAKA_SDK_LIB_DIR "${JAKA_SDK_LIBRARY}" DIRECTORY)
    find_file(JAKA_SDK_DLL
        NAMES jakaAPI.dll
        HINTS "${JAKA_SDK_LIB_DIR}"
              "${JAKA_SDK_ROOT}/Windows/c&c++/x64"
              "${JAKA_SDK_ROOT}/Windows/WINDOWS_MSVC_V142_x64/Windows/c&c++/x64")
    if(NOT JAKA_SDK_DLL)
        message(FATAL_ERROR "jakaAPI.dll not found next to the Windows SDK library")
    endif()

    add_library(jaka_sdk INTERFACE)
    target_include_directories(jaka_sdk INTERFACE "${JAKA_SDK_INCLUDE_DIR}")
    target_link_libraries(jaka_sdk INTERFACE "${JAKA_SDK_LIBRARY}" ws2_32)

    add_executable(windows_operator src/windows_operator.cpp src/udp_transport.cpp)
    target_include_directories(windows_operator PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    target_link_libraries(windows_operator PRIVATE jaka_sdk)

    add_executable(windows_follower src/windows_follower.cpp src/udp_transport.cpp)
    target_include_directories(windows_follower PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    target_link_libraries(windows_follower PRIVATE jaka_sdk)

    add_executable(windows_teleop_gui WIN32 src/windows_teleop_gui.cpp)
    target_include_directories(windows_teleop_gui PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    target_link_libraries(windows_teleop_gui PRIVATE comctl32 shell32 user32 gdi32 gdiplus)

    set(JAKA_GUI_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/../江苏开放大学.png")
    if(EXISTS "${JAKA_GUI_LOGO}")
        add_custom_command(TARGET windows_teleop_gui POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${JAKA_GUI_LOGO}"
                "$<TARGET_FILE_DIR:windows_teleop_gui>/江苏开放大学.png")
    endif()

    # Only the SDK-linked binaries need the vendor DLL next to them. The GUI
    # links against comctl32/shell32/user32/gdi32 only, so copying the SDK DLL
    # there just makes GUI-only builds fail when the SDK path is unavailable.
    foreach(target windows_operator windows_follower)
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${JAKA_SDK_DLL}"
                "$<TARGET_FILE_DIR:${target}>/jakaAPI.dll")
    endforeach()
endif()

add_executable(windows_robot_manager WIN32 src/windows_robot_manager.cpp)
target_include_directories(windows_robot_manager PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
target_link_libraries(windows_robot_manager PRIVATE comctl32 user32 gdiplus)

set(JAKA_ROBOT_MANAGER_LOGO "${CMAKE_CURRENT_SOURCE_DIR}/../江苏开放大学.png")
if(EXISTS "${JAKA_ROBOT_MANAGER_LOGO}")
    add_custom_command(TARGET windows_robot_manager POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${JAKA_ROBOT_MANAGER_LOGO}"
            "$<TARGET_FILE_DIR:windows_robot_manager>/江苏开放大学.png")
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(packet_test tests/packet_test.cpp)
    target_include_directories(packet_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME packet_test COMMAND packet_test)

    add_executable(mapping_test tests/mapping_test.cpp)
    target_include_directories(mapping_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME mapping_test COMMAND mapping_test)

    add_executable(trajectory_test tests/trajectory_test.cpp)
    target_include_directories(trajectory_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME trajectory_test COMMAND trajectory_test)

    add_executable(robot_registry_test tests/robot_registry_test.cpp)
    target_include_directories(robot_registry_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME robot_registry_test COMMAND robot_registry_test)

    add_executable(runtime_plan_test tests/runtime_plan_test.cpp)
    target_include_directories(runtime_plan_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME runtime_plan_test COMMAND runtime_plan_test)

    add_executable(runtime_status_test tests/runtime_status_test.cpp)
    target_include_directories(runtime_status_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME runtime_status_test COMMAND runtime_status_test)

    add_executable(dry_run_test tests/dry_run_test.cpp)
    target_include_directories(dry_run_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
    add_test(NAME dry_run_test COMMAND dry_run_test)

    if(WIN32)
        add_executable(udp_loopback_test tests/udp_loopback_test.cpp src/udp_transport.cpp)
        target_include_directories(udp_loopback_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
        target_link_libraries(udp_loopback_test PRIVATE ws2_32)
        add_test(NAME udp_loopback_test COMMAND udp_loopback_test)

        add_executable(control_pipe_test tests/control_pipe_test.cpp)
        target_include_directories(control_pipe_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
        add_test(NAME control_pipe_test COMMAND control_pipe_test)

        add_executable(control_pipe_server_test tests/control_pipe_server_test.cpp)
        target_include_directories(control_pipe_server_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
        add_test(NAME control_pipe_server_test COMMAND control_pipe_server_test)

        add_executable(windows_clock_test tests/windows_clock_test.cpp)
        target_include_directories(windows_clock_test PRIVATE "${WINDOWS_JAKA_INCLUDE_DIR}")
        add_test(NAME windows_clock_test COMMAND windows_clock_test)
    endif()
endif()
