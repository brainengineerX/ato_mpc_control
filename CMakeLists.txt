cmake_minimum_required(VERSION 3.16)

project(ato_control_algorithm_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(ato_control_algorithm
    src/auto_adjust.cpp
    src/control_loop.cpp
    src/evaluator.cpp
    src/linalg.cpp
    src/mpc_controller.cpp
    src/output_transform.cpp
    src/constraint_provider.cpp
    src/parking_brake.cpp
    src/sqpmethod.cpp
    src/target_speed.cpp
    src/vehicle_plant.cpp
)

target_include_directories(ato_control_algorithm
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(ato_control_algorithm
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

# MPC 模块单元测试
add_executable(ato_mpc_unit_tests tests/test_mpc_units.cpp)
target_link_libraries(ato_mpc_unit_tests PRIVATE ato_control_algorithm)

# CLI 工具
add_executable(ato_sim_cli tools/ato_sim_cli.cpp)
target_link_libraries(ato_sim_cli PRIVATE ato_control_algorithm)

enable_testing()
add_test(NAME ato_mpc_unit_tests COMMAND ato_mpc_unit_tests)
