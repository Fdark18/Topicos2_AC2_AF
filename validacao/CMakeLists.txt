cmake_minimum_required(VERSION 3.16)
project(esteira_validacao C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(SIM_SOURCES
    src/freertos_sim.c
    src/validation_main.c
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Src/base_node.c
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Src/camera_node.c
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Src/conveyor_node.c
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Src/piston_node.c
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Src/safety_node.c
)

add_executable(esteira_sim ${SIM_SOURCES})

target_include_directories(esteira_sim PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
    ${CMAKE_CURRENT_LIST_DIR}/../Core/Inc
)

if(UNIX AND NOT APPLE)
    target_link_libraries(esteira_sim PRIVATE pthread m)
elseif(WIN32)
    target_link_libraries(esteira_sim PRIVATE pthreadVC2)
endif()
