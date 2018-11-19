cmake_minimum_required(VERSION 3.8)
project(Bachelor)

add_subdirectory(visualizer)

set(core_src bachelor_solution_src)
include_directories(${core_src})

add_executable(Bachelor main.cpp
						${core_src}/ElevationMap.cxx
						${core_src}/ShortestPathSolver.cxx
						${core_src}/GeometricTools.cxx
						${core_src}/GeometricTools.txx)
target_link_libraries(Bachelor visualizer)

add_custom_command(
    TARGET Bachelor
    POST_BUILD COMMAND
        ${CMAKE_COMMAND} -E copy_directory
        "${CMAKE_CURRENT_SOURCE_DIR}/assets"
        $<TARGET_FILE_DIR:Bachelor>/assets)





