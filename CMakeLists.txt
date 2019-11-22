cmake_minimum_required(VERSION 2.8)
project(MeshBeamFEA)

find_package(Eigen3 3.3 REQUIRED)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/ext/vcglib/
                    ${EIGEN3_INCLUDE_DIR}
                    )

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB EXT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/src/*.cpp
                      ${CMAKE_CURRENT_SOURCE_DIR}/ext/vcglib/wrap/ply/plylib.cpp)

add_executable(${PROJECT_NAME} ${SOURCES} ${EXT_SOURCES})
