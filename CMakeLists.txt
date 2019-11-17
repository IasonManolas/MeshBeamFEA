cmake_minimum_required(VERSION 2.8)
project(MeshBeamFEA)

find_package(Eigen3 3.3 REQUIRED)
file(GLOB SOURCES "src/*.cpp" ${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/src/*.cpp
                  )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/include
                    ${EIGEN3_INCLUDE_DIR}
                    )
add_executable(${PROJECT_NAME} ${SOURCES})
