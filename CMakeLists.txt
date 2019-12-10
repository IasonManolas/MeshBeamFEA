cmake_minimum_required(VERSION 2.8)
project(MeshBeamFEA)

set(CMAKE_CXX_STANDARD 17)
if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
endif(MSVC)

find_package(Eigen3 3.3 REQUIRED)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/ext/libigl)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/include
	${CMAKE_CURRENT_SOURCE_DIR}/ext/vcglib/
	${CMAKE_CURRENT_SOURCE_DIR}/ext/nlohmann/
        ${CMAKE_CURRENT_SOURCE_DIR}/ext/libigl/external
	)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp)
file(GLOB EXT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ext/vcglib/wrap/ply/plylib.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCES} ${EXT_SOURCES})

target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui Eigen3::Eigen
)
