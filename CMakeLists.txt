cmake_minimum_required(VERSION 2.8)
project(MeshBeamFEA)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(LIBIGL_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/ext/libigl/include)
set(CMAKE_CXX_STANDARD 17)
if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
endif(MSVC)

find_package(Eigen3 3.3 REQUIRED)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
find_package(LIBIGL REQUIRED)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/include
	${CMAKE_CURRENT_SOURCE_DIR}/ext/vcglib/
	${CMAKE_CURRENT_SOURCE_DIR}/ext/nlohmann/
        ${CMAKE_CURRENT_SOURCE_DIR}/ext/libigl/external
        ${CMAKE_CURRENT_SOURCE_DIR}/ext/gsl/
	)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB EXT_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ext/threed-beam-fea/src/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ext/vcglib/wrap/ply/plylib.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCES} ${EXT_SOURCES})

target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui Eigen3::Eigen
)
