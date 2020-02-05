cmake_minimum_required(VERSION 2.8)
project(MeshBeamFEA)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(CMAKE_CXX_STANDARD 17)
if(MSVC)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
endif(MSVC)

if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

include(${CMAKE_MODULE_PATH}/DownloadProject.cmake)

download_project(PROJ                threed-beam-fea
                 GIT_REPOSITORY      https://github.com/IasonManolas/threed-beam-fea.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
add_subdirectory(${threed-beam-fea_SOURCE_DIR} ${threed-beam-fea_BINARY_DIR})
message(STATUS ${threed-beam-fea_BINARY_DIR})

download_project(PROJ		    vcglib_devel
                 GIT_REPOSITORY     https://github.com/cnr-isti-vclab/vcglib.git
                 GIT_TAG	    devel
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

download_project(PROJ               nlohmann_json
                 GIT_REPOSITORY     https://github.com/nlohmann/json.git
                 GIT_TAG	    456478b3c50d60100dbb1fb9bc931f370a2c1c28
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

download_project(PROJ               GSL
                 GIT_REPOSITORY     https://github.com/microsoft/GSL.git
                 GIT_TAG	    b4dd39615aebb52dd73b6b1dfabd41b5e96c6962
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

download_project(PROJ               LIBIGL
                 GIT_REPOSITORY     https://github.com/libigl/libigl.git
                 GIT_TAG	    	master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

find_package(Eigen3 3.3 REQUIRED)
set(LIBIGL_INCLUDE_DIR ${LIBIGL_SOURCE_DIR}/include)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
find_package(LIBIGL)

include_directories(
        ${threed-beam-fea_SOURCE_DIR}/include
        ${threed-beam-fea_SOURCE_DIR}/ext/boost-1.65.1
        ${vcglib_devel_SOURCE_DIR}
        ${nlohmann_json_SOURCE_DIR}/include
        ${LIBIGL_INCLUDE_DIR}
        ${GSL_SOURCE_DIR}/include
	)

file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB EXT_SOURCES
        ${vcglib_devel_SOURCE_DIR}/wrap/ply/plylib.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCES} ${EXT_SOURCES})

target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui Eigen3::Eigen threed_beam_fea
)


