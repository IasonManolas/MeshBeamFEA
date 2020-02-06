cmake_minimum_required(VERSION 2.8)
project(MeshBeamFEA)

#Add the project cmake scripts to the module path
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#Download external dependencies NOTE: If the user has one of these libs it shouldn't be downloaded again.
include(${CMAKE_MODULE_PATH}/DownloadProject.cmake)
if (CMAKE_VERSION VERSION_LESS 3.2)
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "")
else()
    set(UPDATE_DISCONNECTED_IF_AVAILABLE "UPDATE_DISCONNECTED 1")
endif()

##Threed-beam-fea
download_project(PROJ                threed-beam-fea
                 GIT_REPOSITORY      https://github.com/IasonManolas/threed-beam-fea.git
                 GIT_TAG             master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

###add_subdirectory for using the generated threed_beam_fea
add_subdirectory(${threed-beam-fea_SOURCE_DIR} ${threed-beam-fea_BINARY_DIR})

##vcglib devel branch
download_project(PROJ		    vcglib_devel
                 GIT_REPOSITORY     https://github.com/cnr-isti-vclab/vcglib.git
                 GIT_TAG	    devel
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

##GSL library
download_project(PROJ               GSL
                 GIT_REPOSITORY     https://github.com/microsoft/GSL.git
                 GIT_TAG	    b4dd39615aebb52dd73b6b1dfabd41b5e96c6962
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)

##LIBIGL
download_project(PROJ               LIBIGL
                 GIT_REPOSITORY     https://github.com/libigl/libigl.git
                 GIT_TAG	    	master
                 ${UPDATE_DISCONNECTED_IF_AVAILABLE}
)
set(LIBIGL_INCLUDE_DIR ${LIBIGL_SOURCE_DIR}/include)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
find_package(LIBIGL)

##Eigen 3 NOTE: Eigen is required on the system the code is ran
find_package(Eigen3 3.3 REQUIRED)

##nlohmann_json use the source found in the external folder. This library is not downloaded during run time because we only use one header and the repository is quite big.
set(nlohmann_json_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/nlohmann)

#Add the project sources
file(GLOB SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/*.hpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
# TODO: EXT_SOURCES should be removed in the future
file(GLOB EXT_SOURCES ${vcglib_devel_SOURCE_DIR}/wrap/ply/plylib.cpp
        )

add_executable(${PROJECT_NAME} ${SOURCES} ${EXT_SOURCES})

target_include_directories(${PROJECT_NAME}
        PRIVATE ${threed-beam-fea_SOURCE_DIR}/include
        PRIVATE ${threed-beam-fea_SOURCE_DIR}/ext/boost-1.65.1
        PRIVATE ${vcglib_devel_SOURCE_DIR}
        PRIVATE ${nlohmann_json_SOURCE_DIR}
        PRIVATE ${LIBIGL_INCLUDE_DIR}
        PRIVATE ${GSL_SOURCE_DIR}/include
        )


#Use C++17
if(MSVC)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
else(MSVC)
        target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
endif(MSVC)

target_link_libraries(${PROJECT_NAME} igl::core igl::opengl_glfw igl::opengl_glfw_imgui Eigen3::Eigen threed_beam_fea
)


