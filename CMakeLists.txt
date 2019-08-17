cmake_minimum_required(VERSION 3.1)
project(RoboticSimulator)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(P ${CMAKE_BINARY_DIR}/glad/$(ConfigurationName)/glad3.lib)

# libigl
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use IMGUI"          ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_USE_STATIC_LIBRARY     "Use Static lib"     ON)


MESSAGE (STATUS ${CMAKE_BINARY_DIR}/glad/$(ConfigurationName)/)
MESSAGE (STATUS ${P}/glad/$(ConfigurationName)/)
MESSAGE ("---------")

find_package(LIBIGL REQUIRED QUIET)

# Add your project files
# file(GLOB SRCFILES *.cpp)

file(GLOB SRCFILES
		${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/include/*.h
	)

include_directories(${CMAKE_SOURCE_DIR}/../libigl/external)
include_directories(${CMAKE_SOURCE_DIR}/../libigl/external/eigen)
include_directories(${CMAKE_SOURCE_DIR}/../libigl/include)
include_directories(${CMAKE_SOURCE_DIR}/../libigl/external/glfw/include)
include_directories(${CMAKE_SOURCE_DIR}/../libigl/external/glad/include)
include_directories(${CMAKE_SOURCE_DIR}/../libigl/external/imgui/examples)
include_directories(${CMAKE_SOURCE_DIR}/../libigl/external/libigl-imgui)


add_executable(${PROJECT_NAME}_bin ${SRCFILES})

MESSAGE (STATUS ${CMAKE_BINARY_DIR}/glad/$(ConfigurationName)/)
MESSAGE (STATUS ${P}/glad/$(ConfigurationName)/)

set(BIN_F C:/ReposBuild/RoboticSimulator/)

target_link_libraries(${PROJECT_NAME}_bin 
	${BIN_F}glad/$(ConfigurationName)/glad.lib
	${BIN_F}glfw/src/$(ConfigurationName)/glfw3.lib
	${BIN_F}$(ConfigurationName)/igl.lib
	${BIN_F}$(ConfigurationName)/igl_opengl.lib
	${BIN_F}$(ConfigurationName)/igl_opengl_glfw.lib
	${BIN_F}$(ConfigurationName)/igl_opengl_glfw_imgui.lib
	${BIN_F}imgui/$(ConfigurationName)/imgui.lib
	
	)

