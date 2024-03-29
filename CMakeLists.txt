cmake_minimum_required(VERSION 3.1)
project(mylar)

#Versioning
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#Tests for Clang and AppleClang which is defined in CMake 3.0 and higher
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message("Clang.")
#  set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++ -Wall -g")
  set(CMAKE_CXX_FLAGS "-std=c++17 -stdlib=libc++ -Wall -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  #    set(CMAKE_CXX_FLAGS_DEBUG "-g")
  set(OPENMP OFF CACHE BOOL "OpenMP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
  message("GCC.")
#  set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -g")
  set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-Ofast -DNDEBUG")
  #    set(CMAKE_CXX_FLAGS_DEBUG "-g")
  set(OPENMP ON CACHE BOOL "OpenMP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Intel")
  message("ICC.")
  if(WIN32)
    set(CMAKE_CXX_FLAGS "/Qstd=c++11")
    set(CMAKE_CXX_FLAGS_RELEASE "/O3 /DNDEBUG")
  else()
    set(CMAKE_CXX_FLAGS "-std=c++11")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  endif()
  set(OPENMP OFF CACHE BOOL "OpenMP")
elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  message("MSVC.")
  set(CMAKE_CXX_FLAGS_DEBUG "/MTd")
  set(CMAKE_CXX_FLAGS_RELEASE "/MT /Ox /Ot /fp:fast /GS- /GL /DNDEBUG")
  set(CMAKE_EXE_LINKER_FLAGS_RELEASE "/LTCG /INCREMENTAL:NO /OPT:REF")
  set(OPENMP ON CACHE BOOL "OpenMP")
else()
  message( FATAL_ERROR "CXX compiler not recognized. CMake will quit." )
endif()

# libigl
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" OFF)
option(LIBIGL_WITH_ANTTWEAKBAR       "Use AntTweakBar"    OFF)
option(LIBIGL_WITH_CGAL              "Use CGAL"           OFF)
option(LIBIGL_WITH_COMISO            "Use CoMiso"         OFF)
option(LIBIGL_WITH_CORK              "Use Cork"           OFF)
option(LIBIGL_WITH_EMBREE            "Use Embree"         OFF)
option(LIBIGL_WITH_LIM               "Use LIM"            OFF)
option(LIBIGL_WITH_MATLAB            "Use Matlab"         OFF)
option(LIBIGL_WITH_MOSEK             "Use MOSEK"          OFF)
option(LIBIGL_WITH_OPENGL            "Use OpenGL"         ON)
option(LIBIGL_WITH_OPENGL_GLFW       "Use GLFW"           ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use ImGui"          ON)
option(LIBIGL_WITH_PNG               "Use PNG"            OFF)
option(LIBIGL_WITH_PYTHON            "Use Python"         OFF)
option(LIBIGL_WITH_TETGEN            "Use Tetgen"         ON)
option(LIBIGL_WITH_TRIANGLE          "Use Triangle"       ON)
option(LIBIGL_WITH_VIEWER            "Use OpenGL viewer"  ON)
option(LIBIGL_WITH_XML               "Use XML"            OFF)

find_package(LIBIGL REQUIRED QUIET)

include_directories("./include/")
include_directories("./include/boost/")
#include_directories("./lib/libigl/external/triangle")

# ============================= linking ===============================

# JZ: use llvm Clang to use openmp
#set(CMAKE_CXX_COMPILER "/usr/local/opt/llvm/bin/clang")
# enable openmp, enable c++ std library, enable o3 optimization level
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -lc++ -o3")
# link openmp in llvm
# TODO: link_directories should be avoided: https://cmake.org/cmake/help/latest/command/link_directories.html
#link_directories("/usr/local/opt/llvm/lib")

add_subdirectory(lib/libShapeOp)
include_directories(lib/libShapeOp/src)

add_library(model SHARED model.cpp Utils.cpp Utils.h)
target_compile_definitions(model PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(model  igl::core igl::opengl_glfw_imgui ShapeOp)

add_library(preprocessor SHARED Preprocessor.cpp)
target_link_libraries(preprocessor igl::triangle)

add_executable(${PROJECT_NAME}  main.cpp)
target_compile_definitions(${PROJECT_NAME} PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(${PROJECT_NAME}  igl::core igl::opengl_glfw_imgui ShapeOp)
target_link_libraries(${PROJECT_NAME}  model)
target_link_libraries(${PROJECT_NAME}  preprocessor)


add_executable(batch  batch.cpp)
target_compile_definitions(batch PRIVATE ROOT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(batch  igl::core igl::opengl_glfw_imgui ShapeOp)
target_link_libraries(batch  model)
target_link_libraries(batch  preprocessor)

