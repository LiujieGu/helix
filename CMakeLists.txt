cmake_minimum_required(VERSION 3.20)
project(helix VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(HELIX_BUILD_TESTS "Build helix tests" ON)
option(HELIX_BUILD_EXAMPLE "Build the helix example" ON)

include(FetchContent)

# Keep Eigen from registering hundreds of unrelated tests without changing a parent project's
# BUILD_TESTING choice when helix is embedded with add_subdirectory().
if(DEFINED BUILD_TESTING)
  set(HELIX_BUILD_TESTING_WAS_DEFINED TRUE)
  set(HELIX_SAVED_BUILD_TESTING "${BUILD_TESTING}")
else()
  set(HELIX_BUILD_TESTING_WAS_DEFINED FALSE)
endif()
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(eigen)
get_target_property(HELIX_EIGEN_TARGET Eigen3::Eigen ALIASED_TARGET)
if(NOT HELIX_EIGEN_TARGET)
  set(HELIX_EIGEN_TARGET Eigen3::Eigen)
endif()
set_property(TARGET ${HELIX_EIGEN_TARGET} PROPERTY INTERFACE_SYSTEM_INCLUDE_DIRECTORIES
             "${eigen_SOURCE_DIR}")
unset(HELIX_EIGEN_TARGET)
if(HELIX_BUILD_TESTING_WAS_DEFINED)
  set(BUILD_TESTING "${HELIX_SAVED_BUILD_TESTING}" CACHE BOOL "" FORCE)
else()
  unset(BUILD_TESTING CACHE)
endif()
unset(HELIX_BUILD_TESTING_WAS_DEFINED)
unset(HELIX_SAVED_BUILD_TESTING)

set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_ALGEBRA_BACKEND "builtin" CACHE STRING "" FORCE)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG v1.0.0
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(osqp)

add_library(helix STATIC
  src/helix/osqp_solver.cpp
  src/helix/portfolio.cpp
)
add_library(helix::helix ALIAS helix)
target_include_directories(helix
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${osqp_SOURCE_DIR}/include
)
target_link_libraries(helix PUBLIC Eigen3::Eigen PRIVATE osqpstatic)
target_compile_features(helix PUBLIC cxx_std_20)
target_compile_options(helix PRIVATE -Wall -Wextra -Wpedantic)

if(HELIX_BUILD_EXAMPLE)
  add_subdirectory(examples)
endif()

if(HELIX_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
