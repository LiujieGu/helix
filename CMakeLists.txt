cmake_minimum_required(VERSION 3.20)
project(helix LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# --- dependencies ---
include(FetchContent)

# Eigen (header-only)
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG        3.4.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(eigen)
mark_as_advanced(FORCE eigen eigen_BINARY_DIR eigen_SOURCE_DIR)

# OSQP
set(OSQP_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(OSQP_ALGEBRA_DEFAULT "mkl" CACHE STRING "" FORCE)
FetchContent_Declare(
  osqp
  GIT_REPOSITORY https://github.com/osqp/osqp.git
  GIT_TAG        v1.0.0
  GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(osqp)
target_compile_definitions(osqpstatic PRIVATE OSQP_ALGEBRA_NATIVE)

# --- helix library ---
set(HELIX_SOURCES
  src/helix/osqp_solver.cpp
)

add_library(helix_core INTERFACE)
target_include_directories(helix_core INTERFACE src)
target_link_libraries(helix_core INTERFACE Eigen3::Eigen)

add_library(helix STATIC ${HELIX_SOURCES})
target_include_directories(helix PUBLIC src)
target_link_libraries(helix PUBLIC helix_core osqpstatic)
target_include_directories(helix PUBLIC ${osqp_SOURCE_DIR}/include)

# --- executable (demo) ---
add_executable(helix_app src/main.cpp)
target_link_libraries(helix_app PRIVATE helix)

# --- tests ---
enable_testing()
add_subdirectory(tests)
