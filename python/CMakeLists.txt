find_package(Python 3.9 COMPONENTS Interpreter Development.Module REQUIRED)
find_package(pybind11 3 CONFIG QUIET)

if(NOT pybind11_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG v3.0.4
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(pybind11)
endif()

pybind11_add_module(helix_python MODULE bindings.cpp)
target_link_libraries(helix_python PRIVATE helix::helix)
target_compile_features(helix_python PRIVATE cxx_std_20)
target_compile_options(helix_python PRIVATE -Wall -Wextra -Wpedantic)
target_compile_definitions(helix_python PRIVATE HELIX_VERSION="${PROJECT_VERSION}")
set_target_properties(helix_python PROPERTIES OUTPUT_NAME helix)

# scikit-build-core collects installed targets into the wheel. RUNTIME is needed on Windows.
install(TARGETS helix_python LIBRARY DESTINATION . RUNTIME DESTINATION .)
