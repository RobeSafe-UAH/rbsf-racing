cmake_minimum_required(VERSION 3.16)
project(range_libc CXX)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Module NumPy)
find_program(CYTHON_EXECUTABLE NAMES cython3 cython REQUIRED)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Python module `range_libc`: the Cython wrapper (pywrapper/RangeLibc.pyx) around the header-only
# C++ library in includes/. CPU ray casting only (bl, rm, cddt/pcddt, glt): the CUDA sources were
# not vendored (PORTING.md).
set(pyx ${CMAKE_CURRENT_SOURCE_DIR}/pywrapper/RangeLibc.pyx)
set(cpp ${CMAKE_CURRENT_BINARY_DIR}/RangeLibc.cpp)
add_custom_command(
  OUTPUT ${cpp}
  COMMAND ${CYTHON_EXECUTABLE} --cplus -3 --module-name range_libc -o ${cpp} ${pyx}
  DEPENDS ${pyx} ${CMAKE_CURRENT_SOURCE_DIR}/includes/RangeLib.h
  COMMENT "Cythonizing RangeLibc.pyx")

Python3_add_library(range_libc MODULE WITH_SOABI ${cpp} vendor/lodepng/lodepng.cpp)
target_include_directories(range_libc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(range_libc PRIVATE Python3::NumPy)
target_compile_definitions(range_libc PRIVATE NPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION)
target_compile_options(range_libc PRIVATE -O3 -w)

ament_get_python_install_dir(python_install_dir)
install(TARGETS range_libc LIBRARY DESTINATION ${python_install_dir})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_range_libc test/test_range_libc.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR})
endif()

ament_package()
