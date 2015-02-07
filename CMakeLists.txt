cmake_minimum_required(VERSION 2.8)

project (mm)

option(BUILD_TESTS "build tests" OFF)

# Preprocessors
add_definitions(-D_USE_MATH_DEFINES)
set (CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11")

# Find dependences
set(EXT_LIBRIES_DIR "${CMAKE_SOURCE_DIR}/3rd")

# Boost
set(Boost_USE_STATIC_LIBS 	 ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)

find_package(Boost REQUIRED system thread filesystem date_time timer program_options log chrono serialization unit_test_framework)

# GDAL
find_package(GDAL REQUIRED)

include_directories(${Boost_INCLUDE_DIRS} ${GDAL_INCLUDE_DIR} ${EXT_LIBRIES_DIR}/ ${CMAKE_SOURCE_DIR}/src/)

set(COMMOM_LIBS ${GDAL_LIBRARY} ${Boost_LIBRARIES} )

set(algorithm_src
  src/geometry.h
  src/debug.h
  src/mm.h
  src/mm.cpp
  src/mm_route.h
  src/mm_route.cpp
  src/mm_tree.h
  src/mm_tree.cpp
  src/mm_graph.h
  src/mm_graph.cpp
  src/mm_density_solver.h
  src/mm_density_solver.cpp
  src/mm_sparse_solver.h
  src/mm_sparse_solver.cpp

  src/rl_utility.h
  src/rl_utility.cpp
  src/rl.h
  src/rl.cpp
  src/scale_model.h
  src/scale_model.cpp
  )

add_library(mm STATIC ${algorithm_src})
add_executable(mm_prepare src/mm_prepare.cpp)
add_executable(mm_match src/mm_match.cpp)

target_link_libraries(mm ${COMMON_LIBS})
target_link_libraries(mm_prepare mm ${COMMOM_LIBS})
target_link_libraries(mm_match mm ${COMMOM_LIBS})

if(BUILD_TESTS)
  add_executable(mm_test test/test_density_solver.cpp test/test_sparse_solver.cpp)
  target_link_libraries(mm_test mm ${COMMOM_LIBS})
endif()