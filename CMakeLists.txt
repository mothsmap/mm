cmake_minimum_required(VERSION 2.8)
project (mm)

# Preprocessors
add_definitions(-D_USE_MATH_DEFINES)
set (CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11")

# Extra find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# Compile flags
if (APPLE)
    set(CMAKE_CXX_FLAGS "-std=c++11 -stdlib=libc++ -mfpmath=sse -g -m64 -O3")
    add_definitions(-DNDEBUG)
elseif(UNIX)
  set(CMAKE_CXX_FLAGS "-std=c++11 -mfpmath=sse -g -m64 -O3")        
elseif(WIN32)
  set(CMAKE_CXX_FLAGS "-fexception") 
endif()

# Find dependences
set(EXT_LIBRIES_DIR "${CMAKE_SOURCE_DIR}/3rd")
message(STATUS "EXT LIBRIES DIR:${EXT_LIBRIES_DIR}")

# Boost
set(Boost_USE_STATIC_LIBS 	 ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)

find_package(Boost REQUIRED system thread filesystem date_time timer program_options log chrono serialization)


# GDAL
find_package(GDAL REQUIRED)
#message(STATUS "GDAL INCLUDE:${GDAL_INCLUDE_DIR}")
#message(STATUS "GDAL LIBS:${GDAL_LIBRARY}")

# GEOS
find_package(GEOS REQUIRED)
#message(STATUS "GEOS INCLUDE:${GEOS_INCLUDE_DIR}")
#message(STATUS "GEOS LIBS:${GEOS_LIBRARY}")

include_directories(
${Boost_INCLUDE_DIRS}
${GDAL_INCLUDE_DIR}
${GEOS_INCLUDE_DIR}
${SKIA_INCLUDE_DIRS}
${EXT_LIBRIES_DIR}/rapidjson/include/
${CMAKE_SOURCE_DIR}/mapping_core/
)

set(COMMOM_LIBS
${GEOS_LIBRARY}
${GDAL_LIBRARY}
${Boost_LIBRARIES}
"-framework OpenGL"
"-framework Foundation"
"-framework Cocoa"
"-framework CoreVideo"
)

message(STATUS "CMAKE_SOURCE_DIR:${CMAKE_SOURCE_DIR}")
include_directories(
${Boost_INCLUDE_DIRS}
${EXT_LIBRIES_DIR}/
${EXT_LIBRIES_DIR}/rapidjson/include/
${CMAKE_SOURCE_DIR}/mm_studio/algorithm/
${CMAKE_SOURCE_DIR}/mm_studio/frame/
)

set(algorithm_src
mm_studio/algorithm/geometry.h
mm_studio/algorithm/debug.h
mm_studio/algorithm/mm_route.h
mm_studio/algorithm/mm_route.cpp
mm_studio/algorithm/mm_tree.h
mm_studio/algorithm/mm_tree.cpp
mm_studio/algorithm/mm_graph.h
mm_studio/algorithm/mm_graph.cpp
mm_studio/algorithm/mm_density_solver.h
mm_studio/algorithm/mm_density_solver.cpp
mm_studio/algorithm/mm_sparse_solver.h
mm_studio/algorithm/mm_sparse_solver.cpp

mm_studio/algorithm/rl_utility.h
mm_studio/algorithm/rl_utility.cpp
mm_studio/algorithm/rl.h
mm_studio/algorithm/rl.cpp
mm_studio/algorithm/scale_model.h
mm_studio/algorithm/scale_model.cpp
  )

add_library(mm STATIC ${algorithm_src})
add_executable(mm_prepare mm_prepare/mm_prepare.cpp)
add_executable(mm_match mm_match/mm_match.cpp)
add_executable(mm_test_density_solver mm_test/test_density_solver.cpp)
add_executable(mm_test_sparse_solver mm_test/test_sparse_solver.cpp)

target_link_libraries(mm ${COMMON_LIBS})
target_link_libraries(mm_test_density_solver mm ${COMMOM_LIBS})
target_link_libraries(mm_test_sparse_solver mm ${COMMOM_LIBS})
target_link_libraries(mm_prepare mm ${COMMOM_LIBS})
target_link_libraries(mm_match mm ${COMMOM_LIBS})
