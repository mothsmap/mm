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
   message(STATUS "Add definitions for unix build")
    add_definitions(
    # Skia preprocessors
   -DSK_INTERNAL
   -DSK_GAMMA_SRGB
   -DSK_GAMMA_APPLY_TO_A8
   -DSK_SCALAR_TO_FLOAT_EXCLUDED
   -DSK_ALLOW_STATIC_GLOBAL_INITIALIZERS=1
   -DSK_SUPPORT_GPU=1
   -DSK_SUPPORT_OPENCL=0
   -DSK_FORCE_DISTANCEFIELD_FONTS=0
   -DSK_PICTURE_USE_SK_RECORD
   -DSK_PICTURE_OPTIMIZE_SK_RECORD
   -DSK_SCALAR_IS_FLOAT
   -DSK_SAMPLES_FOR_X
   -DSK_BUILD_FOR_UNIX
   -DSK_USE_POSIX_THREADS
   -DSK_BUILD_POPPLER
   -DSK_SYSTEM_ZLIB=1
   -DSK_SUPPORT_PDF
   -DNDEBUG
   )
elseif(WIN32)
  set(CMAKE_CXX_FLAGS "-fexception")          
  add_definitions(
    # Skia preprocessors
   -DSK_INTERNAL
   -DSK_GAMMA_SRGB
   -DSK_GAMMA_APPLY_TO_A8
   -DSK_SCALAR_TO_FLOAT_EXCLUDED
   -DSK_ALLOW_STATIC_GLOBAL_INITIALIZERS=1
   -DSK_SUPPORT_GPU=1
   -DSK_SCALAR_IS_FLOAT
   -DSK_BUILD_FOR_WIN32
   -DNDEBUG
   )
endif()

# Find dependences
set(EXT_LIBRIES_DIR "${CMAKE_SOURCE_DIR}/3rd")
message(STATUS "EXT LIBRIES DIR:${EXT_LIBRIES_DIR}")

# Boost
set(Boost_USE_STATIC_LIBS 	 ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)

find_package(Boost REQUIRED system thread filesystem date_time program_options log chrono)

# wxWidgets
if(WIN32)
  set(wxWidgets_ROOT_DIR CACHE PATH "wxWdigets root directory")
  set(wxWidgets_LIB_DIR CACHE PATH "wxWdigets lib directory")
endif()
find_package(wxWidgets COMPONENTS core base REQUIRED)
include(${wxWidgets_USE_FILE})
message(STATUS "wxWidgets libs:${wxWidgets_LIBRARIES}")

# mapping
set(MAPPING_INCLUDE "${CMAKE_SOURCE_DIR}/3rd/mapping/include")
if(WIN32)
  set(MAPPING_LIB "${CMAKE_SOURCE_DIR}/3rd/mapping/lib/mapping_core.lib")
else()
    set(MAPPING_LIB "${CMAKE_SOURCE_DIR}/3rd/mapping/lib/libmapping_core.a")
endif()


# GDAL
find_package(GDAL REQUIRED)
#message(STATUS "GDAL INCLUDE:${GDAL_INCLUDE_DIR}")
#message(STATUS "GDAL LIBS:${GDAL_LIBRARY}")

# GEOS
find_package(GEOS REQUIRED)
#message(STATUS "GEOS INCLUDE:${GEOS_INCLUDE_DIR}")
#message(STATUS "GEOS LIBS:${GEOS_LIBRARY}")

# Skia
set(SKIA_ROOT CACHE PATH "Skia root directory")
include(cmake/modules/FindSkia.cmake)

# protobuf
# find_package(Protobuf REQUIRED)
if(UNIX)
    message(STATUS "set protobuf library")
    set(PROTOBUF_INCLUDE_DIRS "/usr/local/include")
    set(PROTOBUF_LITE_LIBRARY "/usr/local/lib/libprotobuf-lite.a")
else()
    set(PROTOBUF_INCLUDE_DIRS CACHE PATH "Protobuf source directory")
    set(PROTOBUF_LITE_LIBRARY CACHE FILEPATH "Protobuf library, e.g. path_to/#libprotobuf-lite.lib or libprotobuf-lite.a")
endif()

if (UNIX)
    # freeimage
    find_package(FreeImage REQUIRED)
    # glew
    find_package(GLEW REQUIRED)

    # libtess2
    set(TESS2_INCLUDES "")
    set(TESS2_LIB "${EXT_LIBRIES_DIR}/libtess2/liblibtess2.a")

    # glfw
    set(GLFW_INCLUDES "")
    set(GLFW_LIB "${EXT_LIBRIES_DIR}/glfw3/libglfw3.a")
endif()


include_directories(
${PROTOBUF_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${GDAL_INCLUDE_DIR}
${GEOS_INCLUDE_DIR}
${SKIA_INCLUDE_DIRS}
${GLEW_INCLUDE_DIRS}
${FreeImage_INCLUDE_DIRS}
${EXT_LIBRIES_DIR}/rapidjson/include/
${CMAKE_SOURCE_DIR}/mapping_core/
)

set(COMMOM_LIBS
${GEOS_LIBRARY}
${GDAL_LIBRARY}
${Boost_LIBRARIES}
${SKIA_LIBRARIES}
${PROTOBUF_LITE_LIBRARY}
${MAPPING_LIB}
${wxWidgets_LIBRARIES}
${TESS2_LIB}
${GLFW_LIB}
${GLEW_LIBRARIES}
${FreeImage_LIBRARIES}
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
${MAPPING_INCLUDE}
${CMAKE_SOURCE_DIR}/mm_studio/algorithm/
${CMAKE_SOURCE_DIR}/mm_studio/frame/
)

set(frame_src
  mm_studio/frame/app.cpp
  mm_studio/frame/frame.h
  mm_studio/frame/frame.cpp
  mm_studio/frame/gui_job.h
  mm_studio/frame/image_panel.h
  mm_studio/frame/image_panel.cpp
)

set(algorithm_src
mm_studio/algorithm/boost_geometry.h
mm_studio/algorithm/debug_utility.h
mm_studio/algorithm/route.h
mm_studio/algorithm/route.cpp
mm_studio/algorithm/rtree.h
mm_studio/algorithm/rtree.cpp
mm_studio/algorithm/shapefile_graph.h
mm_studio/algorithm/shapefile_graph.cpp
mm_studio/algorithm/mm_density.h
mm_studio/algorithm/mm_density.cpp
mm_studio/algorithm/mm_model.h
mm_studio/algorithm/mm_model.cpp

mm_studio/algorithm/rl_utility.h
mm_studio/algorithm/rl_utility.cpp
mm_studio/algorithm/rl_algorithm.h
mm_studio/algorithm/rl_algorithm.cpp
mm_studio/algorithm/scale_model.h
mm_studio/algorithm/scale_model.cpp
  )

add_library(mm STATIC ${algorithm_src})

add_executable(mm_studio
  ${frame_src}
)

add_executable(mm_test
  mm_test/test.cpp
)

target_link_libraries(mm ${COMMON_LIBS})
target_link_libraries(mm_studio mm ${COMMOM_LIBS})
target_link_libraries(mm_test mm ${COMMOM_LIBS})

source_group(frame FILES ${frame_src})
source_group(algorithm FILES ${algorithm_src})
