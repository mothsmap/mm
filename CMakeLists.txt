cmake_minimum_required(VERSION 2.8)
project (mm)

# Preprocessors
add_definitions(-D_USE_MATH_DEFINES)
set (CMAKE_CXX_FLAGS "-stdlib=libc++ -std=c++11")

# Extra find modules
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/modules/")

# Find dependences
set(EXT_LIBRIES_DIR "${CMAKE_SOURCE_DIR}/3rd")
message(STATUS "EXT LIBRIES DIR:${EXT_LIBRIES_DIR}")

# Boost
set(Boost_USE_STATIC_LIBS 	 ON)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME     OFF)

find_package(Boost REQUIRED system thread filesystem date_time program_options log chrono)

# wxWidgets
find_package(wxWidgets COMPONENTS core base REQUIRED)
include(${wxWidgets_USE_FILE})
message(STATUS "wxWidgets libs:${wxWidgets_LIBRARIES}")

# mapping
set(MAPPING_INCLUDE "${CMAKE_SOURCE_DIR}/3rd/mapping/include")
set(MAPPING_LIB "${CMAKE_SOURCE_DIR}/3rd/mapping/lib/libmapping_core.a")

# GDAL
find_package(GDAL REQUIRED)
#message(STATUS "GDAL INCLUDE:${GDAL_INCLUDE_DIR}")
#message(STATUS "GDAL LIBS:${GDAL_LIBRARY}")

# GEOS
find_package(GEOS REQUIRED)
#message(STATUS "GEOS INCLUDE:${GEOS_INCLUDE_DIR}")
#message(STATUS "GEOS LIBS:${GEOS_LIBRARY}")

# Skia
set(SKIA_ROOT /Users/xuxiang/Documents/workspace/src/3rd/skia)
set(Skia_INCLUDE_DIRS
"${SKIA_ROOT}/include/config"
"${SKIA_ROOT}/include/animator"
"${SKIA_ROOT}/include/core"
"${SKIA_ROOT}/include/device"
"${SKIA_ROOT}/include/effects"
"${SKIA_ROOT}/include/gpu"
"${SKIA_ROOT}/include/images"
"${SKIA_ROOT}/src/pathops"
"${SKIA_ROOT}/include/pdf"
"${SKIA_ROOT}/include/pipe"
"${SKIA_ROOT}/include/ports"
"${SKIA_ROOT}/include/record"
"${SKIA_ROOT}/include/svg"
"${SKIA_ROOT}/include/utils"
"${SKIA_ROOT}/include/views"
"${SKIA_ROOT}/include/xml"
"${SKIA_ROOT}/src/animator"
"${SKIA_ROOT}/src/core"
"${SKIA_ROOT}/src/device"
"${SKIA_ROOT}/src/effects"
"${SKIA_ROOT}/src/gpu"
"${SKIA_ROOT}/src/images"
"${SKIA_ROOT}/src/image"
"${SKIA_ROOT}/src/lazy"
"${SKIA_ROOT}/include/opts"
"${SKIA_ROOT}/include/pathops"
"${SKIA_ROOT}/src/pdf"
"${SKIA_ROOT}/src/pipe"
"${SKIA_ROOT}/src/ports"
"${SKIA_ROOT}/src/sfnt"
"${SKIA_ROOT}/src/svg"
"${SKIA_ROOT}/src/utils"
"${SKIA_ROOT}/src/views"
"${SKIA_ROOT}/src/xml"
)
#if(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
#  	set(SKIA_LIB_DIR ${SKIA_ROOT}/out/Debug)
#else()
    set(SKIA_LIB_DIR ${SKIA_ROOT}/out/Release)
#endif()

  set(Skia_LIBRARIES
    "${SKIA_LIB_DIR}/libcrash_handler.a"
    "${SKIA_LIB_DIR}/libetc1.a"
    "${SKIA_LIB_DIR}/libexperimental.a"
    "${SKIA_LIB_DIR}/libflags_common.a"
    "${SKIA_LIB_DIR}/libflags.a"
  "${SKIA_LIB_DIR}/libjpeg.a"
  "${SKIA_LIB_DIR}/libpng.a"
    #"${SKIA_LIB_DIR}/libjsoncpp.a"
    "${SKIA_LIB_DIR}/libpicture_utils.a"
    "${SKIA_LIB_DIR}/libresources.a"
    "${SKIA_LIB_DIR}/libsk_tool_utils.a"
    "${SKIA_LIB_DIR}/libskflate.a"
    "${SKIA_LIB_DIR}/libskia_core.a"
    "${SKIA_LIB_DIR}/libskia_effects.a"
    "${SKIA_LIB_DIR}/libskia_images.a"
    "${SKIA_LIB_DIR}/libskia_opts_sse4.a"
  "${SKIA_LIB_DIR}/libskia_opts_ssse3.a"
    "${SKIA_LIB_DIR}/libskia_opts.a"
    "${SKIA_LIB_DIR}/libskia_pdf.a"
    "${SKIA_LIB_DIR}/libskia_ports.a"
    "${SKIA_LIB_DIR}/libskia_sfnt.a"
    "${SKIA_LIB_DIR}/libskia_skgpu.a"
    "${SKIA_LIB_DIR}/libskia_skgputest.a"
    "${SKIA_LIB_DIR}/libskia_utils.a"
    "${SKIA_LIB_DIR}/libSKKTX.a"
    "${SKIA_LIB_DIR}/libwebp_dec.a"
    "${SKIA_LIB_DIR}/libwebp_demux.a"
    "${SKIA_LIB_DIR}/libwebp_dsp.a"
    "${SKIA_LIB_DIR}/libwebp_enc.a"
    "${SKIA_LIB_DIR}/libwebp_utils.a"
  )

message(STATUS "SKIA_ROOT: ${SKIA_ROOT}")
message(STATUS "SKIA_LIB_DIR: ${SKIA_LIB_DIR}")

# FreeImage
find_package(FreeImage REQUIRED)

#message(STATUS "FreeImage includes:${FreeImage_INCLUDE_DIRS}")
#message(STATUS "FreeImage libs:${FreeImage_LIBRARIES}")

# protobuf
# find_package(Protobuf REQUIRED)
set (PROTOBUF_INCLUDE_DIRS /usr/local/include)
set (PROTOBUF_LITE_LIBRARY /usr/local/lib/libprotobuf-lite.a)

# png
#find_package(PNG REQUIRED)
#message(STATUS "PNG_LIBRARIES libs:${PNG_LIBRARIES}")

# glew
find_package(GLEW REQUIRED)

# libtess2
set(TESS2_INCLUDES "")
set(TESS2_LIB "${EXT_LIBRIES_DIR}/libtess2/liblibtess2.a")

# glfw
set(GLFW_INCLUDES "")
set(GLFW_LIB "${EXT_LIBRIES_DIR}/glfw3/libglfw3.a")

include_directories(
${PROTOBUF_INCLUDE_DIRS}
${Boost_INCLUDE_DIRS}
${GDAL_INCLUDE_DIR}
${GEOS_INCLUDE_DIR}
${GLEW_INCLUDE_DIRS}
${Skia_INCLUDE_DIRS}
${FreeImage_INCLUDE_DIRS}
${EXT_LIBRIES_DIR}/rapidjson/include/
${CMAKE_SOURCE_DIR}/mapping_core/
)

set(COMMOM_LIBS
${GEOS_LIBRARY}
${GDAL_LIBRARY}
${Boost_LIBRARIES}
${GLEW_LIBRARIES}
${FreeImage_LIBRARIES}
${Skia_LIBRARIES}
${PROTOBUF_LITE_LIBRARY}
${MAPPING_LIB}
${wxWidgets_LIBRARIES}
${TESS2_LIB}
"-framework OpenGL"
"-framework Foundation"
"-framework Cocoa"
"-framework CoreVideo"
${GLFW_LIB}
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

add_executable(mm
mm_studio/frame/app.cpp
mm_studio/frame/frame.h
mm_studio/frame/frame.cpp
mm_studio/frame/gui_job.h
mm_studio/frame/image_panel.h
mm_studio/frame/image_panel.cpp

mm_studio/algorithm/boost_geometry.h
mm_studio/algorithm/route.h
mm_studio/algorithm/route.cpp
mm_studio/algorithm/rtree.h
mm_studio/algorithm/rtree.cpp
mm_studio/algorithm/shapefile_graph.h
mm_studio/algorithm/shapefile_graph.cpp
mm_studio/algorithm/mm_model.h
mm_studio/algorithm/mm_model.cpp

mm_studio/algorithm/rl_utility.h
mm_studio/algorithm/rl_utility.cpp
mm_studio/algorithm/rl_algorithm.h
mm_studio/algorithm/rl_algorithm.cpp
mm_studio/algorithm/scale_model.h
mm_studio/algorithm/scale_model.cpp
)

target_link_libraries(mm ${COMMOM_LIBS})
