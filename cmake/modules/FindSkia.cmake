# SKIA_ROOT should be set

# Find header directories
file(GLOB SKIA_INCLUDE_DIRS
    ${SKIA_ROOT}/include
    ${SKIA_ROOT}/include/*
    ${SKIA_ROOT}/src
    ${SKIA_ROOT}/src/*
    )

if (UNIX)
  if (NOT APPLE)
    set(SKIA_INCLUDE_DIRS
      ${SKIA_INCLUDE_DIRS}
      ${SKIA_ROOT}/src/pipe/utils
      ${SKIA_ROOT}/src/utils/debugger
      ${SKIA_ROOT}/third_party/etc1
      ${SKIA_ROOT}/tools 
      ${SKIA_ROOT}/tools/flags
      ${SKIA_ROOT}/third_party/externals/jsoncpp/include
      ${SKIA_ROOT}/experimental/PdfViewer
      ${SKIA_ROOT}/experimental/PdfViewer/src 
      ${SKIA_ROOT}/experimental
      /usr/include/poppler/cpp 
      )
  endif()
endif()

# Find libraries
if (APPLE)
	message(STATUS "Apple platform!")
  	file(GLOB SKIA_LIBRARIES
    	${SKIA_ROOT}/out/Release/*.a
    	${SKIA_ROOT}/out/Release/obj/gyp/*.a
    	)
elseif(UNIX)
	message(STATUS "unix platform!")
	set(SKIA_LIBRARIES ${SKIA_ROOT}/out/Release/lib/libskia.so)
elseif(WIN32)
   file(GLOB SKIA_LIBRARIES
    ${SKIA_ROOT}/out/Release/*.lib
    ${SKIA_ROOT}/out/Release/obj/gyp/*.lib
    )
endif()

if(WIN32)
  set(SKIA_LIBRARIES
    ${SKIA_LIBRARIES}
    ws2_32.lib
    OpenGL32.lib
	usp10.lib
	kernel32.lib
	gdi32.lib
	winspool.lib
	comdlg32.lib
	advapi32.lib
	shell32.lib
	ole32.lib
	oleaut32.lib
	user32.lib
	uuid.lib
	odbc32.lib
	odbccp32.lib
	DelayImp.lib
	windowscodecs.lib
  )
elseif(APPLE)
  set(SKIA_LIBRARIES
    ${SKIA_LIBRARIES}
    "-framework Foundation"
    "-framework Cocoa"
    "-framework OpenGL"
    )
elseif(UNIX)
  set(SKIA_LIBRARIES
    ${SKIA_LIBRARIES}
    rt
    poppler-cpp
    png
    z
    gif
    pthread
    fontconfig
    dl
    freetype
    )
 endif()