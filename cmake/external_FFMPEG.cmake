message(STATUS "ExternalProject FFMPEG") 

if(WIN32)   

#set( EXTERNAL_DIR "${CMAKE_BINARY_DIR}/external")
#set( EXTERNAL_INSTALL_DIR "${EXTERNAL_DIR}/install")
#set( EXTERNAL_BUILD_DIR "${EXTERNAL_DIR}/build")



	set(ffmpeg_name ffmpeg-3.3.2-win64)
				
	if (NOT EXISTS ${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-dev.zip)
		file(DOWNLOAD 
		https://ffmpeg.zeranoe.com/builds/win64/dev/${ffmpeg_name}-dev.zip 
		${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-dev.zip)
	endif()
	if (NOT EXISTS ${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-shared.zip)
		file(DOWNLOAD
		https://ffmpeg.zeranoe.com/builds/win64/shared/${ffmpeg_name}-shared.zip 
		${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-shared.zip)
	endif()
	
	execute_process(COMMAND ${CMAKE_COMMAND} -E tar xfz ${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-dev.zip
		WORKING_DIRECTORY ${EXTERNAL_BUILD_DIR}/ffmpeg
		RESULT_VARIABLE rv)

	if(NOT rv EQUAL 0)
	  message(STATUS "extracting... [error clean up]")
	  file(REMOVE_RECURSE "${EXTERNAL_BUILD_DIR}/ffmpeg")
	  message(FATAL_ERROR "error: extract of '${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-dev.zip' failed")
	endif()	
	
	execute_process(COMMAND ${CMAKE_COMMAND} -E tar xfz ${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-shared.zip
		WORKING_DIRECTORY ${EXTERNAL_BUILD_DIR}/ffmpeg
		RESULT_VARIABLE rvs)
	if(NOT rvs EQUAL 0)
	  message(STATUS "extracting... [error clean up]")
	  file(REMOVE_RECURSE "${EXTERNAL_BUILD_DIR}/ffmpeg")
	  message(FATAL_ERROR "error: extract of '${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-shared.zip' failed")
	endif()	
	
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-dev ${EXTERNAL_INSTALL_DIR} )
	execute_process(COMMAND ${CMAKE_COMMAND} -E copy_directory ${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-shared ${EXTERNAL_INSTALL_DIR} )
	
#	file(COPY ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg/${ffmpeg_name}-dev/* DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg)
#	file(COPY ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg/${ffmpeg_name}-shared/* DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/ffmpeg)	
		
	file(REMOVE_RECURSE "${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-dev")
	file(REMOVE_RECURSE "${EXTERNAL_BUILD_DIR}/ffmpeg/${ffmpeg_name}-shared")	
		
		
	set(FFMPEG_DIR ${EXTERNAL_INSTALL_DIR} )
	#STRING( REPLACE "\\" "/" FFMPEG_DIR  $ENV{FFMPEG_DIR})
	MESSAGE( STATUS " FFMPEG_DIR #  ${FFMPEG_DIR}")
else()
  message(WARNING "FFMPEG: using system version of FFMPEG, check version compatibility, tested on ffmpeg 3.3.1")
endif()	
	
# FIND and SET FFMPEG --------------------------------------------------
#STRING( REPLACE "\\" "/" FFMPEG_DIR  $ENV{FFMPEG_DIR})
#MESSAGE( STATUS " FFMPEG_DIR #  ${FFMPEG_DIR}")
set(FFMPEG_LIBRARIES )

find_library(FFMPEG_avcodec_LIBRARY NAMES avcodec-52 avcodec aviplayavcodec
  PATHS
  ${FFMPEG_DIR}/lib
  /usr/lib
  /usr/local/lib
  /usr/lib64/lib
  /usr/local/lib64
)
if (FFMPEG_avcodec_LIBRARY)
  set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_avcodec_LIBRARY})
endif(FFMPEG_avcodec_LIBRARY)

find_library(FFMPEG_avformat_LIBRARY NAMES avformat
  PATHS
  ${FFMPEG_DIR}
  ${FFMPEG_DIR}/lib
  ${FFMPEG_DIR}/libavformat
  /usr/lib
  /usr/local/lib
  /usr/lib64/lib
  /usr/local/lib64
)
if (FFMPEG_avformat_LIBRARY)
  set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_avformat_LIBRARY})
endif(FFMPEG_avformat_LIBRARY)

find_library(FFMPEG_avutil_LIBRARY NAMES avutil-49 avutil aviplayavutil
  PATHS
  ${FFMPEG_DIR}/lib
  /usr/lib
  /usr/local/lib
  /usr/lib64/lib
  /usr/local/lib64
)
if (FFMPEG_avutil_LIBRARY)
  set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_avutil_LIBRARY})
endif(FFMPEG_avutil_LIBRARY)

find_library(FFMPEG_swscale_LIBRARY NAMES swscale-0 swscale
  PATHS
  ${FFMPEG_DIR}/lib
  /usr/lib
  /usr/local/lib
  /usr/lib64/lib
  /usr/local/lib64
)
if (FFMPEG_swscale_LIBRARY)
  set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_swscale_LIBRARY})
endif(FFMPEG_swscale_LIBRARY)
  
find_library(FFMPEG_avdevice_LIBRARY NAMES avdevice
  PATHS
  ${FFMPEG_DIR}
  ${FFMPEG_DIR}/lib
  ${FFMPEG_DIR}/libavdevice
  /usr/lib
  /usr/local/lib
  /usr/lib64/lib
  /usr/local/lib64
)  
if (FFMPEG_avdevice_LIBRARY)
  set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_avdevice_LIBRARY})
endif(FFMPEG_avdevice_LIBRARY)
  
  
# find_library(_FFMPEG_z_LIBRARY_ NAMES z
  # PATHS
  # ${FFMPEG_DIR}
  # ${FFMPEG_DIR}/lib
  # /usr/lib
  # /usr/local/lib
  # /usr/lib64/lib
  # /usr/local/lib64
# )
# if (_FFMPEG_z_LIBRARY_)
  # set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${_FFMPEG_z_LIBRARY_})
# endif(_FFMPEG_z_LIBRARY_)
  
find_library(FFMPEG_swresample_LIBRARY NAMES swresample
  PATHS
  ${FFMPEG_DIR}
  ${FFMPEG_DIR}/lib
  /usr/lib
  /usr/local/lib
  /usr/lib64/lib
  /usr/local/lib64
)
if (FFMPEG_swresample_LIBRARY)
  set(FFMPEG_LIBRARIES ${FFMPEG_LIBRARIES} ${FFMPEG_swresample_LIBRARY})
endif(FFMPEG_swresample_LIBRARY) 
  
find_path(FFMPEG_AV_INCLUDE_DIR avcodec.h
  ${FFMPEG_DIR}/include/libavcodec
  /usr/include/libavcodec
  /usr/include/x86_64-linux-gnu/libavcodec
  /usr/local/include/libavcodec
  /usr/lib64/include/libavcodec
  /usr/local/include/libavcodec
)

get_filename_component(FFMPEG_INCLUDE_DIR "${FFMPEG_AV_INCLUDE_DIR}/.." ABSOLUTE)
#SET(FFMPEG_INCLUDE_DIR ${FFMPEG_AV_INCLUDE_DIR}/..)

if (FFMPEG_swscale_LIBRARY)
  set(FFMPEG_swscale_FOUND "YES" )
endif()

#include_directories(${FFMPEG_INCLUDE_DIR})



