project (MechEye_OpenInterface)
cmake_minimum_required(VERSION 3.7)

set(DEFAULT_INCLUDE_PATH "D:/dependencies for MechEye_OpenInterface/include")
set(DEFAULT_LIBRARY_PATH "D:/dependencies for MechEye_OpenInterface/lib")

set(INCLUDE_PATH_OPENCV ${DEFAULT_INCLUDE_PATH} CACHE PATH "Where to look for OpenCV include files")
set(INCLUDE_PATH_ZMQ ${DEFAULT_INCLUDE_PATH} CACHE PATH "Where to look for ZeroMQ include files")
set(INCLUDE_PATH_PROTOBUF ${DEFAULT_INCLUDE_PATH} CACHE PATH "Where to look for Protobuf include files")
set(INCLUDE_PATH_EIGEN ${DEFAULT_INCLUDE_PATH}/eigen3 CACHE PATH "Where to look for Eigen include files")
set(INCLUDE_PATH_PCL ${DEFAULT_INCLUDE_PATH} CACHE PATH "Where to look for PCL include files")
set(INCLUDE_PATH_BOOST ${DEFAULT_INCLUDE_PATH} CACHE PATH "Where to look for Boost include files")

set(LIBRARY_PATH ${DEFAULT_LIBRARY_PATH}/release CACHE PATH "Where to look for library files")
set(LIBRARY_PATH_DEBUG ${DEFAULT_LIBRARY_PATH}/debug CACHE PATH "Where to look for debug library files")


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/proto ${CMAKE_CURRENT_SOURCE_DIR}/include ${INCLUDE_PATH_OPENCV}
${INCLUDE_PATH_ZMQ} ${INCLUDE_PATH_PROTOBUF} ${INCLUDE_PATH_EIGEN} ${INCLUDE_PATH_PCL} ${INCLUDE_PATH_BOOST})

aux_source_directory(src SRC_LIST)
aux_source_directory(sample SRC_LIST)
aux_source_directory(proto SRC_PROTO_LIST)
add_executable(MechEye_OpenInterface ${SRC_LIST} ${SRC_PROTO_LIST})
	
target_link_libraries(MechEye_OpenInterface
debug ${LIBRARY_PATH_DEBUG}/opencv_core320d.lib 	 optimized ${LIBRARY_PATH}/opencv_core320.lib
debug ${LIBRARY_PATH_DEBUG}/opencv_imgcodecs320d.lib optimized ${LIBRARY_PATH}/opencv_imgcodecs320.lib
debug ${LIBRARY_PATH_DEBUG}/libprotobufd.lib 		 optimized ${LIBRARY_PATH}/libprotobuf.lib 
debug ${LIBRARY_PATH_DEBUG}/libzmq.lib 				 optimized ${LIBRARY_PATH}/libzmq.lib
debug ${LIBRARY_PATH_DEBUG}/pcl_io_debug.lib 		 optimized ${LIBRARY_PATH}/pcl_io_release.lib
debug ${LIBRARY_PATH_DEBUG}/libboost_chrono-vc140-mt-gd-1_62.lib 	 optimized ${LIBRARY_PATH}/libboost_chrono-vc140-mt-1_62.lib
debug ${LIBRARY_PATH_DEBUG}/libboost_date_time-vc140-mt-gd-1_62.lib  optimized ${LIBRARY_PATH}/libboost_date_time-vc140-mt-1_62.lib
debug ${LIBRARY_PATH_DEBUG}/libboost_filesystem-vc140-mt-gd-1_62.lib optimized ${LIBRARY_PATH}/libboost_filesystem-vc140-mt-1_62.lib
debug ${LIBRARY_PATH_DEBUG}/libboost_iostreams-vc140-mt-gd-1_62.lib  optimized ${LIBRARY_PATH}/libboost_iostreams-vc140-mt-1_62.lib
debug ${LIBRARY_PATH_DEBUG}/libboost_system-vc140-mt-gd-1_62.lib 	 optimized ${LIBRARY_PATH}/libboost_system-vc140-mt-1_62.lib
debug ${LIBRARY_PATH_DEBUG}/libboost_thread-vc140-mt-gd-1_62.lib 	 optimized ${LIBRARY_PATH}/libboost_thread-vc140-mt-1_62.lib)
 



