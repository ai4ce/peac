cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ahc)

find_package(OpenCV COMPONENTS core highgui imgproc imgcodecs QUIET)
if(NOT OpenCV_FOUND)
	find_package(OpenCV REQUIRED)
endif()
find_package(PCL COMPONENTS io REQUIRED)

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/../include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(REMOVE_ITEM PCL_LIBRARIES ${VTK_LIBRARIES} ${QHULL_LIBRARIES} ${FLANN_LIBRARIES}) #we don't need vtk, qhull, flann
message(STATUS "OpenCV_LIBS=${OpenCV_LIBS}")
message(STATUS "PCL_LIBRARIES=${PCL_LIBRARIES}")

###### Options
option(OPT_USE_BOOST_SHARED_PTR "use boost::shared_ptr instead of std::shared_ptr (in C++11) as ahc::shared_ptr" ON)
if(OPT_USE_BOOST_SHARED_PTR)
	add_definitions( -DUSE_BOOST_SHARED_PTR )
	message(STATUS "use boost::shared_ptr as ahc::shared_ptr")
else()
	message(STATUS "use std::shared_ptr (C++11) as ahc::shared_ptr")
endif()

option(OPT_USE_DSYEVH3 "use dsyevh3 (requires Internet to download package) instead of Eigen::SelfAdjointEigenSolver in LA::eig33sym" OFF)
if(OPT_USE_DSYEVH3)
	set(DSYEVH3_URL "http://www.mpi-hd.mpg.de/personalhomes/globes/3x3/download/3x3-C.tar.gz")
	set(DSYEVH3_DOWNLOAD_PATH "${CMAKE_BINARY_DIR}/3x3-C.tar.gz")
	
	file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/../3rdparty)
	set(DSYEVH3_ROOT "${CMAKE_SOURCE_DIR}/../3rdparty/3x3-C")
	
	if(NOT EXISTS "${DSYEVH3_ROOT}/dsyevh3.h")
		message(STATUS "downloading...\n  src=${DSYEVH3_URL}\n  dst=${DSYEVH3_DOWNLOAD_PATH}")
		file(DOWNLOAD ${DSYEVH3_URL} ${DSYEVH3_DOWNLOAD_PATH} STATUS status LOG log SHOW_PROGRESS EXPECTED_MD5 dc56d40543c41e6b975dbd91c0b1ddeb)
		list(GET status 0 status_code)
		list(GET status 1 status_string)
		if(NOT status_code EQUAL 0)
			message(WARNING "error: downloading ${DSYEVH3_URL} failed\n  status_code: ${status_code}\n  status_string: ${status_string}\n  log: ${log}")
			set(OPT_USE_DSYEVH3 OFF)
			set(HAS_DSYEVH3 FALSE)
		else()
			message(STATUS "dsyevh3 downloaded!")
			execute_process(
				COMMAND ${CMAKE_COMMAND} -E tar xzf ${DSYEVH3_DOWNLOAD_PATH}
				WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/../3rdparty
			)
			message(STATUS "dsyevh3 extracted to ${DSYEVH3_ROOT}!")
			set(HAS_DSYEVH3 TRUE)
			
			#now let's slightly modify two files in 3x3-C folder
			file(READ "${DSYEVH3_ROOT}/dsytrd3.h" DSYTRD3_H_TEXT)
			file(WRITE "${DSYEVH3_ROOT}/dsytrd3.h.orig" "${DSYTRD3_H_TEXT}")
			string(REPLACE "inline " "" DSYTRD3_H_TEXT "${DSYTRD3_H_TEXT}")
			file(WRITE "${DSYEVH3_ROOT}/dsytrd3.h" "${DSYTRD3_H_TEXT}")
			message(STATUS "removed 'inline ' in ${DSYEVH3_ROOT}/dsytrd3.h")
			
			file(READ "${DSYEVH3_ROOT}/dsytrd3.c" DSYTRD3_C_TEXT)
			file(WRITE "${DSYEVH3_ROOT}/dsytrd3.c.orig" "${DSYTRD3_C_TEXT}")
			string(REPLACE "inline " "" DSYTRD3_C_TEXT "${DSYTRD3_C_TEXT}")
			file(WRITE "${DSYEVH3_ROOT}/dsytrd3.c" "${DSYTRD3_C_TEXT}")
			message(STATUS "removed 'inline ' in ${DSYEVH3_ROOT}/dsytrd3.c")
		endif()
	else()
		set(HAS_DSYEVH3 TRUE)
	endif()
else()
	set(HAS_DSYEVH3 FALSE)
endif()
if(HAS_DSYEVH3)
	include_directories(${DSYEVH3_ROOT})
	set(DSYEVH3_SRC ${DSYEVH3_ROOT}/dsyevc3.c ${DSYEVH3_ROOT}/dsyevh3.c ${DSYEVH3_ROOT}/dsyevq3.c ${DSYEVH3_ROOT}/dsytrd3.c)
	set_source_files_properties(${DSYEVH3_SRC} PROPERTIES LANGUAGE CXX)
	add_definitions( -DUSE_DSYEVH3 )
	
	add_library(lib3x3 STATIC ${DSYEVH3_SRC})
	set(LIB3x3 lib3x3)
	message(STATUS "added project: ${LIB3x3}")
	
	message(STATUS "use dsyevh3 in LA::eig33sym")
else()
	set(LIB3x3 )
	message(STATUS "use Eigen::SelfAdjointEigenSolver in LA::eig33sym")
endif()

option(OPT_USE_OPENNI1 "use pcl::OpenNIGrabber instead of pcl::io::OpenNI2Grabber in plane_fitter.cpp" OFF)
if(OPT_USE_OPENNI1)
	add_definitions( -DUSE_OPENNI1 )
	message(STATUS "use pcl::OpenNIGrabber in plane_fitter.cpp")
else()
	message(STATUS "use pcl::io::OpenNI2Grabber in plane_fitter.cpp")
endif()

###### Projects
add_executable(plane_fitter plane_fitter.cpp)
target_link_libraries(plane_fitter ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LIB3x3})
install(TARGETS plane_fitter DESTINATION bin)
message(STATUS "added project: plane_fitter")

add_executable(plane_fitter_pcd plane_fitter_pcd.cpp plane_fitter_pcd.ini plane_fitter_pcd.tls.ini)
target_link_libraries(plane_fitter_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LIB3x3})
install(TARGETS plane_fitter_pcd DESTINATION bin)
install(FILES plane_fitter_pcd.ini plane_fitter_pcd.tls.ini DESTINATION bin)
message(STATUS "added project: plane_fitter_pcd")

add_executable(plane_fitter_pcd_debug plane_fitter_pcd_debug.cpp)
target_link_libraries(plane_fitter_pcd_debug ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LIB3x3})
install(TARGETS plane_fitter_pcd_debug DESTINATION bin)
message(STATUS "added project: plane_fitter_pcd_debug")

add_executable(pcd_recorder pcd_recorder.cpp)
target_link_libraries(pcd_recorder ${PCL_LIBRARIES} ${OpenCV_LIBS} ${LIB3x3})
install(TARGETS pcd_recorder DESTINATION bin)
message(STATUS "added project: pcd_recorder")