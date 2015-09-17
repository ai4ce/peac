cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ahc)

find_package(OpenCV REQUIRED)
find_package(PCL 1.2 REQUIRED)

include_directories("../include")
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(plane_fitter plane_fitter.cpp)
target_link_libraries(plane_fitter ${PCL_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS plane_fitter DESTINATION bin)

add_executable(plane_fitter_pcd plane_fitter_pcd.cpp)
target_link_libraries(plane_fitter_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS})
install(TARGETS plane_fitter_pcd DESTINATION bin)
install(FILES plane_fitter_pcd.ini DESTINATION bin)