cmake_minimum_required(VERSION 2.4.6)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(PkgConfig)

# opencv
find_package(OpenCV REQUIRED)
include_directories(${OPENCV_INCLUDE_DIRS})
link_directories(${OPENCV_LIBRARY_DIRS})

#pcl
find_package(PCL REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

#add_definitions(${PCL_DEFINITIONS})
#add_executable(pcd_write_test pcd_write.cpp)
#target_link_libraries(pcd_write_test ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})

add_library (rgbd_features
  src/features/feature_detector.cpp
  src/features/orb_detector.cpp
  src/features/surf_detector.cpp
  src/features/gft_detector.cpp
  src/features/star_detector.cpp)

target_link_libraries (rgbd_features
  ${OPENCV_LIBRARIES})

add_library (rgbd_registration
  src/registration/motion_estimation.cpp
  src/registration/motion_estimation_icp_prob_model.cpp)
  
target_link_libraries (rgbd_registration
  boost_signals 
  boost_system
  ${PCL_LIBRARIES})

add_library (rgbdtools
  src/rgbd_frame.cpp
  src/rgbd_keyframe.cpp
  src/rgbd_util.cpp)

target_link_libraries (rgbdtools
  boost_signals 
  boost_system
  boost_filesystem
  rgbd_features
  rgbd_registration
  ${OPENCV_LIBRARIES}
  ${PCL_LIBRARIES})
  
install(TARGETS rgbdtools DESTINATION $(LIBRARY_OUTPUT_PATH))
install(DIRECTORY include/rgbdtools DESTINATION include)
