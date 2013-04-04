cmake_minimum_required(VERSION 2.8)

project(rgbdtools)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(PkgConfig)

# OpenCV
FIND_PACKAGE( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_libraries(${OpenCV_LIBRARIES})

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

# Boost
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})    

add_library (rgbdtools SHARED
  src/rgbd_frame.cpp
  src/rgbd_keyframe.cpp
  src/rgbd_util.cpp
  src/features/feature_detector.cpp
  src/features/orb_detector.cpp
  src/features/surf_detector.cpp
  src/features/gft_detector.cpp
  src/features/star_detector.cpp
  src/registration/motion_estimation.cpp
  src/registration/motion_estimation_icp_prob_model.cpp)

target_link_libraries (rgbdtools
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES})
  
install(TARGETS rgbdtools DESTINATION lib)
install(DIRECTORY include/rgbdtools DESTINATION include)
