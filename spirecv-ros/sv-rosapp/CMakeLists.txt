cmake_minimum_required(VERSION 2.8.3)
project(spirecv_ros)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  actionlib_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_eigen
  spirecv_msgs
  image_transport
  cv_bridge
)


catkin_package(
  CATKIN_DEPENDS
  message_runtime
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  message_runtime
  std_msgs
)

find_package(SpireCV REQUIRED)
find_package(SpireCV-nonfree REQUIRED)
find_package(OpenCV 4 REQUIRED)

message(STATUS "SpireCV library status:")
message(STATUS "    include path: ${SV_INCLUDE_DIRS}")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SV_INCLUDE_DIRS}
  ${SVNF_INCLUDE_DIRS}
)

add_executable(mbzirc_suav_vessel_detection
  samples/mbzirc_suav_vessel_detection.cpp
)
target_link_libraries(mbzirc_suav_vessel_detection ${catkin_LIBRARIES} ${SV_LIBRARIES} ${SVNF_LIBRARIES})

add_executable(mbzirc_usv_vessel_detection
  samples/mbzirc_usv_vessel_detection.cpp
)
target_link_libraries(mbzirc_usv_vessel_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(mbzirc_tuav_object_detection
  samples/mbzirc_tuav_object_detection.cpp
)
target_link_libraries(mbzirc_tuav_object_detection ${catkin_LIBRARIES} ${SV_LIBRARIES} ${SVNF_LIBRARIES})

add_executable(mbzirc_arm_object_detection
  samples/mbzirc_arm_object_detection.cpp
)
target_link_libraries(mbzirc_arm_object_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(aruco_detection
  samples/aruco_detection.cpp
)
target_link_libraries(aruco_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(aruco_detection_with_cv_bridge
  samples/aruco_detection_with_cv_bridge.cpp
)
target_link_libraries(aruco_detection_with_cv_bridge ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(web_cam
  samples/web_cam.cpp
)
target_link_libraries(web_cam ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(rtsp2topic
  samples/rtsp2topic.cpp
)
target_link_libraries(rtsp2topic ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(common_object_detection
  samples/common_object_detection.cpp
)
target_link_libraries(common_object_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(saving_image_with_airborne_info
  samples/saving_image_with_airborne_info.cpp
)
target_link_libraries(saving_image_with_airborne_info ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(ellipse_detection
  samples/ellipse_detection.cpp
)
target_link_libraries(ellipse_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(landing_marker_detection
  samples/landing_marker_detection.cpp
)
target_link_libraries(landing_marker_detection ${catkin_LIBRARIES} ${SV_LIBRARIES})

add_executable(single_object_tracking
  samples/single_object_tracking.cpp
)
target_link_libraries(single_object_tracking ${catkin_LIBRARIES} ${SV_LIBRARIES})


