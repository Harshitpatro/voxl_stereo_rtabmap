cmake_minimum_required(VERSION 3.5)
project(slam)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/merge_pc.py
  scripts/voxl_camera_info_publisher.py
  scripts/voxl_stereo_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
