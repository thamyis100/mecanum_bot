cmake_minimum_required(VERSION 3.8)
project(mecanum_bot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch urdf config worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/cmd_vel_scaler.py scripts/slip_monitor.py scripts/odom_tf_broadcaster.py scripts/pose_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
