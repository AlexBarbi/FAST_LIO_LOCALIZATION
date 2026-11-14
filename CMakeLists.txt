cmake_minimum_required(VERSION 3.8)
project(fast_lio_localization)

# Python nodes only: the odometry comes from a FAST-LIO running elsewhere (/Odometry, /cloud_registered)
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/global_localization.py
  scripts/transform_fusion.py
  scripts/publish_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)
# helper module imported by the scripts above, must sit next to them
install(FILES scripts/se3_utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
