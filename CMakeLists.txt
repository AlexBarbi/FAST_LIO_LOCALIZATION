cmake_minimum_required(VERSION 3.8)
project(fast_lio_localization)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions")

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)")
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# laserMapping.cpp includes Python.h for matplotlibcpp
find_package(Python3 REQUIRED COMPONENTS Development)

# ROS dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf2_ros
  pcl_ros
  pcl_conversions
  livox_ros_driver2
)

# Thirdparty libraries
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pose6D.msg"
)
ament_export_dependencies(rosidl_default_runtime)

add_executable(fastlio_mapping src/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/preprocess.cpp)
target_include_directories(fastlio_mapping PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(fastlio_mapping ${PCL_LIBRARIES} Python3::Python Eigen3::Eigen)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(fastlio_mapping ${cpp_typesupport_target})
ament_target_dependencies(fastlio_mapping ${dependencies})

install(TARGETS fastlio_mapping
  DESTINATION lib/${PROJECT_NAME}
)

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
  DIRECTORY config launch rviz_cfg
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
