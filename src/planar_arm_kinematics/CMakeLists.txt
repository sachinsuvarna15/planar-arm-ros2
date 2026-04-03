cmake_minimum_required(VERSION 3.8)
project(planar_arm_kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED) 

# GENERATE SERVICE
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ComputeIK.srv"
)

# EXPORT RUNTIME
ament_export_dependencies(rosidl_default_runtime)


install(PROGRAMS
  planar_arm_kinematics/fk_solver.py
  planar_arm_kinematics/ik_solver.py
  planar_arm_kinematics/test_ik_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
