cmake_minimum_required(VERSION 3.8)
project(nao_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# Task 1 startup loop: minimal world, launch and (reserved) config.
install(DIRECTORY launch worlds config
  DESTINATION share/${PROJECT_NAME}
)

# Task 3B: minimal joint-state converter (raw gz-bridged JointState -> frozen /joint_states).
add_executable(joint_state_converter src/joint_state_converter.cpp)
ament_target_dependencies(joint_state_converter rclcpp sensor_msgs)

install(TARGETS joint_state_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Task 3C: minimal body-pose converter (Gazebo Pose_V -> frozen /body_pose).
add_executable(body_pose_converter src/body_pose_converter.cpp)
ament_target_dependencies(body_pose_converter rclcpp geometry_msgs)
target_link_libraries(body_pose_converter
  gz-transport13::core
  gz-msgs10::core
)

install(TARGETS body_pose_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Task 3D: minimal IMU converter (raw bridged Imu -> frozen /imu/data).
add_executable(imu_converter src/imu_converter.cpp)
ament_target_dependencies(imu_converter rclcpp sensor_msgs)

install(TARGETS imu_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Task 3E: minimal foot-contact converter (raw bridged Contacts -> frozen
# /left_foot/contact and /right_foot/contact).
add_executable(contact_converter src/contact_converter.cpp)
ament_target_dependencies(contact_converter
  rclcpp
  std_msgs
  rosgraph_msgs
  ros_gz_interfaces
)

install(TARGETS contact_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Task 3F: minimal world -> base_link TF converter (Gazebo nao model pose ->
# one dynamic transform on /tf). robot_state_publisher remains the authority
# for the canonical URDF joint/fixed tree.
add_executable(world_tf_converter src/world_tf_converter.cpp)
ament_target_dependencies(world_tf_converter rclcpp geometry_msgs tf2_msgs)
target_link_libraries(world_tf_converter
  gz-transport13::core
  gz-msgs10::core
)

install(TARGETS world_tf_converter
  DESTINATION lib/${PROJECT_NAME}
)

# Task 4 Phase 4A: gz-sim system plugin that consumes the frozen /joint_command
# (sensor_msgs/msg/JointState) and applies a bounded position servo through
# gz::sim::components::JointVelocityCmd. It is loaded only into the
# enable_phase4a runtime model.
add_library(nao_phase4a_actuator SHARED src/phase4a_actuator.cpp)
ament_target_dependencies(nao_phase4a_actuator rclcpp sensor_msgs)
target_link_libraries(nao_phase4a_actuator
  gz-sim8::core
  gz-plugin2::register
)

install(TARGETS nao_phase4a_actuator
  DESTINATION lib/${PROJECT_NAME}
)

# Task 4 Phase 4A: explicit, finite probe and read-only trace recorder.
# Task 4 Phase 4B: minimal periodic gait producer and its read-only trace.
# Task 4 Phase 4D: frozen-schema trajectory collector and its shared schema
# module (installed side by side so ``import nao_trajectory_schema`` works).
install(PROGRAMS
  scripts/phase4a_probe.py
  scripts/phase4a_trace.py
  scripts/phase4b_producer.py
  scripts/phase4b_trace.py
  scripts/phase4d_collector.py
  scripts/nao_trajectory_schema.py
  scripts/nao_gait_phase.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
