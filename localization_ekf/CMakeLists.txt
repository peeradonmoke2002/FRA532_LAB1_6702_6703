cmake_minimum_required(VERSION 3.5)
project(localization_ekf)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/dummy_script.py
  scripts/yawrate/odom_filtered_yawrate.py
  scripts/yawrate/gps.py
  scripts/yawrate/ackerman_yaw_rate_odom.py
  scripts/yawrate/pid_yawrate.py
  scripts/yawrate/stanlee_yawrate.py
  scripts/yawrate/pp_yawrate.py
  scripts/yawrate/ekf.py

  scripts/double_track/odom_filtered_doubletrack.py
  scripts/double_track/gps.py
  scripts/double_track/ackerman_odom_double_track.py
  scripts/double_track/pid_doubletrack.py
  scripts/double_track/stanlee_doubletrack.py
  scripts/double_track/pp_doubletrack.py
  

  scripts/single_track/odom_filtered_singletrack.py
  scripts/single_track/gps.py
  scripts/single_track/ackerman_odom_single_track.py
  scripts/single_track/pid_singletrack.py
  scripts/single_track/stanlee_singletrack.py
  scripts/single_track/pp_singletrack.py


  DESTINATION lib/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node_test src/cpp_node.cpp)
ament_target_dependencies(cpp_node_test rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node_test
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  config launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
