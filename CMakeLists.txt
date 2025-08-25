cmake_minimum_required(VERSION 3.8)
project(hardware_ctl)

# Add compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS and other dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(person_detection_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# ----------------------------
# LibSerial via pkg-config
# ----------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

# Add include dirs
include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${LIBSERIAL_INCLUDE_DIRS}
)

# Add your executable
add_executable(hardware_ctl_node src/hardware_ctl_node.cpp)

# ament dependencies
ament_target_dependencies(hardware_ctl_node
  rclcpp
  person_detection_msgs
  geometry_msgs
  cv_bridge
  OpenCV
)

# Link libraries
target_link_libraries(hardware_ctl_node
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${LIBSERIAL_LDFLAGS}
)

# Install target
install(TARGETS
  hardware_ctl_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Optional linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
