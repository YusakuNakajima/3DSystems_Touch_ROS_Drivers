cmake_minimum_required(VERSION 3.10)
project(touch_common)

# ============================================================
# 1. Dependencies
# ============================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdf REQUIRED)

# Optional: touch_msgs (custom message package)
find_package(touch_msgs REQUIRED)

# Phantom Touch SDK include path (manual search if necessary)
find_path(OPENHAPTICS_INCLUDE_DIR
  NAMES HD/hd.h HL/hl.h
  PATHS /opt/OpenHaptics/Developer/3.4/include
)

if(NOT OPENHAPTICS_INCLUDE_DIR)
  message(FATAL_ERROR "OpenHaptics headers not found! Set OPENHAPTICS_INCLUDE_DIR manually.")
endif()

include_directories(
  ${OPENHAPTICS_INCLUDE_DIR}
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================
# 2. Sources and Executables
# ============================================================

# Main node (touch_state)
add_executable(touch_state src/touch_state.cpp)
ament_target_dependencies(touch_state
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
  orocos_kdl
  touch_msgs
  urdf
)
target_link_libraries(touch_state
  HD
  HDU
  pthread
)

# Optionally another node (touch)
# add_executable(touch src/touch.cpp)
# ament_target_dependencies(touch
#   rclcpp
#   geometry_msgs
#   sensor_msgs
#   std_msgs 
#   tf2
#   tf2_ros
#   tf2_geometry_msgs
#   kdl_parser
#   orocos_kdl
#   touch_msgs
# )
# target_link_libraries(touch
#   HD
#   HDU
#   pthread
# )


# ============================================================
# 3. Install
# ============================================================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS touch_state 
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# 4. Package
# ============================================================
ament_package()
