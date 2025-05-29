cmake_minimum_required(VERSION 3.10)
project(omni_common)

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

# Optional: omni_msgs (custom message package)
find_package(omni_msgs REQUIRED)

# Phantom Omni SDK include path (manual search if necessary)
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

# Main node (omni_state)
add_executable(omni_state src/omni_state.cpp)
ament_target_dependencies(omni_state
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
  orocos_kdl
  omni_msgs
  urdf
)
target_link_libraries(omni_state
  HD
  HDU
  pthread
)

# Optionally another node (omni)
add_executable(omni src/omni.cpp)
ament_target_dependencies(omni
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs 
  tf2
  tf2_ros
  tf2_geometry_msgs
  kdl_parser
  orocos_kdl
  omni_msgs
)
target_link_libraries(omni
  HD
  HDU
  pthread
)


# ============================================================
# 3. Install
# ============================================================
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(TARGETS omni_state omni
  DESTINATION lib/${PROJECT_NAME}
)

# ============================================================
# 4. Package
# ============================================================
ament_package()
