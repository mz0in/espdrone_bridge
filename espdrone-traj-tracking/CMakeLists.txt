cmake_minimum_required(VERSION 3.0.2)
project(espdrone-traj-tracking)

set(CMAKE_BUILD_TYPE "Release")

# set(CMAKE_BUILD_TYPE "RelWithDebInfo")
# set(CMAKE_BUILD_TYPE "Debug")
find_package(catkin REQUIRED COMPONENTS
  mav_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  tf
  tf_conversions
)
find_package(gazebo REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES aerial_catcher
#  CATKIN_DEPENDS mav_msgs nav_msgs roscpp rospy std_msgs
#  DEPENDS system_lib
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ################## source and driver ######################

# add_executable(ac_lee_position_controller_esp_node src/ac_lee_position_controller_esp_node.cpp)
# target_link_libraries(ac_lee_position_controller_esp_node ${catkin_LIBRARIES})

# add_executable(ac_lee_position_controller_esp src/ac_lee_position_controller_esp.cpp)
# target_link_libraries(ac_lee_position_controller_esp ${catkin_LIBRARIES})

add_library(ac_lee_position_controller_esp src/ac_lee_position_controller_esp.cpp)
add_dependencies(ac_lee_position_controller_esp ${catkin_EXPORTED_TARGETS})
target_link_libraries(ac_lee_position_controller_esp ${catkin_LIBRARIES})

add_executable(ac_lee_position_controller_esp_node src/ac_lee_position_controller_esp_node.cpp)
add_dependencies(ac_lee_position_controller_esp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ac_lee_position_controller_esp_node ${catkin_LIBRARIES} ac_lee_position_controller_esp )

add_executable(traj_ctl src/traj_ctl.cpp)
target_link_libraries(traj_ctl ${catkin_LIBRARIES})