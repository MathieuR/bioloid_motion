cmake_minimum_required(VERSION 2.8.3)
project(bioloid_motion)

# C++ 11
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  hardware_interface
  controller_manager
  roscpp
  control_msgs
  trajectory_msgs
  actionlib
  urdf
  joint_limits_interface
  transmission_interface
  control_toolbox
  std_msgs
  sensor_msgs
  rosparam_shortcuts
)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)
find_package(Eigen3 REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    hardware_interface
    controller_manager
    roscpp
    control_msgs
    trajectory_msgs
    urdf
    joint_limits_interface
    transmission_interface
    control_toolbox
    std_msgs
    sensor_msgs
    rosparam_shortcuts
  LIBRARIES
    bioloid_motion_robot
    bioloid_motion_utils
)

###########
## Build ##
###########

include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
)

# Robot Interface
add_library(bioloid_motion_robot src/bioloid_motion_robot.cpp)
target_link_libraries(bioloid_motion_robot
  ${catkin_LIBRARIES}
)

# Utils
add_library(bioloid_motion_utils
  src/bioloid_motion_utils.cpp
)
target_link_libraries(bioloid_motion_utils
  ${catkin_LIBRARIES}
)


# Main control executable
add_executable(${PROJECT_NAME}_walk src/bioloid_motion_walk.cpp)
set_target_properties(${PROJECT_NAME}_walk PROPERTIES OUTPUT_NAME robot_motion_walk PREFIX "")
target_link_libraries(${PROJECT_NAME}_walk
  bioloid_motion_robot
  bioloid_motion_utils
  ${catkin_LIBRARIES}
)

# Main control executable
add_executable(${PROJECT_NAME}_walk_test src/bioloid_motion_walk_test.cpp)
set_target_properties(${PROJECT_NAME}_walk_test PROPERTIES OUTPUT_NAME robot_motion_walk_test PREFIX "")
target_link_libraries(${PROJECT_NAME}_walk_test
  bioloid_motion_robot
  bioloid_motion_utils
  ${catkin_LIBRARIES}
)


## TOOLS ------------------------------------------------------


## Install ------------------------------------------------------------

# Install libraries
install(TARGETS
    bioloid_motion_robot
    bioloid_motion_utils
  LIBRARY DESTINATION
    ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# Install executables
install(TARGETS
    ${PROJECT_NAME}_walk
    ${PROJECT_NAME}_walk_test
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
