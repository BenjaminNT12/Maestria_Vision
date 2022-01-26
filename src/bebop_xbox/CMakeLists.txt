cmake_minimum_required(VERSION 2.8.3)
project(bebop_xbox)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


catkin_package()


include_directories(
# include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_bebop_xbox_node src/bebop_fly_controller.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_bebop_xbox_node PROPERTIES OUTPUT_NAME bebop_xbox PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_bebop_xbox_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_bebop_xbox_node
   ${OpenCV_INCLUDE_DIRS}
   ${catkin_LIBRARIES}
)


