cmake_minimum_required(VERSION 3.0.2)
project(mobility_v1)

## Compile as C++11
add_compile_options(-std=c++11)

## Manually set the path for dynamixel_sdk (if installed from source)
set(dynamixel_sdk_DIR "/home/taka/mobility/src/DynamixelSDK/ros")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
  rospy
  std_msgs
  sensor_msgs  
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_controller
  CATKIN_DEPENDS dynamixel_sdk roscpp rospy std_msgs sensor_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamixel_sdk_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include  
  ${PROJECT_SOURCE_DIR}/include/mobility_v1 
)

## Declare the Dynamixel Controller as a library
add_library(dynamixel_controller src/dynamixel_controller.cpp)

## Ensure the library is linked properly
target_link_libraries(dynamixel_controller
  ${catkin_LIBRARIES}
  ${dynamixel_sdk_LIBRARIES} 
)

## Declare executables
add_executable(yaw_pitch_test src/yaw_pitch_test.cpp)
add_executable(mobility1 src/mobility1.cpp)
add_executable(mobilityROS src/mobilityROS.cpp) 

## Link executables to libraries
target_link_libraries(yaw_pitch_test
  ${catkin_LIBRARIES}
  dynamixel_controller  
)

target_link_libraries(mobility1
  ${catkin_LIBRARIES}
  dynamixel_controller  
)

target_link_libraries(mobilityROS
  ${catkin_LIBRARIES}
  dynamixel_controller 
)

## Add dependencies to ensure correct build order
add_dependencies(yaw_pitch_test ${catkin_EXPORTED_TARGETS})
add_dependencies(mobility1 ${catkin_EXPORTED_TARGETS})
add_dependencies(mobilityROS ${catkin_EXPORTED_TARGETS})  
add_dependencies(dynamixel_controller ${catkin_EXPORTED_TARGETS})
