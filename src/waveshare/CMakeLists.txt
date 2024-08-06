cmake_minimum_required(VERSION 2.8.3)
project(waveshare)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

find_package(serial REQUIRED)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${serial_INCLUDE_DIRS}
)

add_executable(roverstate src/roverstate.cpp)
target_link_libraries(roverstate ${catkin_LIBRARIES} ${serial_LIBRARIES})
add_executable(statepublisher src/statepublisher.cpp)
target_link_libraries(statepublisher ${catkin_LIBRARIES} )
