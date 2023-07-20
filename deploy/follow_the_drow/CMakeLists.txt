cmake_minimum_required(VERSION 2.8...3.13)
project(follow_the_drow)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  visualization_msgs
  geometry_msgs
  genmsg
)

add_message_files(FILES
  raw_data.msg
)

catkin_python_setup()

generate_messages(DEPENDENCIES
  geometry_msgs
)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_generation
  message_runtime
  visualization_msgs
  geometry_msgs
  genmsg
)

find_package(FollowTheDrow CONFIG REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  $ENV{FollowTheDrow_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  $ENV{FollowTheDrow_LIBRARY_DIRS}
)

set(UTIL_SOURCES src/transformation.cpp)

add_executable(live_loader src/live_loader.cpp ${UTIL_SOURCES})
add_dependencies(live_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(live_loader ${catkin_LIBRARIES} FollowTheDrow)

catkin_install_python(PROGRAMS nodes/file_loader.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
