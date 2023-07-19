cmake_minimum_required(VERSION 2.8)
project(follow_the_drow)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
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

generate_messages(DEPENDENCIES
  geometry_msgs
)

catkin_package(INCLUDE_DIRS CATKIN_DEPENDS
  roscpp
  std_msgs
  sensor_msgs
  message_generation
  visualization_msgs
  geometry_msgs
  genmsg
)

find_package(FollowTheDrow CONFIG REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${FollowTheDrow_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${FollowTheDrow_LIBRARY_DIRS}
)

set(UTIL_SOURCES src/transformation.cpp)

add_executable(raw_data_aggregator src/raw_data_aggregator.cpp ${UTIL_SOURCES})
add_dependencies(raw_data_aggregator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(raw_data_aggregator ${catkin_LIBRARIES} ${FollowTheDrow_LIBRARY_DIRS})
