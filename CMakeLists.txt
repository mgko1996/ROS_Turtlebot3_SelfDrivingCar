cmake_minimum_required(VERSION 2.8.3)
project(final)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

generate_messages(DEPENDENCIES sensor_msgs std_msgs)

catkin_package(
  LIBRARIES final
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(final_1 src/final_1.cpp) 
add_dependencies(final_1 ${${PROJECT_NAME}_EXPORTED_TARGETS}${catkin_EXPOTED_TARGETS})
target_link_libraries(final_1 ${catkin_LIBRARIES})

