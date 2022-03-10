cmake_minimum_required(VERSION 3.0.2)
project(firefly_mapping)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  eigen_conversions
)
add_message_files(
        FILES
        ImageWithPose.msg
)
generate_messages(
        DEPENDENCIES
        sensor_msgs
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(firefly_mapping src/firefly_mapping.cpp)
add_dependencies(firefly_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(firefly_mapping ${catkin_LIBRARIES})

