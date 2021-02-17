cmake_minimum_required(VERSION 2.8.3)
project(moveit_config_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_visual_tools
    moveit_core
    std_msgs
    geometry_msgs
    sensor_msgs
    
)

catkin_package(CATKIN_DEPENDS
    roscpp
    moveit_ros_planning_interface
    moveit_ros_planning
    moveit_visual_tools
    moveit_core
    std_msgs
    geometry_msgs
    sensor_msgs
)



include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(move_group_interface src/moveGroupInterface.cpp)
add_dependencies(move_group_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_group_interface ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_group_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
