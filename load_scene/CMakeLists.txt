cmake_minimum_required(VERSION 2.8.3)
project(load_scene)

# C++11 Support
add_compile_options(-std=c++11)


# Find Needed Packages
find_package(catkin REQUIRED COMPONENTS
  
  osrf_gear
  roscpp
  roslaunch
  
  sensor_msgs
  std_srvs
  trajectory_msgs
  geometry_msgs
  
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  geometric_shapes
  
  tf2
  tf2_ros
  
)


# Find Launch Files
roslaunch_add_file_check(launch)


# Why do I need this?
catkin_package(
   CATKIN_DEPENDS moveit_core moveit_ros_planning_interface
   #~ DEPENDS EIGEN3
)


## Build ##

include_directories(
 include
 ${catkin_INCLUDE_DIRS} 
 #~ ${Eigen_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


# Add executable file
add_executable(load_scene_node src/load_scene.cpp)
add_dependencies(load_scene_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(load_scene_node ${catkin_LIBRARIES})

# Link other class files
#~ target_sources(ariac_control PUBLIC src/Monitor.cpp src/Planner.cpp src/Executor.cpp   )











