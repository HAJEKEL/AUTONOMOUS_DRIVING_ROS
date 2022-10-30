# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(pcl_obstacle_detector)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_ros pcl_conversions vision_msgs)


# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})


# Declare the executable, along with its source files.
add_executable(${PROJECT_NAME}_node src/main.cpp src/pcl_obstacle_detector.cpp)

# Specify libraries against which to link.
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

# Add cmake target dependencies of the executable same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})