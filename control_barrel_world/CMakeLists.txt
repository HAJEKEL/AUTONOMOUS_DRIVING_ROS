# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(control_barrel_world)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs pcl_ros pcl_conversions vision_msgs prius_msgs)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(${PROJECT_NAME}_node src/control_barrel_world.cpp src/main.cpp)

# Specify libraries against which to link.
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})