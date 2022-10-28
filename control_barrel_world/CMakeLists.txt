# What version of CMake is needed?
cmake_minimum_required(VERSION 2.8.3)

# The name of this package.
project(control_barrel_world)

# Find the catkin build system, and any other packages on which we depend.
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs turtlesim std_srvs)

# Declare our catkin package.
catkin_package()

# Specify locations of header files.
include_directories(include ${catkin_INCLUDE_DIRS})

# Declare the executable, along with its source files.
add_executable(hello hello.cpp)
add_executable(pubvel pubvel.cpp)
add_executable(subpose subpose.cpp)
add_executable(count count.cpp)
# Specify libraries against which to link.
target_link_libraries(hello ${catkin_LIBRARIES})
target_link_libraries(pubvel ${catkin_LIBRARIES})
target_link_libraries(subpose ${catkin_LIBRARIES})
target_link_libraries(count ${catkin_LIBRARIES})