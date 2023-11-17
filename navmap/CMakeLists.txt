cmake_minimum_required(VERSION 3.0.2)
project(navmap)
find_package(catkin REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES navmap
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_executable(${PROJECT_NAME}_node src/navmap_node.cpp)