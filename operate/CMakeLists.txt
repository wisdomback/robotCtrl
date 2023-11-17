cmake_minimum_required(VERSION 2.8.3)
project(operate)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(operate src/operate.cpp)
target_link_libraries(operate ${catkin_LIBRARIES})
add_executable(keyboard src/keyboard.cpp)
target_link_libraries(keyboard ${catkin_LIBRARIES})

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
