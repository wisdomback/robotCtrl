cmake_minimum_required(VERSION 3.0.2)
project(track)
find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)