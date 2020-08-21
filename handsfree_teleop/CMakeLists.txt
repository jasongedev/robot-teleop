cmake_minimum_required(VERSION 2.8.3)
project(handsfree_teleop)
find_package(catkin REQUIRED)
catkin_metapackage()

install(PROGRAMS custom_joint_mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

