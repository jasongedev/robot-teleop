cmake_minimum_required(VERSION 2.8.3)
project(handsfree_teleop)
find_package(catkin REQUIRED)
catkin_metapackage()

install(PROGRAMS custom_joint_mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

<<<<<<< HEAD
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

=======
>>>>>>> 6baebfdc6b109b2f904891b7b82fb0ac31b8a8db
