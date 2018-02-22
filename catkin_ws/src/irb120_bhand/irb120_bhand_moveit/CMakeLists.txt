cmake_minimum_required(VERSION 2.8.3)
project(irb120_bhand_moveit)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#add_executable(JointStatesBarrett src/JointStatesBarrett.cpp)
#target_link_libraries(JointStatesBarrett ${catkin_LIBRARIES})
#add_dependencies(JointStatesBarrett irb120_bhand_moveit_generate_messages_cpp)
