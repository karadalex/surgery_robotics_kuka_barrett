cmake_minimum_required(VERSION 2.8.3)
project(kuka_barrett_control)

find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
)


# message
add_message_files(FILES ControllerJointState.msg)
generate_messages(DEPENDENCIES std_msgs)

# include
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    dynamic_reconfigure
  LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME}
  src/ComputedTorqueControllerTest.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

