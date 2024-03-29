cmake_minimum_required(VERSION 2.8.3)
project(kuka_barrett)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_visual_tools
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    kinematics
    custom_math
    trajectory
    control_msgs
    genmsg
    geometry_msgs
)

## System dependencies are found with CMake's conventions
 find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES 
  GoToHomePosition.action 
  GoToToolHomePosition.action 
  ScanToolTableVisualServo.action 
  PickSurgicalTool.action
  PlaceSurgicalTool.action
  ExecutePivotMotion.action
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kuka_barrett
  CATKIN_DEPENDS roscpp kinematics trajectory actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${catkin_INCLUDE_DIRS}
     ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/kuka_barrett.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/kuka_barrett_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kuka_barrett.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(robot_planner_custom src/robot_planner_custom.cpp)
target_link_libraries(robot_planner_custom ${catkin_LIBRARIES})

add_executable(robot_planner1 src/robot_planner1.cpp)
target_link_libraries(robot_planner1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner2 src/robot_planner2.cpp)
target_link_libraries(robot_planner2 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner2b src/robot_planner2b.cpp)
target_link_libraries(robot_planner2b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3a src/robot_planner3a.cpp)
target_link_libraries(robot_planner3a ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3b src/robot_planner3b.cpp)
target_link_libraries(robot_planner3b ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3c src/robot_planner3c.cpp)
target_link_libraries(robot_planner3c ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3d src/robot_planner3d.cpp)
target_link_libraries(robot_planner3d ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3e src/robot_planner3e.cpp)
target_link_libraries(robot_planner3e ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3f src/robot_planner3f.cpp)
target_link_libraries(robot_planner3f ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3g src/robot_planner3g.cpp)
target_link_libraries(robot_planner3g ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner3h src/robot_planner3h.cpp)
target_link_libraries(robot_planner3h ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner4 src/robot_planner4.cpp)
target_link_libraries(robot_planner4 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner5 src/robot_planner5.cpp)
target_link_libraries(robot_planner5 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner6 src/robot_planner6.cpp)
target_link_libraries(robot_planner6 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(robot_planner8 src/robot_planner8.cpp)
target_link_libraries(robot_planner8 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scene_objects src/scene_objects.cpp)
target_link_libraries(scene_objects ${catkin_LIBRARIES})

add_executable(scene_objects2 src/scene_objects2.cpp)
target_link_libraries(scene_objects2 ${catkin_LIBRARIES})

add_executable(simple_circular_traj src/simple_circular_traj.cpp)
target_link_libraries(simple_circular_traj ${catkin_LIBRARIES})


# ACTION SERVERS

add_executable(GoToHomePositionAS src/actions/GoToHomePositionAS.cpp)
target_link_libraries(GoToHomePositionAS ${catkin_LIBRARIES})

add_executable(GoToToolHomePositionAS src/actions/GoToToolHomePositionAS.cpp)
target_link_libraries(GoToToolHomePositionAS ${catkin_LIBRARIES})

add_executable(ScanToolTableVisualServoAS src/actions/ScanToolTableVisualServoAS.cpp)
target_link_libraries(ScanToolTableVisualServoAS ${catkin_LIBRARIES})

add_executable(PickSurgicalToolAS src/actions/PickSurgicalToolAS.cpp)
target_link_libraries(PickSurgicalToolAS ${catkin_LIBRARIES})

add_executable(PlaceSurgicalToolAS src/actions/PlaceSurgicalToolAS.cpp)
target_link_libraries(PlaceSurgicalToolAS ${catkin_LIBRARIES})

add_executable(ExecutePivotMotionAS src/actions/ExecutePivotMotionAS.cpp)
target_link_libraries(ExecutePivotMotionAS ${catkin_LIBRARIES})