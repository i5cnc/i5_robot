cmake_minimum_required(VERSION 2.8.3)
project(i5_driver)

add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  industrial_robot_client
  i5_msgs
)

add_definitions(-DLINUXSOCKETS=1)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES driver
  CATKIN_DEPENDS industrial_robot_client i5_msgs
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  include/i5_driver
)



# non-swapping robot state node
add_executable(${PROJECT_NAME}_robot_state
  src/robot_state_node.cpp
  src/joint_feedback_handler.cpp
  )
target_link_libraries(${PROJECT_NAME}_robot_state
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_robot_state
  PROPERTIES OUTPUT_NAME robot_state
  PREFIX "")




# io_service node
add_executable(${PROJECT_NAME}_io_service
  src/io_service_node.cpp
  src/io_service.cpp
  src/i5_simple_message/i5_write_single_io.cpp
  src/i5_simple_message/messages/i5_write_single_io_message.cpp
  src/i5_simple_message/i5_read_single_io.cpp
  src/i5_simple_message/messages/i5_read_single_io_message.cpp
  )
target_link_libraries(${PROJECT_NAME}_io_service
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_io_service
  PROPERTIES OUTPUT_NAME io_service
  PREFIX "")
  
  
  # system_service node
add_executable(${PROJECT_NAME}_system_service
  src/system_service_node.cpp
  src/system_service.cpp
  src/i5_simple_message/i5_application_cmd.cpp
  src/i5_simple_message/messages/i5_application_cmd_message.cpp
  )
target_link_libraries(${PROJECT_NAME}_system_service
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
  set_target_properties(${PROJECT_NAME}_system_service
  PROPERTIES OUTPUT_NAME system_service
  PREFIX "")




# non-swapping joint streaming node
add_executable(${PROJECT_NAME}_motion_streaming_interface
  src/joint_streaming_node.cpp
  src/joint_trajectory_pt_full_streamer.cpp
  )
target_link_libraries(${PROJECT_NAME}_motion_streaming_interface
  industrial_robot_client
  simple_message
  ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_motion_streaming_interface
  PROPERTIES OUTPUT_NAME motion_streaming_interface
  PREFIX "")




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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_driver.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
