cmake_minimum_required(VERSION 3.5.1) # version on Ubuntu Trusty
project(behaviortree_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

######################################################

set(ROS_DEPENDENCIES
    roscpp std_msgs
    behaviortree_cpp
    actionlib_msgs
    actionlib
    message_generation
    yolo_object_detection
    common_msg_srv
    common_action_service_servers
    )

find_package(catkin REQUIRED COMPONENTS ${ROS_DEPENDENCIES} )
find_package(GTest)

add_message_files(
  FILES
  BehaviorTree.msg
  NodeParameter.msg
  NodeStatus.msg
  StatusChange.msg
  StatusChangeLog.msg
  TreeNode.msg
)

add_service_files(
   FILES
   AddTwoInts.srv
 )

add_action_files(
   DIRECTORY action
   FILES     Fibonacci.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES
    CATKIN_DEPENDS ${ROS_DEPENDENCIES}
)

include_directories( include ${catkin_INCLUDE_DIRS})

######################################################
# LIBRARIES

#header only for the time being

#add_library(behaviortree_ros ... )
#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

######################################################
# EXAMPLES

add_executable(generate_tree_node_model scripts/generate_tree_node_model.cpp)
add_dependencies(generate_tree_node_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(generate_tree_node_model ${catkin_LIBRARIES} )

add_executable(bt_task2 src/bt_task2.cpp)
add_dependencies(bt_task2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bt_task2 ${catkin_LIBRARIES} )

add_executable(obj_depo src/obj_depo.cpp)
add_dependencies(obj_depo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obj_depo ${catkin_LIBRARIES} )

add_executable(navigation src/navigation.cpp)
add_dependencies(navigation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(navigation ${catkin_LIBRARIES} )

add_executable(test_action src/test_action.cpp)
add_dependencies(test_action ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_action ${catkin_LIBRARIES} )
######################################################
# INSTALL

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

######################################################
# EXAMPLES and TOOLS



