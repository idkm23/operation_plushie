cmake_minimum_required(VERSION 2.8.3)
project(operation_plushie)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  baxter_core_msgs
  pcl_conversions
  pcl_ros
  message_generation
)

find_package( OpenCV REQUIRED )
 
add_service_files(
   FILES
   Deliver.srv
   Pickup.srv
   RepositionHand.srv
   isComplete.srv
   Ping.srv
   BowlValues.srv
   PositionJoints.srv
 )

 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
 )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES operation_plushie_node
  CATKIN_DEPENDS message_runtime roscpp sensor_msgs cv_bridge baxter_core_msgs
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
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a cpp executable
 add_executable(operation_plushie_node src/operation_plushie_node.cpp)
 add_executable(delivery_node src/delivery_node.cpp)
 add_executable(pickup_node src/pickup_node.cpp)
 add_executable(reposition_hand_node src/reposition_hand_node.cpp)
 add_executable(find_bowl_node src/find_bowl_node.cpp)
 add_executable(position_joints_node src/position_joints_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(operation_plushie_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(operation_plushie_node FaceDetector.cpp)
 add_dependencies(operation_plushie_node FaceDetector.h)
 add_dependencies(operation_plushie_node operation_plushie_generate_messages_cpp)

 add_dependencies(delivery_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(delivery_node Delivery.cpp)
 add_dependencies(delivery_node Delivery.h)
 add_dependencies(delivery_node operation_plushie_generate_messages_cpp)

 add_dependencies(pickup_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(pickup_node Pickup.cpp)
 add_dependencies(pickup_node operation_plushie_generate_messages_cpp)

 add_dependencies(reposition_hand_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(reposition_hand_node RepositionHand.cpp)
 add_dependencies(reposition_hand_node operation_plushie_generate_messages_cpp)
 
 add_dependencies(find_bowl_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(find_bowl_node FindBowl.cpp)
 add_dependencies(find_bowl_node operation_plushie_generate_messages_cpp)

 add_dependencies(position_joints_node ${catkin_EXPORTED_TARGETS})
 add_dependencies(position_joints_node PositionJoints.cpp)
 add_dependencies(position_joints_node operation_plushie_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(operation_plushie_node
    ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
 )

 target_link_libraries(delivery_node
    ${catkin_LIBRARIES}
 )

 target_link_libraries(pickup_node
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
 )

 target_link_libraries(reposition_hand_node
    ${catkin_LIBRARIES}
 )

target_link_libraries(find_bowl_node
    ${catkin_LIBRARIES}
 )

target_link_libraries(position_joints_node
    ${catkin_LIBRARIES}
 )
