cmake_minimum_required(VERSION 2.8.3)

project(application_control)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  message_generation  
  roscpp 
  tf
  baxter_core_msgs
  control_msgs
  actionlib_msgs
  genmsg
  actionlib
  image_transport
  cv_bridge
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories($ENV{HALCONROOT}/include)
include_directories($ENV{HALCONROOT}/include/halconcpp)
include_directories($ENV{HALCONROOT}/include/hdevengine)
include_directories(${catkin_INCLUDE_DIRS})
include_directories(include)


## Generate messages in the 'msg' folder
add_message_files(
    FILES
    State.msg
    changeState.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   PauseRequest.srv
   request_camera_parameters.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
)


catkin_package(
    DEPENDS yaml-cpp
    CATKIN_DEPENDS roscpp std_msgs control_msgs message_runtime
)

add_executable(application_control_node
src/Main.cpp 
src/ColdBootState.cpp
src/ReadyState.cpp
src/MoveToHomeState.cpp
src/ScanPlantState.cpp
src/VisualServoState.cpp
)


link_directories(/opt/halcon/lib/x64-linux)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

target_link_libraries(application_control_node yaml-cpp  ${catkin_LIBRARIES})
target_link_libraries(application_control_node $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhdevenginecpp.so ${catkin_LIBRARIES})
target_link_libraries(application_control_node $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalconcpp.so ${catkin_LIBRARIES})
target_link_libraries(application_control_node $ENV{HALCONROOT}/lib/$ENV{HALCONARCH}/libhalcon.so ${catkin_LIBRARIES})
target_link_libraries(application_control_node yaml-cpp  ${catkin_LIBRARIES})

add_dependencies(application_control_node ${catkin_EXPORTED_TARGETS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(application_control_node ${OpenCV_LIBRARIES})


