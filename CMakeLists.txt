cmake_minimum_required(VERSION 2.8.3)
project(racecar)

##project name important! what you run

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
## find package includes these packages in binaries

## Declare ROS messages and services
# add_message_files(FILES Num.msg)
# add_service_files(FILES AddTwoInts.srv)
## for custom messages ^^

## Generate added messages and services
# generate_messages(DEPENDENCIES std_msgs)
## turns .msg files into .mnh? files
## custom messages ^^

## Declare a catkin package
catkin_package()
## every catkin file needs one


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
##makes sure .h files included properly

add_executable(rbControl src/rbControl.cpp)
##adds an executable, call w. this given name
target_link_libraries(rbControl ${catkin_LIBRARIES})
# add_dependencies(talker beginner_tutorials_generate_messages_cpp)
## ^^ gives it these dependences

#add_executable(listener src/listener.cpp) RESTORE
#target_link_libraries(listener ${catkin_LIBRARIES}) RESTORE

# add_dependencies(listener beginner_tutorials_generate_messages_cpp)
