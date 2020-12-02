cmake_minimum_required(VERSION 2.8.3)
project(tutorial_node)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  exampleMessage.msg
)

add_service_files(
  FILES
  exampleService.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(tutorial_node src/tutorial_node.cpp)
target_link_libraries(tutorial_node
        ${catkin_LIBRARIES}
        )
