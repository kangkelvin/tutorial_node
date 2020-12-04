cmake_minimum_required(VERSION 2.8.3)
# TODO: change the name of the project, which will be your package name
project(<whatever you want>)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

# TODO: we are using std_msgs in our code, hence need to add the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
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

# TODO: we have our header file in a folder named "include", add that folder inside
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# TODO: name your executible and add the source file reference
add_executable(<whatever-you-want> src/<your-source-files>.cpp)
# TODO: Link your executible to libraries dependencies
target_link_libraries(<whatever-you-want>
        ${catkin_LIBRARIES}
        )
