cmake_minimum_required(VERSION 2.8.3)
## TODO: name your ros package
project(############)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++17 -pthread")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ## TODO: since we are using std_msgs include it here
  message_generation
)

catkin_package(CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ## TODO: add the "include" folder into the include directories
)

## TODO: add the executible here and the source code reference
add_executable(############# src/############.cpp)
target_link_libraries(############## ${catkin_LIBRARIES})
