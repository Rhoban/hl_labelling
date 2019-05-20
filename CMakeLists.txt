cmake_minimum_required(VERSION 2.8)
project(hl_labelling)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  hl_monitoring
  rhoban_utils
  )

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

catkin_package(
  CATKIN_DEPENDS hl_monitoring rhoban_utils
  )

include_directories(${catkin_INCLUDE_DIRS})

add_executable(rhoban_dynamic_camera_labelling_tool apps/rhoban_dynamic_camera_labelling_tool.cpp)
target_link_libraries(rhoban_dynamic_camera_labelling_tool ${catkin_LIBRARIES})

