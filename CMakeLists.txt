cmake_minimum_required(VERSION 2.8)
project(hl_labelling)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  hl_monitoring
  robot_model
  )

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS hl_monitoring robot_model
  )

include_directories(${catkin_INCLUDE_DIRS} include)

set(SOURCES_DIRECTORIES
  src/hl_labelling
  )

  #Include Sources sub sources
foreach (DIRECTORY ${SOURCES_DIRECTORIES})
  include (${DIRECTORY}/sources.cmake)
  set (PREFIXED_SOURCES)
  foreach (SOURCE ${SOURCES})
    set (PREFIXED_SOURCES ${PREFIXED_SOURCES} ${DIRECTORY}/${SOURCE})
  endforeach (SOURCE)
  set (ALL_SOURCES ${ALL_SOURCES} ${PREFIXED_SOURCES})
endforeach (DIRECTORY)

add_library (${PROJECT_NAME} SHARED ${ALL_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(labelling_tool apps/labelling_tool.cpp)
target_link_libraries(labelling_tool ${catkin_LIBRARIES} ${PROJECT_NAME})

