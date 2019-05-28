cmake_minimum_required(VERSION 2.8)
project(hl_labelling)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOOUIC ON)

find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  qt_monitoring
  rhoban_utils
  )

find_package(Qt5 REQUIRED COMPONENTS MultimediaWidgets)

#Enable C++17
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS qt_monitoring rhoban_utils
  )

include_directories(${catkin_INCLUDE_DIRS} src)

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
qt5_use_modules(${PROJECT_NAME} MultimediaWidgets)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(rhoban_dynamic_camera_labelling_tool apps/rhoban_dynamic_camera_labelling_tool.cpp)
target_link_libraries(rhoban_dynamic_camera_labelling_tool ${catkin_LIBRARIES} ${PROJECT_NAME})

add_executable(labelling_tool apps/labelling_tool.cpp)
target_link_libraries(labelling_tool ${catkin_LIBRARIES} ${PROJECT_NAME})

