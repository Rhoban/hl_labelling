cmake_minimum_required(VERSION 3.16.3)
project(hl_labelling)

find_package(OpenCV REQUIRED)

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
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC hl_monitoring robot_model)

add_executable(labelling_tool apps/labelling_tool.cpp)
target_link_libraries(labelling_tool ${catkin_LIBRARIES} ${PROJECT_NAME} tclap)

