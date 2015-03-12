cmake_minimum_required(VERSION 2.8.3)
project(marker_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  moveit_ros_planning_interface
  tf
)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include/)

SET(SOURCES ${PROJECT_SOURCE_DIR}/src/marker_control.cpp
            ${PROJECT_SOURCE_DIR}/src/marker_control_lib.cpp
    )

SET(HEADERS ${PROJECT_SOURCE_DIR}/include/marker_control_lib.h
    )

catkin_package(
  DEPENDS
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp 
  LIBRARIES
  roscpp
  cmake_modules
)

add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${ROS_LIBRARIES} ${catkin_LIBRARIES}
)


