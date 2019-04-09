# adapted from aruco_detect package

cmake_minimum_required(VERSION 2.8.3)
project(detect_aruco)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_geometry_msgs
  tf2_ros
  tf2
  visualization_msgs
  image_transport
  cv_bridge
  sensor_msgs
  std_msgs
  fiducial_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(INCLUDE_DIRS DEPENDS OpenCV)

###########
## Build ##
###########

add_definitions(-std=c++11)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(aruco_detect src/aruco_detect.cpp)

add_dependencies(aruco_detect ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})

target_link_libraries(aruco_detect ${catkin_LIBRARIES} ${OpenCV_LIBS})

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS aruco_detect
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)  
