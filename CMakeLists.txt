cmake_minimum_required(VERSION 2.8.3)
project(grid_propagate)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  dynamic_reconfigure
  image_transport
  nodelet
  roscpp
  roslint
  sensor_msgs
  std_msgs
)

roslint_cpp()
find_package(OpenCV REQUIRED)

# generate_dynamic_reconfigure_options(
#   cfg/Convert.cfg
# )

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES grid_propagate
  CATKIN_DEPENDS nodelet roscpp sensor_msgs std_msgs
  DEPENDS OpenCV
)

include_directories(
#   include
  ${catkin_INCLUDE_DIRS}
)

add_executable(grid_propagate src/grid_propagate.cpp)
# add_dependencies(grid_propagate ${PROJECT_NAME}_gencfg)
target_link_libraries(grid_propagate
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS grid_propagate
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(FILES
#   nodelet_grid_propagate.xml
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
