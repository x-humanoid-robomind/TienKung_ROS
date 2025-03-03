cmake_minimum_required(VERSION 2.8.3)
project(usb_sbus)


find_package(catkin REQUIRED
    roscpp
    sensor_msgs
    std_msgs
    roslib
    rostime
    )

catkin_package(
# INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp std_msgs sensor_msgs
# DEPENDS system_lib
)

include_directories(include  ${Boost_INCLUDE_DIRS} 
  include
	${catkin_INCLUDE_DIRS} 
	${sdkapi_INCLUDE_DIRS}
)

add_definitions(-O3)

# generate_messages(
#  DEPENDENCIES
#  std_msgs  # Or other packages containing msgs
#)

add_executable(${PROJECT_NAME}_node
  src/CerealPort.cpp
  src/usbSbus.cpp
  src/sbus_main.cpp
)

add_dependencies(${PROJECT_NAME}_node
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
   ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
