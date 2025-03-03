cmake_minimum_required(VERSION 3.0.2)
project(body_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  rospy
  std_msgs
  bodyctrl_msgs
  fast_ros
  message_generation
)

find_package (Eigen3 REQUIRED)
if(Eigen3)
    message(STATUS "EIGEN3_INCLUDE_DIR=${EIGEN3_INCLUDE_DIR}")
	message(STATUS "EIGEN3_LIBRARIES=${EIGEN3_LIBRARIES}")	
endif()

include_directories (${EIGEN3_INCLUDE_DIR})

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES body_control
  CATKIN_DEPENDS nodelet roscpp rospy fast_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
if(UNIX AND NOT APPLE)
  set(OS "linux")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ")
  set(OS_LIBS pthread rt)
endif()

include_directories(
# include
  src/common
  src/plugins/body_control/soem_master
  src/plugins/body_control/soem_master/soem/app
  src/plugins/body_control/soem_master/soem/soem
  src/plugins/body_control/soem_master/soem/osal
  src/plugins/body_control/soem_master/soem/osal/${OS}
  src/plugins/body_control/soem_master/soem/oshw/${OS}
  src/plugins/body_control/devices/rm_imu
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xscommon)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xscontroller)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xsensdeviceapi)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/include/xsense/xstypes)
include_directories(${PROJECT_SOURCE_DIR}/src/plugins/vn100_imu/vn100/include)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/body_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/lib)

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/MotionTestNode.cpp)

add_library(glog_init src/common/glog/GlogInitializer.cpp)

add_library(bodyctrl_plugin 
  src/plugins/body_control/BodyControlPlugin.cpp
  # device - motor
  src/plugins/body_control/devices/motor/MotorDevice.cpp
  src/plugins/body_control/devices/motor/MotorDeviceManager.cpp
  src/plugins/body_control/devices/motor/math_ops.cpp
  # device - imu_rm
  src/plugins/body_control/devices/rm_imu/RmImuDevice.cpp
  src/plugins/body_control/devices/rm_imu/ubt_hw/filters/complementary_filter.cpp
  src/plugins/body_control/devices/rm_imu/ubt_hw/filters/lp_filter.cpp
  src/plugins/body_control/devices/rm_imu/ubt_hw/drive/imu/rm_imu.cpp
  # device - imu_xsens_base
  src/plugins/body_control/devices/xsens_imu/base/XsensImuDevice.cpp
  src/plugins/body_control/devices/xsens_imu/base/mode1/XsensImuDevice.cpp
  src/plugins/body_control/devices/xsens_imu/base/mode2/XsensImuDevice.cpp
  # device - imu_xsens_hr
  src/plugins/body_control/devices/xsens_imu/high_rate/XsensImuHRDevice.cpp
  src/plugins/body_control/devices/xsens_imu/high_rate/XsensImuHR.cpp
  # device - sri
  src/plugins/body_control/devices/sri_sensor/SriSensorDevice.cpp
  src/plugins/body_control/devices/sri_sensor/SriDeviceManager.cpp
  # device - power_board
  src/plugins/body_control/devices/power_board/PowerBoardDevice.cpp
  src/plugins/body_control/devices/power_board/PowerMgr.cpp
  # device - hand_tsinghua
  src/plugins/body_control/devices/hand/tsinghua/TsinghuaHandDevice.cpp
  src/plugins/body_control/devices/hand/tsinghua/TsinghuaHandDeviceManager.cpp
  # device - ze_motor
  src/plugins/body_control/devices/zeroerr_motor/ZeroErrMotorDevice.cpp
  src/plugins/body_control/devices/zeroerr_motor/ZeroErrMotorDevMgr.cpp
  # device - eyou_motor
  src/plugins/body_control/devices/eyou_motor/EyouMotorDevice.cpp
  src/plugins/body_control/devices/eyou_motor/EyouMotorDevMgr.cpp
)
add_library(monitor_plugin src/plugins/monitor/MonitorPlugin.cpp)
add_library(test_plugin src/plugins/test/TestPlugin.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
LINK_DIRECTORIES(${CATKIN_PACKAGE_LIB_DESTINATION})

target_link_libraries(glog_init
  glog
)

#message(STATUS "catkin:        ")
set(INSTALL_DIR ${PROJECT_SOURCE_DIR}/../../install_isolated/lib)
#LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/../../install_isolated/lib)

target_link_libraries(bodyctrl_plugin
  soem_master
  soem
  glog_init
  ${INSTALL_DIR}/libfast_ros.so
  #fast_ros
  ${catkin_LIBRARIES}
)
target_link_libraries(test_plugin 
  ${catkin_LIBRARIES}
)

target_link_libraries(monitor_plugin
  glog_init 
  ${INSTALL_DIR}/libfast_ros.so
  #fast_ros
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )
install(TARGETS bodyctrl_plugin test_plugin monitor_plugin glog_init
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

# install(TARGETS xsensimu_plugin
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(DIRECTORY ${PROJECT_SOURCE_DIR}/src/plugins/xsens_imu/xsens/lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/../
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(
  DIRECTORY launch param
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  FILES bodyctrl_plugin.xml test_plugin.xml monitor_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_body_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_subdirectory(src/plugins/body_control/soem_master)
