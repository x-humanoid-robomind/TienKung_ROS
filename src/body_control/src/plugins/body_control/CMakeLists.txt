cmake_minimum_required(VERSION 3.5)
project(MasterStack)

add_subdirectory(soem_master/soem)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(
        soem_master/soem/soem
        soem_master/soem/osal
        soem_master/soem/osal/linux
        soem_master/soem/oshw/linux
)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES 
        soem_master/SoemMaster.cpp 
        soem_master/SoemDevice.cpp
        devices/motor/MotorDevice.cpp
        devices/motor/math_ops.cpp
        devices/rm_imu/RmImuDevice.cpp
)
LINK_LIBRARIES(m)
add_library(soem_master ${SOURCES})
target_link_libraries(soem_master soem)