cmake_minimum_required(VERSION 3.0.2)
project(scanbot_hwiface)

find_package(boost_system REQUIRED)
find_package(async_comm REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    roscpp
    hardware_interface
    controller_manager
  DEPENDS
    async_comm
)

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${async_comm_LIBRARIES})
