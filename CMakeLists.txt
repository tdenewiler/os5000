cmake_minimum_required(VERSION 2.8.3)
project(os5000)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp sensor_msgs tf)

find_package(Boost REQUIRED COMPONENTS system thread)

generate_dynamic_reconfigure_options(cfg/os5000.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure roscpp sensor_msgs tf)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(os5000 src/nodes/os5000_node.cpp src/os5000_asio.cpp src/os5000_core.cpp)
target_link_libraries(os5000 ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
