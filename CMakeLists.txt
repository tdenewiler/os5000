cmake_minimum_required(VERSION 2.8.3)
project(os5000)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure roscpp tf)

generate_dynamic_reconfigure_options(cfg/os5000.cfg)

catkin_package(CATKIN_DEPENDS dynamic_reconfigure roscpp tf)

include_directories(include ${catkin_INCLUDE_DIRS})

set(SRCS ${SRCS} src/nodes/os5000_node.cpp)
set(SRCS ${SRCS} src/os5000_core.cpp)

add_executable(${PROJECT_NAME} ${SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
