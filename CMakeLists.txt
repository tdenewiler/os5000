cmake_minimum_required(VERSION 2.8.3)
project(os5000)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation rosconsole roscpp tf)

# Generate dynamic reconfigure files.
generate_dynamic_reconfigure_options (cfg/os5000.cfg)

## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES os5000
#  CATKIN_DEPENDS dynamic_reconfigure message_generation rosconsole roscpp
#  DEPENDS system_lib
#)

## Specify additional locations of header files
include_directories(
  include
  cfg/cpp
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
set (SRCS ${SRCS} src/os5000_core.cpp)
set (SRCS ${SRCS} src/os5000_ros.cpp)
set (SRCS ${SRCS} src/serial.cpp)
set (SRCS ${SRCS} src/timing.cpp)
add_executable(os5000_node ${SRCS})
target_link_libraries (os5000_node ${catkin_LIBRARIES})

## Mark executable scripts (Python etc.) for installation
## not required for python when using catkin_python_setup()
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS os5000 os5000_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

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
