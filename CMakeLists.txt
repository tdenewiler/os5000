cmake_minimum_required (VERSION 2.8.3)
project (os5000)

find_package (catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation rosconsole roscpp tf)

# Generate dynamic reconfigure files.
generate_dynamic_reconfigure_options (cfg/os5000.cfg)

# Specify additional locations of header files.
include_directories (
  include
  cfg/cpp
  ${catkin_INCLUDE_DIRS}
)

# Declare a cpp executable.
set (SRCS ${SRCS} src/nodes/os5000_node.cpp)
set (SRCS ${SRCS} src/os5000_core.cpp)
set (SRCS ${SRCS} src/timing.cpp)
add_executable (os5000_node ${SRCS})
target_link_libraries (os5000_node ${catkin_LIBRARIES})
