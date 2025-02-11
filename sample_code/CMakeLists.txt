cmake_minimum_required(VERSION 2.8.3)
project(sample_code)

find_package(catkin REQUIRED
	roscpp
	rospy
	sensor_msgs
)

catkin_package(CATKIN_DEPENDS
  roscpp
  rospy
  sensor_msgs
)

include_directories(
	${catkin_INCLUDE_DIRS}
)


# ===== C++ Needs to Include the Following Command =====
# add_executable(<your_executable_name> <path_to_cpp_file>)
# target_link_libraries(<your_executable_name> ${catkin_LIBRARIES})

# example:
add_executable(cpp_sample src/cpp_sample.cpp)
target_link_libraries(cpp_sample ${catkin_LIBRARIES})


# ===== Python Needs to Include the Following Command =====
# catkin_install_python(PROGRAMS
# DESTINATION DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# example:
catkin_install_python(PROGRAMS 
  scripts/py_sample.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
