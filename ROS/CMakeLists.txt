list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic")
list(APPEND CMAKE_PREFIX_PATH "/opt/ros/melodic/share/catkin/cmake")
find_package(catkin REQUIRED COMPONENTS
        roscpp
        )

#catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

# ROS_node
add_executable( run_dso_node run_dso_node.cpp )
target_link_libraries( run_dso_node
  ldso ${THIRD_PARTY_LIBS}
        ${catkin_LIBRARIES})
