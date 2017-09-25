cmake_minimum_required(VERSION 2.8.3)
project(acrobat_reward_calculation)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    tf
)

# catkin_python_setup()

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    tf
)

include_directories(
    ros/include
    ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES
add_executable(tf_euclidean_distance ros/src/tf_euclidean_distance.cpp)
target_link_libraries(tf_euclidean_distance ${catkin_LIBRARIES})
