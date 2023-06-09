cmake_minimum_required(VERSION 3.5)
project(monitor_cpp)

# detect ROS1 vs ROS2
find_package(catkin QUIET)

# detect ROS version
string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
set(ROS_DISTRO "ROS_${ROS_DISTRO}")
message("detected ROS_DISTRO=${ROS_DISTRO}")

if(catkin_FOUND)
    message("detected ROS1 (catkin_make)")
    
    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package.ros1.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

    find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
    )

    catkin_package(
        CATKIN_DEPENDS roscpp std_msgs
    )

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")    # enable c++11 (TensorRT requirement)
    include_directories(${catkin_INCLUDE_DIRS})
    add_definitions(-DROS1 -DROS_DISTRO=${ROS_DISTRO})

else()
    find_package(ament_cmake QUIET)

    if(ament_cmake_FOUND)
        message("detected ROS2 (ament_cmake)")

        configure_file(${CMAKE_CURRENT_SOURCE_DIR}/package.ros2.xml ${CMAKE_CURRENT_SOURCE_DIR}/package.xml COPYONLY)

        find_package(rclcpp REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(std_msgs REQUIRED)
        find_package(geometry_msgs REQUIRED)
        find_package(sensor_msgs REQUIRED)
        find_package(nav_msgs REQUIRED)
        find_package(ament_index_cpp REQUIRED)
        find_package(yaml-cpp REQUIRED)

        set(CMAKE_CXX_STANDARD 14)
        add_definitions(-DROS2 -DROS_DISTRO=${ROS_DISTRO})

    else()
        message(FATAL_ERROR "Could not find either ROS1 (catkin_make) or ROS2 (ament_cmake) packages")
    endif()
endif()

if(catkin_FOUND)
    add_executable(monitor_ros1_node src/monitor_ros1_node.cpp)
    target_link_libraries(monitor_ros1_node ${catkin_LIBRARIES} yaml-cpp)

else()
    add_executable(monitor_ros2_node src/monitor_ros2_node.cpp src/ros_compat.cpp)
    ament_target_dependencies(monitor_ros2_node rclcpp std_msgs sensor_msgs nav_msgs geometry_msgs ament_index_cpp yaml-cpp)
    target_link_libraries(monitor_ros2_node ${YAML_CPP_LIBRARIES})
    install(TARGETS monitor_ros2_node DESTINATION lib/${PROJECT_NAME})

    install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME}/)

    ament_package()
endif()
