cmake_minimum_required(VERSION 3.8)
project(unity_launch)

# find dependencies
find_package(ament_cmake REQUIRED)


install(
        DIRECTORY launch rviz
        DESTINATION share/unity_launch
)

ament_package()
