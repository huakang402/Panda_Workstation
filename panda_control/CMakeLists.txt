cmake_minimum_required(VERSION 2.8.3)
project(panda_control)

find_package(catkin REQUIRED COMPONENTS
    panda_moveit_config
)

catkin_package()

catkin_python_setup()
