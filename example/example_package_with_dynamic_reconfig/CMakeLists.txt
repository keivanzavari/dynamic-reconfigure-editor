cmake_minimum_required(VERSION 3.1.0)
project(example_package_with_dynamic_reconfig)

## Find catkin macros and libraries
find_package(
  catkin
  REQUIRED
  COMPONENTS rospy
             dynamic_reconfigure
)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(cfg/ExampleDynamicParameters.cfg)

# Declare that this catkin package's runtime dependencies
catkin_package(
  CATKIN_DEPENDS
  rospy
  dynamic_reconfigure
)
