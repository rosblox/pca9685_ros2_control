cmake_minimum_required(VERSION 3.16)
project(pca9685_ros2_control_example LANGUAGES CXX)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  pca9685_hardware_interface
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


install(
  DIRECTORY launch config description
  DESTINATION share/pca9685_ros2_control_example
)


## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
