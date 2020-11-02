# Setup for Ignition Dome
find_package(ignition-cmake2 REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-msgs6 REQUIRED)
find_package(ignition-transport9 REQUIRED)
find_package(ignition-gazebo4 REQUIRED)
ign_find_package(ignition-plugin1 REQUIRED COMPONENTS register)
# Add sources for each plugin to be registered.
set(LIBRARY_TARGET ignition-gazebo-joint-trajectory-controller-system)
add_library(${LIBRARY_TARGET} SHARED JointTrajectoryController.cc)
set_property(TARGET ${LIBRARY_TARGET} PROPERTY CXX_STANDARD 17)
target_link_libraries(${LIBRARY_TARGET} ignition-common3 ignition-plugin1
                      ignition-msgs6 ignition-transport9 ignition-gazebo4)
ament_target_dependencies(${LIBRARY_TARGET} rclcpp trajectory_msgs)
install(TARGETS ${LIBRARY_TARGET}
        DESTINATION ${IGNITION_GAZEBO_PLUGIN_INSTALL_DIR})
