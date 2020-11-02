cmake_minimum_required(VERSION 3.5)
project(ign_moveit2)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Testing and linting
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Executables
set(EXECUTABLE_0 ign_moveit2)
# Executable 0
add_executable(${EXECUTABLE_0} src/${EXECUTABLE_0}.cpp)
ament_target_dependencies(${EXECUTABLE_0}
    rclcpp
    moveit_ros_planning_interface
    moveit_msgs
)

# Install cpp targets
install(TARGETS
    ${EXECUTABLE_0}
    DESTINATION lib/${PROJECT_NAME}
)

# Install python modules
ament_python_install_module(
    ign_moveit2_py/ign_moveit2.py
)

# Install python scripts and examples
set(SCRIPTS_DIR scripts)
set(EXAMPLES_DIR examples)
install(PROGRAMS
    ${EXAMPLES_DIR}/example_joint_goal.py
    ${EXAMPLES_DIR}/example_pose_goal.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Setup the project
ament_package()
