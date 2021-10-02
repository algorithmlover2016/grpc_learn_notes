cmake_minimum_required(VERSION 3.8)
project(cpp_srvcli)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tutorial_interfaces REQUIRED)

# add exectable for server 
add_executable(server src/add_two_ints_server.cpp)
ament_target_dependencies(server
    rclcpp example_interfaces)

add_executable(server_github src/add_two_ints_server.github.cpp)
ament_target_dependencies(server_github
    rclcpp example_interfaces)

add_executable(server_custom_interface src/add_two_ints_server_custom_interface.cpp)
ament_target_dependencies(server_custom_interface
    rclcpp tutorial_interfaces)

# add exectuable for client
add_executable(client src/add_two_ints_client.cpp)
ament_target_dependencies(client
    rclcpp example_interfaces)

add_executable(client_github src/add_two_ints_client.github.cpp)
ament_target_dependencies(client_github
    rclcpp example_interfaces)

add_executable(client_custom_interface src/add_two_ints_client_custom_interface.cpp)
ament_target_dependencies(client_custom_interface
    rclcpp tutorial_interfaces)
install(TARGETS
    client
    server
    client_github
    server_github
    client_custom_interface
    server_custom_interface
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
