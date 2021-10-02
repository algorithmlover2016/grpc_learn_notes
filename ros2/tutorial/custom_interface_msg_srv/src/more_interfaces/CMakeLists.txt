cmake_minimum_required(VERSION 3.8)
project(more_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_tutorials_msgs REQUIRED)

set(msg_files
  "msg/AddressBook.msg"
  "msg/AddressBookArr.msg"
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   ${msg_files}
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES rosidl_tutorials_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# for Build the publisher
add_executable(publish_address_book
  src/publish_address_book.cpp
)
ament_target_dependencies(publish_address_book
  rclcpp
)

add_executable(publish_address_book_array
  src/publish_address_book_array.cpp
)
ament_target_dependencies(publish_address_book_array
  "rclcpp" rosidl_tutorials_msgs
)

install(TARGETS
        publish_address_book
        publish_address_book_array
        DESTINATION lib/${PROJECT_NAME})

rosidl_target_interfaces(publish_address_book
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# if not add the following command, the more_interface will not find the "more_interface/msg/address_book_arr.hpp"
rosidl_target_interfaces(publish_address_book_array
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
