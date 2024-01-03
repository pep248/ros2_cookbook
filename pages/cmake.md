# CMake

While you don't need to know everything about CMake to use ROS2, knowing a bit
will really be helpful. You might be interested in the
[CMake tutorial](https://cmake.org/cmake/help/latest/guide/tutorial/index.html)
which explains the basics of CMake.

## Ament

Ament is a set of CMake modules specifically designed for ROS2 with the intent
of making CMake easier to use. See also the
[Ament CMake](https://index.ros.org/doc/ros2/Tutorials/Ament-CMake-Documentation/)
documentation.

The basic structure of an ament package:

```cmake
cmake_minimum_required(VERSION 3.8)
project(simple_cpp_python_publish_subscribe)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# DEPENDENCIES

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()


# CPP
# Add the action client executable
ament_auto_add_executable(publisher_cpp src/publisher.cpp)
# Install the executable
install(TARGETS publisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)
# Add the action client executable
ament_auto_add_executable(subscriber_cpp src/subscriber.cpp)
# Install the executable
install(TARGETS subscriber_cpp
  DESTINATION lib/${PROJECT_NAME}
)


# PYTHON
# Install Python modules
ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/publisher.py
  scripts/subscriber.py
  DESTINATION lib/${PROJECT_NAME}
)


# COPY A PARTICULAR FOLDER TO THE INSTALL DIRECTORY
# Install config dependencies
install(
  DIRECTORY
    config
  DESTINATION
    share/${PROJECT_NAME}
)


# LAUNCH
# Install launchfile
ament_auto_package(INSTALL_TO_SHARE launch)
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_msgs</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="rueda_999@hotmail.com">pep248</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rosidl_default_generators</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```


## Linting Configuration

I prefer a more ROS1-style code style. To allow braces to be on their
own lines:

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_cpplint)
  ament_cpplint(FILTERS "-whitespace/braces" "-whitespace/newline")
endif()
```

## Package defining a custom message

See the [Implementing custom interfaces tutorial](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Single-Package-Define-And-Use-Interface.html#link-against-the-interface) for newer ROS2 distributions.

:warning: Remeber that the name of the custom message has to start with Capital letter and CANNOT contain symbols.

```cmake
cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Custommessage.msg")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>custom_msgs</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="rueda_999@hotmail.com">pep248</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>rosidl_default_generators</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

## Removing Boost from Pluginlib

Pluginlib supports both boost::shared_ptrs and std::shared_ptrs by default,
if you want to avoid depending on Boost in your shiny new ROS2 library, you
need to specifically tell pluginlib not to include the Boost versions:

```cmake
target_compile_definitions(your_library PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
```

## Using Eigen3

Add _eigen_ to your package.xml as a dependency, and then:

```cmake
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
```

## Building Python Extensions in C++

The example below is based on the
[etherbotix](https://github.com/mikeferguson/etherbotix) package.

```cmake
find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED python)
find_package(ament_cmake_python REQUIRED)
find_package(python_cmake_module REQUIRED)

ament_python_install_package(${PROJECT_NAME})

add_library(
  my_python SHARED
  ${SOURCE_FILES}
)
set_target_properties(
  my_python PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
  PREFIX ""
)
target_link_libraries(my_python
  ${Boost_LIBRARIES}
  ${PYTHON_LIBRARIES}
)

install(
  TARGETS my_python
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
)
```
