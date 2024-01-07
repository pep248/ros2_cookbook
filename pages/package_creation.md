# Simple python publisher subscriber
Example package with commands to create ros2 packages


## C++
### Set up
Create the package
```sh
ros2 pkg create --build-type ament_cmake --dependencies ament_cmake ament_cmake_auto rclcpp <your_package_name>
# Launch
mkdir -p src/<your_package_name>/launch
# C++ nodes folder
mkdir -p src/<your_package_name>/src
# C++ include folder
mkdir -p src/<your_package_name>/include/<your_package_name>
```

## Python
### Set up
Create the package
```sh
ros2 pkg create --build-type ament_python --dependencies ament_python rclpy <your_package_name>
mkdir src/<your_package_name>/
# Launch
mkdir -p src/<your_package_name>/launch
# Python scripts (nodes) folder
mkdir -p src/<your_package_name>/scripts/
touch src/<your_package_name>/scripts/__init__.py
# Python src (include) folder
mkdir -p src/<your_package_name>/<your_package_name>/src
touch src/<your_package_name>/<your_package_name>/__init__.py
```

## Hybrid
### Set up
Create the package
```sh
ros2 pkg create --build-type ament_cmake --dependencies ament_cmake ament_cmake_python ament_cmake_auto rclcpp rclpy <your_package_name>
# Launch
mkdir -p src/<your_package_name>/launch
# C++ nodes folder
mkdir -p src/<your_package_name>/src
# C++ include folder
mkdir -p src/<your_package_name>/include/<your_package_name>
# Python scripts (nodes) folder
mkdir -p src/<your_package_name>/scripts/
touch src/<your_package_name>/scripts/__init__.py
# Python src (include) folder
mkdir -p src/<your_package_name>/<your_package_name>/src
touch src/<your_package_name>/<your_package_name>/__init__.py
```
