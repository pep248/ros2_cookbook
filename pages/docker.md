# Docker and ROS2

Docker is well prepared to work with ROS2.

One habitual configuration is to have a set of ROS2 packages running on a Docker container, and another set of ROS2 packages running on another container, with the intention of sharing messages through topics like if they were on the same system.

An interesting example and motive to do this, is avoiding obsolescence of packages. Once a set of packages is successfully containerized into a Docker Image, it will work forever, independently of the updates of the individual packages. Also, it is possible to create this Docker images with pre-compiled packages, making it very convenient to deploy solutions into new environments. Allowing us to even define launching arguments for the container, providing said arguments to the ROS2 launch files and, therefore, modifying the way the Nodes will be launched way before the container has even been created.

All these are covered in the following repo:

