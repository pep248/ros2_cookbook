# rclpy: Logging

More info regarding the logging concepts can be found inside the [ROS 2 Humble Documentation: Logging](https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html)


## Python

### Debug
```py
node.get_logger().debug('My log message %d' % (4))
```

### Info
```py
node.get_logger().info('My log message %d' % (4))
```

### Warn
```py
node.get_logger().warning('My log message %d' % (4))
```

### Error
```py
node.get_logger().error('My log message %d' % (4))
```


## Changing the logging level

```cpp
#include <rclcpp/logger.hpp>

rclcpp::get_logger("nav2_costmap_2d").set_level(rclcpp::Logger::Level::Debug);
```
