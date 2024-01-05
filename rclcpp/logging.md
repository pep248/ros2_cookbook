# rclcpp: Logging

More info regarding the logging concepts can be found inside the [ROS 2 Humble Documentation: Logging](https://docs.ros.org/en/foxy/Tutorials/Demos/Logging-and-logger-configuration.html)


## C++

### Debug
```cpp
// printf style
RCLCPP_DEBUG(node->get_logger(), "My log message %d", 4);

// C++ stream style
RCLCPP_DEBUG_STREAM(node->get_logger(), "My log message " << 4);
```

### Info
```cpp
// printf style
RCLCPP_INFO_ONCE(node->get_logger(), "My log message %d", 4);

// C++ stream style
RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "My log message " << 4);
```

### Warn
```cpp
// printf style
RCLCPP_WARN_SKIPFIRST(node->get_logger(), "My log message %d", 4);

// C++ stream style
RCLCPP_WARN_STREAM_SKIPFIRST(node->get_logger(), "My log message " << 4);
```

### Error
```cpp
// printf style
RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "My log message %d", 4);

// C++ stream style
RCLCPP_ERROR_STREAM_THROTTLE(node->get_logger(), *node->get_lock(), 1000, "My log message " << 4);
```



## Changing the logging level

```cpp
#include <rclcpp/logger.hpp>

rclcpp::get_logger("nav2_costmap_2d").set_level(rclcpp::Logger::Level::Debug);
```
