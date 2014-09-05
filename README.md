# camera_base

Some base classes for simplifing ROS camera driver node.
## Basics

For writing a new ros camera driver, you need to inherit and implement the following to base classes.

### camera_node_base

Base class for a camera node. The node will have a dynamic reconfigure server.
```(c++)
virtual void Acquire() = 0;
virtual void Setup(ConfigType& config) = 0;
```

### camera_ros_base

Base class for a ros camera. A Ros camera will have the following common features:

* Camera Publisher
* Camera Info Manager
* Diagnostic Updater

```(c++)
virtual bool Grab(const sensor_msgs::ImagePtr& image_msg) = 0;
```

## ROS API 

### Published topics
`image_raw` ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))    
    The unprocessed image data.

`camera_info` ([sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))    
Contains the camera calibration (if calibrated) and extra data about the camera configuration.

### Services
`set_camera_info` ([sensor_msgs/SetCameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html))  
Set the appropriate camera info (TF frame, calibration parameters, ROI etc.)

## Style
C++11  
Google style
