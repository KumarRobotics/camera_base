# camera_base

Some base classes for simplifing ROS camera driver node.
## Basics

### camera_node_base

Base class for a camera node. The node will have a dynamic reconfigure server.

### camera_ros_base

Base class for a ros camera. A Ros camera will have the following common features:

* Camera Publisher
* Camera Info Manager
* Diagnostic Updater

## ROS API 

### Published topics
image_raw ([sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html))
The unprocessed image data.
camera_info (sensor_msgs/CameraInfo)
Contains the camera calibration (if calibrated) and extra data about the camera configuration.
