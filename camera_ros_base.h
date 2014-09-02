#ifndef CAMERA_ROS_BASE_H_
#define CAMERA_ROS_BASE_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <diagnostic_updater/publisher.h>
#include <diagnostic_updater/diagnostic_updater.h>

class CameraRosBase {
 public:
  CameraRosBase(const ros::NodeHandle& nh,
                const std::string& prefix = std::string())
      : nh_{nh},
        it_{nh},
        camera_publisher_{it_.advertiseCamera("image_raw", 1)},
        cinfo_manager_{nh},
        fps_{1},
        topic_diagnostic_{
            "image_raw", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(0, 2 / fps_)} {
    nh_.param<std::string>("frame_id", frame_id_, "camera");
    // Setup camera info manager
    std::string camera;
    std::string calib_url;
    nh_.getParam(ResolveParam(prefix, "camera"), camera);
    nh_.getParam(ResolveParam(prefix, "calib_url"), calib_url);
    if (cinfo_manager_.setCameraName(camera) &&
        cinfo_manager_.validateURL(calib_url) &&
        cinfo_manager_.loadCameraInfo(calib_url)) {
      if (!cinfo_manager_.isCalibrated()) {
        ROS_WARN_STREAM(camera << " not calibarted.");
      }
    }
    image_msg_.reset(new sensor_msgs::Image());
    cinfo_msg_.reset(
        new sensor_msgs::CameraInfo(cinfo_manager_.getCameraInfo()));
    diagnostic_updater_.setHardwareID("camera");
  }

  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  void set_fps(double fps) { fps_ = fps; }

  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg) = 0;

  void Publish(const ros::Time& time) {
    cinfo_msg_->header.stamp = time;
    cinfo_msg_->header.frame_id = frame_id_;
    image_msg_->header = cinfo_msg_->header;
    if (Grab(image_msg_)) {
      camera_publisher_.publish(image_msg_, cinfo_msg_);
      topic_diagnostic_.tick(image_msg_->header.stamp);
    }
    diagnostic_updater_.update();
  }

 private:
  std::string ResolveParam(const std::string& prefix,
                           const std::string& param) {
    return nh_.resolveName(prefix.empty() ? param
                                          : ros::names::append(prefix, param));
  }

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_publisher_;
  camera_info_manager::CameraInfoManager cinfo_manager_;
  sensor_msgs::ImagePtr image_msg_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;
  double fps_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::TopicDiagnostic topic_diagnostic_;
  std::string frame_id_;
};

#endif  // ROS_CAMERA_BASE_H_
