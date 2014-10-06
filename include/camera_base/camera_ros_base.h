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

namespace {
template <typename T>
T getParam(const ros::NodeHandle& nh, const std::string& name) {
  T value;
  if (!nh.getParam(name, value)) {
    ROS_ERROR("Cannot find parameter: %s", name.c_str());
  }
  return value;
}
}

namespace camera_base {

class CameraRosBase {
 public:
  CameraRosBase(const ros::NodeHandle& nh,
                const std::string& prefix = std::string())
      : nh_(nh),
        cnh_(nh, prefix),
        it_(cnh_),
        camera_pub_(it_.advertiseCamera("image_raw", 1)),
        cinfo_mgr_(cnh_, ::getParam<std::string>(cnh_, "camera_name"),
                   ::getParam<std::string>(cnh_, "calib_url")),
        image_msg_(new sensor_msgs::Image()),
        cinfo_msg_(new sensor_msgs::CameraInfo(cinfo_mgr_.getCameraInfo())),
        fps_(10),
        topic_diagnostic_(
            "image_raw", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(-0.01, 0.1)) {
    nh_.param<std::string>("frame_id", frame_id_, "camera");
    cnh_.param<std::string>("identifier", identifier_, "");
  }

  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  const std::string& identifier() const { return identifier_; }
  const std::string& frame_id() const { return frame_id_; }

  double fps() const { return fps_; }
  void set_fps(double fps) { fps_ = fps; }

  void SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

  void PublishCamera(const ros::Time& time) {
    image_msg_->header.frame_id = frame_id_;
    image_msg_->header.stamp = time;
    if (Grab(image_msg_, cinfo_msg_)) {
      // Update camera info header
      cinfo_msg_->header = image_msg_->header;
      camera_pub_.publish(image_msg_, cinfo_msg_);
      topic_diagnostic_.tick(image_msg_->header.stamp);
    }
    diagnostic_updater_.update();
  }

  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg,
                    const sensor_msgs::CameraInfoPtr& cinfo_msg) = 0;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle cnh_;
  image_transport::ImageTransport it_;
  image_transport::CameraPublisher camera_pub_;
  camera_info_manager::CameraInfoManager cinfo_mgr_;
  sensor_msgs::ImagePtr image_msg_;
  sensor_msgs::CameraInfoPtr cinfo_msg_;
  double fps_;
  diagnostic_updater::Updater diagnostic_updater_;
  diagnostic_updater::TopicDiagnostic topic_diagnostic_;
  std::string frame_id_;
  std::string identifier_;
};

}  // namespace camera_base

#endif  // ROS_CAMERA_BASE_H_
