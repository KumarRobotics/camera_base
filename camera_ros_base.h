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
        cnh_{nh, prefix},
        it_{cnh_},
        camera_pub_{it_.advertiseCamera("image_raw", 1)},
        cinfo_mgr_{cnh_},
        fps_{10},
        topic_diagnostic_{
            "image_raw", diagnostic_updater_,
            diagnostic_updater::FrequencyStatusParam(&fps_, &fps_, 0.1, 10),
            diagnostic_updater::TimeStampStatusParam(0, 0.05)} {
    nh_.param<std::string>("frame_id", frame_id_, "camera");
    nh_.getParam(ResolveName(prefix, "identifier"), identifier_);
    // Setup camera info manager
    std::string camera;
    std::string calib_url;
    nh_.getParam("camera", camera);
    nh_.getParam(ResolveName(prefix, "calib_url"), calib_url);
    if (cinfo_mgr_.setCameraName(camera) && cinfo_mgr_.validateURL(calib_url) &&
        cinfo_mgr_.loadCameraInfo(calib_url)) {
      if (!cinfo_mgr_.isCalibrated()) {
        ROS_WARN_STREAM(camera << " not calibarted.");
      }
    }
    image_msg_.reset(new sensor_msgs::Image());
    cinfo_msg_.reset(new sensor_msgs::CameraInfo(cinfo_mgr_.getCameraInfo()));
  }

  CameraRosBase(const CameraRosBase&) = delete;
  CameraRosBase& operator=(const CameraRosBase&) = delete;
  virtual ~CameraRosBase() = default;

  const std::string& identifier() const { return identifier_; }

  double fps() const { return fps_; }
  void set_fps(double fps) { fps_ = fps; }
  void SetHardwareId(const std::string& id) {
    diagnostic_updater_.setHardwareID(id);
  }

  void Publish(const ros::Time& time) {
    cinfo_msg_->header.stamp = time;
    cinfo_msg_->header.frame_id = frame_id_;
    image_msg_->header = cinfo_msg_->header;
    if (Grab(image_msg_)) {
      camera_pub_.publish(image_msg_, cinfo_msg_);
      topic_diagnostic_.tick(image_msg_->header.stamp);
    }
    diagnostic_updater_.update();
  }

  virtual bool Grab(const sensor_msgs::ImagePtr& image_msg) = 0;

 private:
  std::string ResolveName(const std::string& prefix, const std::string& param) {
    return nh_.resolveName(prefix.empty() ? param
                                          : ros::names::append(prefix, param));
  }

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

#endif  // ROS_CAMERA_BASE_H_
