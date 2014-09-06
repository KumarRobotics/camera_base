#ifndef CAMERA_NODE_BASE_H_
#define CAMERA_NODE_BASE_H_

#include <thread>
#include <memory>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include "camera_base/camera_ros_base.h"

template <typename ConfigType>
class CameraNodeBase {
 public:
  CameraNodeBase(const ros::NodeHandle& nh) : is_acquire_(false), nh_(nh) {}

  CameraNodeBase(const CameraNodeBase&) = delete;
  CameraNodeBase& operator=(const CameraNodeBase&) = delete;
  virtual ~CameraNodeBase() = default;

  const ros::NodeHandle& nh() const { return nh_; }
  bool is_acquire() const { return is_acquire_; }

  void Run() {
    cfg_server_.setCallback(
        boost::bind(&CameraNodeBase::ConfigCb, this, _1, _2));
  }

  void End() { Stop(); }

  void Sleep() const { rate_->sleep(); }

  void ConfigCb(ConfigType& config, int level) {
    if (level < 0) {
      ROS_INFO("%s: %s", nh().getNamespace().c_str(),
               "Initializaing reconfigure server");
    }
    if (is_acquire()) {
      Stop();
    }
    Setup(config);
    SetRate(config.fps);
    Start();
  }

  virtual void Acquire() = 0;

  virtual void Setup(ConfigType& config) = 0;

 private:
  void SetRate(double fps) { rate_.reset(new ros::Rate(fps)); }

  void Start() {
    is_acquire_ = true;
    acquire_thread_.reset(new std::thread(&CameraNodeBase::Acquire, this));
  }

  void Stop() {
    if (!is_acquire_) return;
    is_acquire_ = false;
    acquire_thread_->join();
  }

  bool is_acquire_;
  ros::NodeHandle nh_;
  std::unique_ptr<ros::Rate> rate_;
  std::unique_ptr<std::thread> acquire_thread_;
  dynamic_reconfigure::Server<ConfigType> cfg_server_;
};

#endif  // CAMERA_NODE_BASE_H_
