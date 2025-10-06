#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Dense>
#include "Fusion.h"
#include "kalman.hpp"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"

using namespace Eigen;

struct sonarFrame {
  cv_bridge::CvImagePtr cv_ptr;
  uint64_t time_ns;
};

struct magFrame {
  FusionVector mag;
  uint64_t time_ns;
};

struct arucoFrame {
  double x;
  double y;
  double z;
  double yaw;
  double yaw_deg;
  uint64_t time_ns;
};

struct imuFrame {
  uint64_t time_ns;
};

class SonarOdom {
private:
  bool visualize = true;
  bool heading_set = false;

  // ROS
  ros::NodeHandle node;
  ros::Subscriber sonar_sub;
  ros::Subscriber imu_sub;
  ros::Subscriber mag_sub;
  ros::Subscriber pose_sub;
  ros::Publisher pose_pub;

  // cv feature matching
  cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(
    cv::AKAZE::DESCRIPTOR_MLDB_UPRIGHT,
    0,
    3,
    0.001,
    4,
    4,
    cv::KAZE::DIFF_PM_G1
  );
  std::unique_ptr<cv::BFMatcher> bf = std::make_unique<cv::BFMatcher>(cv::NORM_HAMMING);
  std::deque<sonarFrame> frame_ptr_buffer;

  // rotation fusion filter
  FusionAhrs ahrs;

  // message buffers
  magFrame mag_frame;
  arucoFrame aruco_frame;
  sonarFrame sonar_frame;
  imuFrame imu_frame;

  // position kalman filter
  int n = 6;
  int m = 6;
  KalmanFilter filter;
  MatrixXd C = (MatrixXd(m, n) <<
    1, 0, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
  ).finished(); // measurement mapping matrix with position;

  MatrixXd C_positionless = (MatrixXd(m, n) <<
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1
  ).finished(); // measurement mapping matrix without position;

  MatrixXd P = (MatrixXd(n, n) <<
    1, 0, 0,  0,  0,  0,
    0, 1, 0,  0,  0,  0,
    0, 0, 1,  0,  0,  0,
    0, 0, 0, .6,  0,  0,
    0, 0, 0,  0, .6,  0,
    0, 0, 0,  0,  0, .6
  ).finished(); // Estimate error covariance

  MatrixXd Q = MatrixXd::Identity(n, n) * .01; // Process noise covariance
  MatrixXd R = MatrixXd::Identity(m, m) * .05; // Measurement noise covariance

  MatrixXd A = MatrixXd(n, n);

public:
  SonarOdom();
  void aruco_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg);
  void imu_cb(const sensor_msgs::Imu::ConstPtr& msg);
  void sonar_cb(const sensor_msgs::Image::ConstPtr& msg);
};