#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/PoseStamped.h"
#include "kalman.hpp"
#include "Fusion.h"
#include "FusionAhrs.h"

#define SONAR_MAX_DIST 5
#define SONAR_BUFFER_LENGTH 3

struct stampedFrame {
  cv_bridge::CvImagePtr cv_ptr;
  uint64_t time_ns;
};

struct stampedPosition {
  double x;
  double y;
  double z;
  double yaw;
  uint64_t time_ns;
};

class SonarOdom {
private:
  bool visualize = false;

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
    0.0005,
    4,
    4,
    cv::KAZE::DIFF_PM_G1
  );
  std::unique_ptr<cv::BFMatcher> bf = std::make_unique<cv::BFMatcher>(cv::NORM_HAMMING);
  std::deque<stampedFrame> frame_ptr_buffer;

  // rotation fusion filter
  FusionAhrs ahrs;
  uint64_t prev_imu_time;
  FusionVector mag_vector;
  uint64_t prev_mag_time;

  // position kalman filter
  KalmanFilter filter;
  int n;
  int m;
  Eigen::MatrixXd C;
  Eigen::MatrixXd C_positionless;

  // aruco tracking
  stampedPosition aruco_pose;

public:
  SonarOdom() {
    // subscribe to sonar image
    sonar_sub = node.subscribe<sensor_msgs::Image>("/oculus/drawn_sonar", 1, &SonarOdom::sonar_cb, this);
    imu_sub = node.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 1, &SonarOdom::imu_cb, this);
    mag_sub = node.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 1, &SonarOdom::mag_cb, this);
    pose_sub = node.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 1, &SonarOdom::pose_cb, this);
    pose_pub = node.advertise<geometry_msgs::PoseStamped>("/sianat/pose", 1);
  
    // aruco tracking setup
    aruco_pose = {0, 0, 0, 0, ros::Time::now().toNSec()};

    // fusion filter construction
    FusionAhrsInitialise(&ahrs);
    prev_imu_time = ros::Time::now().toNSec();
    mag_vector = {0, 0, 0};
    prev_mag_time = std::numeric_limits<uint64_t>::max();

    // kalman filter construction
    n = 6; // Number of states (x, y, z, vx, vy, vz)
    m = 6; // Number of measurements (px, py, pz, vx, vy, vz)
    double dt = 1.0/10.0;

    Eigen::MatrixXd A = (Eigen::MatrixXd(n, n) <<
      1, 0, 0, dt, 0,  0,
      0, 1, 0, 0,  dt, 0,
      0, 0, 1, 0,  0,  dt,
      0, 0, 0, 1,  0,  0,
      0, 0, 0, 0,  1,  0,
      0, 0, 0, 0,  0,  1
    ).finished();
    
    C = (Eigen::MatrixXd(m, n) <<
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
    ).finished(); // Output matrix

    C_positionless= (Eigen::MatrixXd(m, n) <<
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
    ).finished(); // Output matrix

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n) * 0.5; // Process noise covariance
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m) * 10; // Measurement noise covariance
    Eigen::MatrixXd P = (Eigen::MatrixXd(n, n) <<
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, .5, 0, 0,
      0, 0, 0, 0, .5, 0,
      0, 0, 0, 0, 0, .5
    ).finished(); // Estimate error covariance

    filter = KalmanFilter(dt,A, C_positionless, Q, R, P);
    filter.init();
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    float yaw = atan2(2.0f * (msg->pose.orientation.w * msg->pose.orientation.x + msg->pose.orientation.y * msg->pose.orientation.z),
        1.0f - 2.0f * (msg->pose.orientation.x * msg->pose.orientation.x + msg->pose.orientation.y * msg->pose.orientation.y));
    yaw = yaw * 180.0f / M_PI;
    this->aruco_pose = {
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z,
      yaw,
      ros::Time::now().toNSec()
    };
  }

  void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg) {
    this->prev_mag_time = ros::Time::now().toNSec();
    this->mag_vector = {float(msg->magnetic_field.x), float(msg->magnetic_field.y), float(msg->magnetic_field.z)};
  }

  void imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
    const FusionVector gyro = {
      float(msg->angular_velocity.x * 180.0 / M_PI),
      float(msg->angular_velocity.y * 180.0 / M_PI),
      float(msg->angular_velocity.z * 180.0 / M_PI)
    };
    const FusionVector accel = {
      float(msg->linear_acceleration.x / 9.80665),
      float(msg->linear_acceleration.y / 9.80665),
      float(msg->linear_acceleration.z / 9.80665)
    };
    float dt = (ros::Time::now().toNSec() - this->prev_imu_time) / 1e9;
    
    // skip magnetometer if it's been too long since reading

    bool pose_fresh = ((ros::Time::now().toNSec() - this->aruco_pose.time_ns) / 1e9) < 0.1;
    bool mag_fresh = ros::Time::now().toNSec() - 2e8 > this->prev_mag_time;
    // if (mag_fresh && pose_fresh) {
    //   FusionAhrsUpdateNoMagnetometer(&this->ahrs, gyro, accel, dt);
    // }
    // else if (pose_fresh) {

    // }
    if (mag_fresh) {
      FusionAhrsUpdateNoMagnetometer(&this->ahrs, gyro, accel, dt);
    }
    else {
      FusionAhrsUpdate(&this->ahrs, gyro, accel, this->mag_vector, dt);
    }

    // if (pose_fresh) {
    //   std::cout << this->aruco_pose.yaw << std::endl;
    //   FusionAhrsSetHeading(&this->ahrs, this->aruco_pose.yaw);
    // }

    this->prev_imu_time = ros::Time::now().toNSec();

    FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = this->filter.state().transpose()[0];
    pose_msg.pose.position.y = this->filter.state().transpose()[1];
    pose_msg.pose.position.z = this->filter.state().transpose()[2];
    pose_msg.pose.orientation.w = quat.array[0];
    pose_msg.pose.orientation.x = quat.array[1];
    pose_msg.pose.orientation.y = quat.array[2];
    pose_msg.pose.orientation.z = quat.array[3];
    this->pose_pub.publish(pose_msg);

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
  }

  void sonar_cb(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    // image preprocessing
    cv::flip(cv_ptr->image, cv_ptr->image, 0);
    int target_height = 100 * SONAR_MAX_DIST;
    int target_width = static_cast<int>(cv_ptr->image.cols * (static_cast<float>(target_height) / cv_ptr->image.rows));
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(target_width, target_height));

    // add new frame, pop previous frame from buffer
    this->frame_ptr_buffer.push_back({
      cv_ptr, 
      msg->header.stamp.toNSec()
    });
    if (this->frame_ptr_buffer.size() < SONAR_BUFFER_LENGTH) {
      return;
    }
    cv_bridge::CvImagePtr prev_image = this->frame_ptr_buffer.at(0).cv_ptr;
    uint64_t prev_time = this->frame_ptr_buffer.at(0).time_ns;
    this->frame_ptr_buffer.pop_front();

    // skip if recieve out-of-order frames
    if (msg->header.stamp.toNSec() < prev_time) {
      return;
    }
    
    // perform AKAZE feature matching
    std::vector<cv::KeyPoint> kp_a;
    cv::Mat des_a;
    this->akaze->detectAndCompute(cv_ptr->image, cv::noArray(), kp_a, des_a);
    
    std::vector<cv::KeyPoint> kp_b;
    cv::Mat des_b;
    this->akaze->detectAndCompute(prev_image->image, cv::noArray(), kp_b, des_b);

    // use BFMatcher to find best matches
    std::vector<std::vector<cv::DMatch>> prelim_matches;
    std::vector<std::vector<cv::DMatch>> matches;
    this->bf->knnMatch(des_a, des_b, prelim_matches, 2);

    // filter out only the good matches from BFMatcher
    for (int i = 0; i < prelim_matches.size(); i++) {
      cv::DMatch* m = &prelim_matches[i][0];
      cv::DMatch* n = &prelim_matches[i][1];

      if (m->distance < (0.75 * n->distance)) {
        matches.push_back(prelim_matches[i]);
      }    
    }

    // perform RANSAC to eliminate bad matches
    std::vector<cv::Point2f> pts_a, pts_b;
    for (const auto& match_pair : matches) {
      pts_a.push_back(kp_a[match_pair[0].queryIdx].pt);
      pts_b.push_back(kp_b[match_pair[0].trainIdx].pt);
    }

    if (pts_a.size() >= 4 && pts_b.size() >= 4) {
      std::vector<uchar> inliers_mask;
      cv::findHomography(pts_a, pts_b, cv::LMEDS, 8, inliers_mask);

      std::vector<std::vector<cv::DMatch>> inlier_matches;
      for (size_t i = 0; i < inliers_mask.size(); ++i) {
        if (inliers_mask[i]) {
          inlier_matches.push_back(matches[i]);
        }
      }
      matches = inlier_matches;
    }

    // matches visualization
    if (this->visualize && !matches.empty()) {
      cv::Mat img_matches;
      std::vector<cv::DMatch> flat_matches;
      for (const auto& m : matches) {
        flat_matches.push_back(m[0]);
      }
      cv::drawMatches(
        cv_ptr->image, kp_a,
        prev_image->image, kp_b,
        flat_matches, img_matches,
        cv::Scalar::all(-1), cv::Scalar::all(-1),
        std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
      );
      cv::imshow("AKAZE Matches", img_matches);
      cv::waitKey(1);
    }

    // extract points, convert to metric scale
    std::vector<std::array<float, 2>> matched_pts_a;
    std::vector<std::array<float, 2>> matched_pts_b;
    for (int i = 0; i < matches.size(); i++) {
      
      matched_pts_a.push_back({
        (kp_a[matches[i][0].queryIdx].pt.x - (cv_ptr->image.cols / 2)) / 100,
        (kp_a[matches[i][0].queryIdx].pt.y) / 100
      });
      matched_pts_b.push_back({
        (kp_b[matches[i][0].trainIdx].pt.x - (cv_ptr->image.cols / 2)) / 100,
        (kp_b[matches[i][0].trainIdx].pt.y) / 100
      });
    }

    // Convert to Eigen matrices
    Eigen::MatrixXf A(2, matched_pts_a.size());
    Eigen::MatrixXf B(2, matched_pts_b.size());
    for (size_t i = 0; i < matched_pts_a.size(); ++i) {
      A(0, i) = matched_pts_a[i][0];
      A(1, i) = matched_pts_a[i][1];
      B(0, i) = matched_pts_b[i][0];
      B(1, i) = matched_pts_b[i][1];
    }

    // align with SVD
    Eigen::Vector2f centroid_A = A.rowwise().mean();
    Eigen::Vector2f centroid_B = B.rowwise().mean();
    Eigen::MatrixXf AA = A.colwise() - centroid_A;
    Eigen::MatrixXf BB = B.colwise() - centroid_B;
    Eigen::Matrix2f H = AA * BB.transpose();
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
    if (R.determinant() < 0) {
      Eigen::Matrix2f V = svd.matrixV();
      V.col(1) *= -1;
      R = V * svd.matrixU().transpose();
    }
    Eigen::Vector2f t = centroid_B - R * centroid_A;

    // get current ahrs rotation and extract only rotation about z
    FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
    Eigen::Quaternionf q(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
    Eigen::Matrix3f rot3d = q.toRotationMatrix();

    // rotate translation to match global z rotation
    float yaw = atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
              1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    Eigen::Matrix2f rot_z;
    rot_z << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    t = rot_z * t;

    double dt = (msg->header.stamp.toNSec() - prev_time) / 1e9;
    float vx = t[0] / dt;
    float vy = t[1] / dt;
    float vz = 0.0f;

    // filter dynamics matrix
    A = Eigen::MatrixXf::Zero(n, n);
    A(0, 0) = 1.0f; A(0, 3) = dt;
    A(1, 1) = 1.0f; A(1, 4) = dt;
    A(2, 2) = 1.0f; A(2, 5) = dt;
    A(3, 3) = 1.0f;
    A(4, 4) = 1.0f;
    A(5, 5) = 1.0f;

    // check if position is up to date, if true update with position
    float pose_dt = (ros::Time::now().toNSec() - this->aruco_pose.time_ns) / 1e9;
    // pose_dt = 0.2;
    if (pose_dt < 0.1) {
      this->filter.C = this->C;
      Eigen::Vector6d sensor_meas;
      sensor_meas << -this->aruco_pose.z, this->aruco_pose.x, this->aruco_pose.y, vy, -vx, 0;
      this->filter.update(sensor_meas, dt, A.cast<double>());
    }
    else {
      this->filter.C = this->C_positionless;
      Eigen::Vector6d sensor_meas;
      sensor_meas << 0, 0, 0, vy, vx, 0;
      this->filter.update(sensor_meas, dt, A.cast<double>());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  SonarOdom odom;
  ros::spin();
  return 0;
}

