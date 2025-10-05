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
#define SONAR_BUFFER_LENGTH 2
#define USE_COMPASS true
#define COMPASS_HEADING 196
<<<<<<< Updated upstream
=======
#define USE_ARUCO false
>>>>>>> Stashed changes

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
  std::deque<stampedFrame> frame_ptr_buffer;

  // rotation fusion filter
  FusionAhrs ahrs;
  uint64_t prev_imu_time;
  FusionVector mag_vector;
  uint64_t prev_mag_time;

  // position kalman filter
  int n;
  int m;
  KalmanFilter filter;
  Eigen::MatrixXd C;
  Eigen::MatrixXd C_positionless;
  Eigen::MatrixXd A;

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
<<<<<<< Updated upstream
    n = 6; // Number of states (x, y, z, dx, dy, dz)
    m = 6; // Number of measurements (px, py, pz, dx, dy, dz)
    double dt = 1.0/10.0;

    A = Eigen::MatrixXd(n, n);
    
=======
    double dt = 1.0/10.0;

    A = Eigen::MatrixXd(n, n);

    // Remove linear acceleration as a state: n = 6, m = 6
    n = 6; // Number of states (x, y, z, vx, vy, vz)
    m = 6; // Number of measurements (x, y, z, vx, vy, vz)

>>>>>>> Stashed changes
    C = (Eigen::MatrixXd(m, n) <<
      1, 0, 0, 0, 0, 0,
      0, 1, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
    ).finished(); // measurement mapping matrix with position

<<<<<<< Updated upstream
    C_positionless= (Eigen::MatrixXd(m, n) <<
=======
    C_positionless = (Eigen::MatrixXd(m, n) <<
>>>>>>> Stashed changes
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0,
      0, 0, 0, 0, 0, 1
    ).finished(); // measurement mapping matrix without position

    Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(n, n) * .01; // Process noise covariance
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(m, m) * .02; // Measurement noise covariance
    Eigen::MatrixXd P = (Eigen::MatrixXd(n, n) <<
      1, 0, 0,  0,  0,  0,
      0, 1, 0,  0,  0,  0,
      0, 0, 1,  0,  0,  0,
      0, 0, 0, .2,  0,  0,
      0, 0, 0,  0, .2,  0,
      0, 0, 0,  0,  0, .2
    ).finished(); // Estimate error covariance

    // initialize filter with no position updates
    filter = KalmanFilter(dt, A, C_positionless, Q, R, P);
    filter.init();
  }

  void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    this->aruco_pose = {
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z,
      msg->pose.orientation.z,
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

    // update IMU with either magnetometer or aruco as external heading
    if (USE_COMPASS) {
      bool mag_fresh = ros::Time::now().toNSec() - 2e8 > this->prev_mag_time;
      if (mag_fresh) {
        FusionAhrsUpdateNoMagnetometer(&this->ahrs, gyro, accel, dt);
      }
      else {
        FusionAhrsUpdate(&this->ahrs, gyro, accel, this->mag_vector, dt);
      }
    }
    else {
      bool pose_fresh = ((ros::Time::now().toNSec() - this->aruco_pose.time_ns) / 1e9) < 0.15;
      if (pose_fresh) {
        float aruco_yaw_deg = this->aruco_pose.yaw * 180.0f / M_PI;
        FusionAhrsUpdateExternalHeading(&this->ahrs, gyro, accel, aruco_yaw_deg, dt);
      }
      else {
        FusionAhrsUpdateNoMagnetometer(&this->ahrs, gyro, accel, dt);
      }
    }
    this->prev_imu_time = ros::Time::now().toNSec();

    // publish pose with madgwick and kalman filter states
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map_ned";
    pose_msg.pose.pose.position.x = this->filter.state().transpose()[0];
    pose_msg.pose.pose.position.y = this->filter.state().transpose()[1];
    pose_msg.pose.pose.position.z = this->filter.state().transpose()[2];
    pose_msg.twist.twist.linear.x = this->filter.state().transpose()[3];
    pose_msg.twist.twist.linear.y = this->filter.state().transpose()[4];
    pose_msg.twist.twist.linear.z = this->filter.state().transpose()[5];

    // rotate pose by compass offset if needed
    if (USE_COMPASS) {
      FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
      Eigen::AngleAxisf compass_rot(float(360 - COMPASS_HEADING) * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
      Eigen::Quaternionf q_rotated = compass_rot * Eigen::Quaternionf(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
      quat.array[0] = q_rotated.w();
      quat.array[1] = q_rotated.x();
      quat.array[2] = q_rotated.y();
      quat.array[3] = q_rotated.z();
      pose_msg.pose.orientation.w = quat.array[0];
      pose_msg.pose.orientation.x = quat.array[1];
      pose_msg.pose.orientation.y = quat.array[2];
      pose_msg.pose.orientation.z = quat.array[3];
    }
    else {
      FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
      pose_msg.pose.pose.orientation.w = quat.array[0];
      pose_msg.pose.pose.orientation.x = quat.array[1];
      pose_msg.pose.pose.orientation.y = quat.array[2];
      pose_msg.pose.pose.orientation.z = quat.array[3];
    }

    this->pose_pub.publish(pose_msg);
  }

  void sonar_cb(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    // image preprocessing
    cv::flip(cv_ptr->image, cv_ptr->image, 0);
    cv::GaussianBlur(cv_ptr->image, cv_ptr->image, cv::Size(3, 3), 3);
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

      if (m->distance < (0.25 * n->distance)) {
        matches.push_back(prelim_matches[i]);
      }    
    }

    // perform RANSAC to eliminate bad matches
    std::vector<cv::Point2f> pts_a, pts_b;
    for (const auto& match_pair : matches) {
      pts_a.push_back(kp_a[match_pair[0].queryIdx].pt);
      pts_b.push_back(kp_b[match_pair[0].trainIdx].pt);
    }
    if (pts_a.size() >= 8 && pts_b.size() >= 8) {
      std::vector<uchar> inliers_mask;
      cv::findHomography(pts_a, pts_b, cv::RHO, 1, inliers_mask, 4000, 0.99);

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
    Eigen::MatrixXf matched_A(2, matched_pts_a.size());
    Eigen::MatrixXf matched_B(2, matched_pts_b.size());
    for (size_t i = 0; i < matched_pts_a.size(); ++i) {
      matched_A(0, i) = matched_pts_a[i][0];
      matched_A(1, i) = matched_pts_a[i][1];
      matched_B(0, i) = matched_pts_b[i][0];
      matched_B(1, i) = matched_pts_b[i][1];
    }

    // get current ahrs rotation and extract only rotation about z
    FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
    Eigen::Quaternionf q;
    if (USE_COMPASS) {
      Eigen::AngleAxisf compass_rot(float(360 - COMPASS_HEADING) * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
      q = compass_rot * Eigen::Quaternionf(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
    }
    else {
      q = Eigen::Quaternionf(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
    }
    Eigen::Matrix3f rot3d = q.toRotationMatrix();

    // rotate translation to match global z rotation
    auto euler = rot3d.eulerAngles(0, 1, 2);
    float yaw = euler[2];
    Eigen::Matrix2f rot2d;
    rot2d << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);

    matched_A = rot2d * matched_A;
    matched_B = rot2d * matched_B;

    // SVD with default of no translation in degenerate cases
    Eigen::Vector2f t = (Eigen::Vector2f() << 0, 0).finished();
    if (matched_A.cols() > 4 || matched_B.cols() > 4) {
      Eigen::Vector2f centroid_A = matched_A.rowwise().mean();
      Eigen::Vector2f centroid_B = matched_B.rowwise().mean();
      Eigen::MatrixXf AA = matched_A.colwise() - centroid_A;
      Eigen::MatrixXf BB = matched_B.colwise() - centroid_B;
      Eigen::Matrix2f H = AA * BB.transpose();
      if (!H.isZero(1e-6)) {
        Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
        if (R.determinant() < 0) {
          Eigen::Matrix2f V = svd.matrixV();
          V.col(1) *= -1;
          R = V * svd.matrixU().transpose();
        }
        t = centroid_B - R * centroid_A;
      }
    }

    // t = rot2d * t;

    double dt = (msg->header.stamp.toNSec() - prev_time) / 1e9;
    float vx = t[1] / dt;
    float vy = -t[0] / dt;

    // get acceleration
    FusionVector accel = FusionAhrsGetLinearAcceleration(&this->ahrs);
    Eigen::Vector3f accel_vec = (Eigen::Vector3f() << accel.axis.x, accel.axis.y, accel.axis.z).finished();
    accel_vec = rot3d * accel_vec;

    // State transition matrix A for 6-state model (x, y, z, vx, vy, vz)
    A = (Eigen::MatrixXd(n, n) <<
      1, 0, 0, dt, 0,  0,
      0, 1, 0, 0,  dt, 0,
      0, 0, 1, 0,  0,  dt,
      0, 0, 0, 1,  0,  0,
      0, 0, 0, 0,  1,  0,
      0, 0, 0, 0,  0,  1
    ).finished();

    std::cout << "dt: " << dt << ", dx: " << dx << ", dy: " << dy << std::endl;
    float vz = 0.0f;

    // check if position is up to date, if true update filter with position
    float pose_dt = (ros::Time::now().toNSec() - this->aruco_pose.time_ns) / 1e9;
    // pose_dt = 0.2;
    if (pose_dt < 0.1) {
      std::cout << "using position" << std::endl;
      Eigen::VectorXd sensor_meas = (Eigen::VectorXd(n) << this->aruco_pose.x, this->aruco_pose.y, this->aruco_pose.z, vx, vy, 0).finished();
      this->filter.C = this->C;
      this->filter.update(sensor_meas, dt, A);
    }
    else {
      std::cout << "not using position" << std::endl;
      Eigen::VectorXd sensor_meas = (Eigen::VectorXd(n) << 0, 0, 0, vx, vy, 0).finished();
      this->filter.C = this->C_positionless;
      this->filter.update(sensor_meas, dt, A);
    }

    std::cout << this->filter.state()[0] << ", " << this->filter.state()[1] << ", " << this->filter.state()[0] << std::endl << "\n";
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  SonarOdom odom;
  ros::spin();
  return 0;
}

