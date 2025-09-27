#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Image.h"
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <memory>
#include <cv_bridge/cv_bridge.h>
#include <open3d/Open3D.h>
#include <Eigen/Dense>

#define SONAR_MAX_DIST 5

struct stampedFrame{
  cv_bridge::CvImagePtr cv_ptr;
  uint64_t time_ns;
};

class SonarOdom {
private:
  bool visualize = true;
  ros::NodeHandle n;
  ros::Subscriber sonar_sub;

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

public:
  SonarOdom() {
    sonar_sub = n.subscribe<sensor_msgs::Image>("/oculus/drawn_sonar", 1, &SonarOdom::sonar_cb, this);
  }

  void svd_align() {

  }

  void sonar_cb(const sensor_msgs::Image::ConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    
    // image preprocessing
    cv::flip(cv_ptr->image, cv_ptr->image, 0);
    int target_height = 100 * SONAR_MAX_DIST;
    int target_width = static_cast<int>(cv_ptr->image.cols * (static_cast<float>(target_height) / cv_ptr->image.rows));
    cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(target_width, target_height));

    // pop previous image and time from buffer
    this->frame_ptr_buffer.push_back({cv_ptr, ros::Time::now().toNSec()});
    if (this->frame_ptr_buffer.size() < 3) {
      return;
    }
    cv_bridge::CvImagePtr prev_image = this->frame_ptr_buffer.at(0).cv_ptr;
    uint64_t prev_time = this->frame_ptr_buffer.at(0).time_ns;
    this->frame_ptr_buffer.pop_front();
    
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
      cv::findHomography(pts_a, pts_b, cv::LMEDS, 1, inliers_mask);

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
      // std::cout << matched_pts_a.back()[0] << ", " << matched_pts_a.back()[1] << " -> " << matched_pts_b.back()[0] << ", " << matched_pts_b.back()[1] << std::endl;
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

    // Compute centroids
    Eigen::Vector2f centroid_A = A.rowwise().mean();
    Eigen::Vector2f centroid_B = B.rowwise().mean();

    // Center the points
    Eigen::MatrixXf AA = A.colwise() - centroid_A;
    Eigen::MatrixXf BB = B.colwise() - centroid_B;

    // Compute covariance matrix
    Eigen::Matrix2f H = AA * BB.transpose();

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix2f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2f R = svd.matrixV() * svd.matrixU().transpose();

    // Ensure a proper rotation (determinant = 1)
    if (R.determinant() < 0) {
      Eigen::Matrix2f V = svd.matrixV();
      V.col(1) *= -1;
      R = V * svd.matrixU().transpose();
    }

    // Compute translation
    Eigen::Vector2f t = centroid_B - R * centroid_A;

    // Print result
    std::cout << "Rotation:\n" << R << std::endl;
    std::cout << "Translation:\n" << t.transpose() << std::endl;
  }

  //   auto cloud_a = std::make_shared<open3d::geometry::PointCloud>();
  //   auto cloud_b = std::make_shared<open3d::geometry::PointCloud>();

  //   // Add points to cloud_a (red) and cloud_b (blue)
  //   for (const auto& pt : matched_pts_a) {
  //     cloud_a->points_.emplace_back(pt[0], pt[1], 0.0f);
  //     cloud_a->colors_.emplace_back(1.0, 0.0, 0.0); // Red
  //   }
  //   for (const auto& pt : matched_pts_b) {
  //     cloud_b->points_.emplace_back(pt[0], pt[1], 0.0f);
  //     cloud_b->colors_.emplace_back(0.0, 0.0, 1.0); // Blue
  //   }

  //   // ICP alignment
  //   open3d::pipelines::registration::RegistrationResult result = open3d::pipelines::registration::RegistrationICP(
  //       *cloud_a, *cloud_b, 0.5, Eigen::Matrix4d::Identity(),
  //       open3d::pipelines::registration::TransformationEstimationPointToPoint());
    
  //   std::cout << result.transformation_ << std::endl;
  // }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  SonarOdom odom;
  ros::spin();
  return 0;
}

