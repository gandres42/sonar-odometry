#include "sonar_odom.hpp"
#include <cstdint>

#define SONAR_MAX_DIST 5
#define SONAR_BUFFER_LENGTH 3
#define USE_COMPASS true
#define COMPASS_HEADING 196
#define USE_ARUCO false

SonarOdom::SonarOdom() {
    sonar_sub = node.subscribe<sensor_msgs::Image>("/oculus/drawn_sonar", 1, &SonarOdom::sonar_cb, this);
    imu_sub = node.subscribe<sensor_msgs::Imu>("/mavros/imu/data_raw", 1, &SonarOdom::imu_cb, this);
    mag_sub = node.subscribe<sensor_msgs::MagneticField>("/mavros/imu/mag", 1, &SonarOdom::mag_cb, this);
    pose_sub = node.subscribe<geometry_msgs::PoseStamped>("/aruco/pose", 1, &SonarOdom::aruco_cb, this);
    pose_pub = node.advertise<nav_msgs::Odometry>("/sianat/pose", 1);

    // message buffers
    uint64_t one_second_ago = ros::Time::now().toNSec() - static_cast<uint64_t>(1e9);
    aruco_frame = {0, 0, 0, 0, 0, one_second_ago};
    sonar_frame = {nullptr, one_second_ago};
    imu_frame = {one_second_ago};

    // fusion filter construction
    FusionAhrsInitialise(&ahrs);

    // kalman filter construction
    filter = KalmanFilter(0.1, A, C_positionless, Q, R, P);
    filter.init();
}

void SonarOdom::aruco_cb(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    this->aruco_frame = {
        msg->pose.position.x, 
        msg->pose.position.y,
        msg->pose.position.z,
        msg->pose.orientation.z,
        msg->pose.orientation.z * 180.0f / M_PI,
        ros::Time::now().toNSec()
    };
}

void SonarOdom::mag_cb(const sensor_msgs::MagneticField::ConstPtr& msg) {
    this->mag_frame = {
        {float(msg->magnetic_field.x), float(msg->magnetic_field.y), float(msg->magnetic_field.z)},
        ros::Time::now().toNSec()
    };
}

void SonarOdom::imu_cb(const sensor_msgs::Imu::ConstPtr& msg) {
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
    float imu_dt = (ros::Time::now().toNSec() - this->imu_frame.time_ns) / 1e9;
    float mag_dt = (ros::Time::now().toNSec() - this->mag_frame.time_ns) / 1e9;
    float aruco_dt = (ros::Time::now().toNSec() - this->aruco_frame.time_ns) / 1e9;

    // update IMU with either magnetometer or aruco as external heading
    if (USE_COMPASS) {
        if (mag_dt <= 0.1) {
            FusionAhrsUpdate(&this->ahrs, gyro, accel, this->mag_frame.mag, imu_dt);
        }
        else {
            FusionAhrsUpdateNoMagnetometer(&this->ahrs, gyro, accel, imu_dt);
        }
    }
    else {
        if (aruco_dt <= 0.1) {
            FusionAhrsUpdateExternalHeading(&this->ahrs, gyro, accel, this->aruco_frame.yaw_deg, imu_dt);
        }
        else {
            FusionAhrsUpdateNoMagnetometer(&this->ahrs, gyro, accel, imu_dt);
        }
    }
    this->imu_frame = {ros::Time::now().toNSec()};

    // publish pose using imu filter and raw acceleration
    nav_msgs::Odometry pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map_ned";
    pose_msg.pose.pose.position.x = this->filter.state().transpose()[0];
    pose_msg.pose.pose.position.y = this->filter.state().transpose()[1];
    pose_msg.pose.pose.position.z = this->filter.state().transpose()[2];
    pose_msg.twist.twist.linear.x = msg->angular_velocity.x;
    pose_msg.twist.twist.linear.y = msg->angular_velocity.y;
    pose_msg.twist.twist.linear.z = msg->angular_velocity.z;

    // get rotation and adjust by compass offset if needed
    FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
    if (USE_COMPASS) {
        AngleAxisf compass_rot(float(360 - COMPASS_HEADING) * M_PI / 180.0f, Vector3f::UnitZ());
        Quaternionf q_rotated = compass_rot * Quaternionf(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
        quat.array[0] = q_rotated.w();
        quat.array[1] = q_rotated.x();
        quat.array[2] = q_rotated.y();
        quat.array[3] = q_rotated.z();
    }
    pose_msg.pose.pose.orientation.w = quat.array[0];
    pose_msg.pose.pose.orientation.x = quat.array[1];
    pose_msg.pose.pose.orientation.y = quat.array[2];
    pose_msg.pose.pose.orientation.z = quat.array[3];

    this->pose_pub.publish(pose_msg);
}

void SonarOdom::sonar_cb(const sensor_msgs::Image::ConstPtr& msg) {
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

    // use BFMatcher to find matches
    std::vector<std::vector<cv::DMatch>> matches;
    this->bf->knnMatch(des_a, des_b, matches, 2);

    // perform RANSAC to eliminate bad matches
    std::vector<cv::Point2f> pts_a, pts_b;
    for (const auto& match_pair : matches) {
        pts_a.push_back(kp_a[match_pair[0].queryIdx].pt);
        pts_b.push_back(kp_b[match_pair[0].trainIdx].pt);
    }
    if (pts_a.size() >= 5 && pts_b.size() >= 5) {
        std::vector<uchar> inliers_mask;
        cv::findHomography(pts_a, pts_b, cv::RHO, 1, inliers_mask, 4000, 0.999);

        std::vector<std::vector<cv::DMatch>> inlier_matches;
        for (size_t i = 0; i < inliers_mask.size(); ++i) {
        if (inliers_mask[i]) {
            inlier_matches.push_back(matches[i]);
        }
        }
        matches = inlier_matches;
    }

    // matches visualization
    if (this->visualize) {
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
    MatrixXf matched_A(2, matched_pts_a.size());
    MatrixXf matched_B(2, matched_pts_b.size());
    for (size_t i = 0; i < matched_pts_a.size(); ++i) {
        matched_A(0, i) = matched_pts_a[i][0];
        matched_A(1, i) = matched_pts_a[i][1];
        matched_B(0, i) = matched_pts_b[i][0];
        matched_B(1, i) = matched_pts_b[i][1];
    }

    // get current ahrs rotation and extract only rotation about z
    FusionQuaternion quat = FusionAhrsGetQuaternion(&this->ahrs);
    Quaternionf q;
    if (USE_COMPASS) {
        AngleAxisf compass_rot(float(360 - COMPASS_HEADING) * M_PI / 180.0f, Vector3f::UnitZ());
        q = compass_rot * Quaternionf(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
    }
    else {
        q = Quaternionf(quat.array[0], quat.array[1], quat.array[2], quat.array[3]);
    }
    Matrix3f rot3d = q.toRotationMatrix();

    // rotate translation to match global z rotation
    float yaw = rot3d.eulerAngles(0, 1, 2)[2];
    Matrix2f rot2d = (Matrix2f() << cos(yaw), -sin(yaw), sin(yaw), cos(yaw)).finished();
    matched_A = rot2d * matched_A;
    matched_B = rot2d * matched_B;

    // SVD with default of no translation in degenerate cases
    Vector2f t = (Vector2f() << 0, 0).finished();
    if (matched_A.cols() > 4 || matched_B.cols() > 4) {
        Vector2f centroid_A = matched_A.rowwise().mean();
        Vector2f centroid_B = matched_B.rowwise().mean();
        MatrixXf AA = matched_A.colwise() - centroid_A;
        MatrixXf BB = matched_B.colwise() - centroid_B;
        Matrix2f H = AA * BB.transpose();
        if (!H.isZero(1e-6)) {
        JacobiSVD<Matrix2f> svd(H, ComputeFullU | ComputeFullV);
        Matrix2f R = svd.matrixV() * svd.matrixU().transpose();
        if (R.determinant() < 0) {
            Matrix2f V = svd.matrixV();
            V.col(1) *= -1;
            R = V * svd.matrixU().transpose();
        }
        t = centroid_B - R * centroid_A;
        }
    }
    double dt = (msg->header.stamp.toNSec() - prev_time) / 1e9;
    float vx = t[1] / dt;
    float vy = -t[0] / dt;

    // get acceleration
    FusionVector accel = FusionAhrsGetLinearAcceleration(&this->ahrs);
    Vector3f accel_vec = (Vector3f() << accel.axis.x, accel.axis.y, accel.axis.z).finished();

    // State transition matrix A for 6-state model (x, y, z, vx, vy, vz)
    A = (MatrixXd(n, n) <<
        1, 0, 0, dt, 0,  0,
        0, 1, 0, 0,  dt, 0,
        0, 0, 1, 0,  0,  dt,
        0, 0, 0, 1,  0,  0,
        0, 0, 0, 0,  1,  0,
        0, 0, 0, 0,  0,  1
    ).finished();

    std::cout << "dt: " << dt << ", dx: " << vx << ", dy: " << vy << std::endl;
    float vz = 0.0f;

    // check if position is up to date, if true update filter with position
    float pose_dt = (ros::Time::now().toNSec() - this->aruco_frame.time_ns) / 1e9;
    // pose_dt = 0.2;
    if (pose_dt < 0.1) {
        std::cout << "using position" << std::endl;
        VectorXd sensor_meas = (VectorXd(m) << this->aruco_frame.x, this->aruco_frame.y, this->aruco_frame.z, vx, vy, 0).finished();
        this->filter.C = this->C;
        this->filter.update(sensor_meas, dt, A);
    }
    else {
        std::cout << "not using position" << std::endl;
        VectorXd sensor_meas = (VectorXd(m) << 0, 0, 0, vx, vy, 0).finished();
        this->filter.C = this->C_positionless;
        this->filter.update(sensor_meas, dt, A);
    }

    std::cout << this->filter.state()[0] << ", " << this->filter.state()[1] << ", " << this->filter.state()[0] << std::endl << "\n";
}