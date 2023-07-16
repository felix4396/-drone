#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <opencv2/core.hpp>
#include <opencv2/video.hpp>

class KalmanFilter {
public:
    KalmanFilter() : A(Eigen::Matrix4d::Identity()), B(Eigen::Matrix4d::Zero()), 
                     H(Eigen::Matrix<double, 3, 4>::Identity()), P(Eigen::Matrix<double, 4, 4>::Identity()), 
                     Q(Eigen::Matrix<double, 4, 4>::Identity()), R(Eigen::Matrix3d::Identity()), 
                     X(Eigen::Vector4d::Zero()), Z(Eigen::Vector3d::Zero()), u(Eigen::Vector4d::Zero()) {}

    void init(Eigen::Quaterniond q, Eigen::Vector3d v, Eigen::Vector3d a) {
        X << q.w(), q.x(), q.y(), q.z();
        Z << v, a;
    }

    Eigen::Quaterniond getQuaternion() const {
        return Eigen::Quaterniond(X(0), X(1), X(2), X(3));
    }

    Eigen::Vector3d getVelocity() const {
        return H * X;
    }

    Eigen::Vector3d getAcceleration() const {
        return Z;
    }

    void update(double dt) {
        // state transition matrix
        A.topRightCorner(3, 1) = Eigen::Vector3d(dt, 0, 0);
        A.bottomRightCorner(3, 1) = Eigen::Vector3d(0, dt, 0);
        A.bottomRightCorner(3, 1) = Eigen::Vector3d(0, 0, dt);

        // control input matrix
        B.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * dt * dt * 0.5;
        B.middleLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * dt;

        // process noise covariance
        Q.topLeftCorner(3, 3) = Eigen::Matrix3d::Identity() * 0.01;  // position
        Q.bottomRightCorner(3, 3) = Eigen::Matrix3d::Identity() * 0.1;  // velocity

        // measurement noise covariance
        R.topLeftCorner(2, 2) = Eigen::Matrix2d::Identity() * 0.01;  // position
        R.bottomRightCorner(1, 1) = Eigen::Matrix<double, 1, 1>::Identity() * 0.1;  // acceleration

        // time update (prediction)
        X = A * X + B * u;
        P = A * P * A.transpose() + Q;

        // measurement update (correction)
        Eigen::Matrix<double, 3, 4> K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
        Eigen::Vector3d y = Z - H * X;
        X = X + K * y;
        P = (Eigen::Matrix<double, 4, 4>::Identity() - K * H) * P;
    }

private:
    Eigen::Matrix4d A;
    Eigen::Matrix4d B;
    Eigen::Matrix<double, 3, 4> H;
    Eigen::Matrix<double, 4, 4> P;
    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix3d R;
    Eigen::Vector4d X;
    Eigen::Vector3d Z;
    Eigen::Vector4d u;
};

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    static ros::Time t_prev = msg->header.stamp;
    ros::Time t_cur = msg->header.stamp;
    double dt = (t_cur - t_prev).toSec();

    Eigen::Quaterniond q(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z);
    Eigen::Vector3d v(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d a(msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z);

    static KalmanFilter filter;
    filter.init(q, v, a);
    filter.update(dt);

    q = filter.getQuaternion();
    v = filter.getVelocity();
    a = filter.getAcceleration();

    // publish filtered pose
    geometry_msgs::PoseStamped filtered_pose;
    filtered_pose.header = msg->header;
    filtered_pose.pose.orientation.w = q.w();
    filtered_pose.pose.orientation.x = q.x();
    filtered_pose.pose.orientation.y = q.y();
    filtered_pose.pose.orientation.z = q.z();
    filtered_pose.pose.position.x = v(0);
    filtered_pose.pose.position.y = v(1);
    filtered_pose.pose.position.z = v(2);
    filtered_pose_publisher.publish(filtered_pose);

    // broadcast transform
    geometry_msgs::TransformStamped transform;
    transform.header = msg->header;
    transform.child_frame_id = "filtered_camera_frame";
    transform.transform.translation.x = v(0);
    transform.transform.translation.y = v(1);
    transform.transform.translation.z = v(2);
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    broadcaster.sendTransform(transform);

    t_prev = t_cur;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "kalman_filter");
    ros::NodeHandle nh;

    ros::Subscriber pose_subscriber = nh.subscribe("camera_pose", 10, poseCallback);
    filtered_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("filtered_camera_pose", 10);
    tf2_ros::TransformBroadcaster broadcaster;

    ros::spin();
    return 0;
}
