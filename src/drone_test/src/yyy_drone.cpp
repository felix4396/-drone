#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/CommandHome.h>
#include <sensor_msgs/Range.h>
#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include <nav_msgs/Odometry.h>
#include <sys/time.h>
#include <pthread.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <mavros_msgs/DebugValue.h>
#include <drone_test/detection.h>
#include <tf/tf.h>
#include "std_msgs/Float64.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "drone_test/Y_Serial_port.h"
#include <typeinfo>

using namespace cv;
using namespace std;
using namespace mavros_msgs;

/*----------------------------------------------------------------------------------------------------------------------------------- */

enum Position_ControlMode
{

    Position_ControlMode_Null = 255,

    // Position_ControlMode_VelocityTrack = 16 ,
    Position_ControlMode_Position = 12,
    Position_ControlMode_Velocity = 11,
    Position_ControlMode_Locking = 10,

    // 2D
    Position_ControlMode_Takeoff = 20,
    Position_ControlMode_RouteLine = 22,

    // 3D
    Position_ControlMode_RouteLine3D = 52,
};

class PIDController
{
public:
    PIDController() : Kp(0), Ki(0), Kd(0), min_output(0), max_output(0), last_error(0), integral(0) {}

    void setParameters(double kp, double ki, double kd, double minoutput, double maxoutput)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        min_output = minoutput;
        max_output = maxoutput;
    }

    double update(double target, double feedback)
    {
        double dt = 1; // PID控制器采样时间
        double error = target - feedback;
        integral += error * dt;
        double derivative = (error - last_error) / dt;
        double output = Kp * error + Ki * integral + Kd * derivative;
        last_error = error;
        output = std::max(output, min_output);
        output = std::min(output, max_output);
        return output;
    }

private:
    double Kp, Ki, Kd;
    double min_output, max_output;
    double last_error;
    double integral;
};

/*----------------------------------------------------------------------------------------------------------------------------------- */
ros::Publisher set_raw_pub;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::TwistWithCovarianceStamped current_speed;
nav_msgs::Odometry current_Alt;
mavros_msgs::State current_state;
mavros_msgs::DebugValue PosAlt_atate;
PIDController pid_controller_set_break_speed;

/*----------------------------------------------------------------------------------------------------------------------------------- */
// 最大速度
double max_vel_x = 36;
double max_vel_y = 0.4;
double max_vel_z = 0.2;
double min_stop_vel = 0.05;

// 辅助变量
double posx_t265_local;
double posy_t265_local;
double posz_t265_local;

// 飞控状态
int Alt_mode;
int Pos_mode;
int is_RC_control;
bool connected, armed;
double yaw;

// 按钮状态(Jeston nano)
// char bz_button_last = 0, bz_button = 0, state_button = 0;
int bz_state_0 = 0, bz_state_1 = 6, bz_state_2 = 5;

// 图像消息_速度
double opencv_erro_x = 0;
double opencv_erro_y = 0;
double opencv_erro_z = 0;

// 图像消息_状态
int opencv_flag = -1;
int start_opencv = 0;

double now_position[2] = {0};
double now_speed[3] = {0};

// double x_pos[12] = {1.25, 2.75, 3.5, 4.25, 4.25, 3.5, 2, 2, 1.25, 2.75, 2.75, 4.25};
// double y_pos[12] = {3.5, 2, 2.75, 0.5, 3.5, 1.25, 1.25, 2.75, 2, 0.5, 3.5, 2};
double x_pos[12] = {0.498, 1.939, 2.641, 3.3423, 3.468, 2.608, 1.161, 1.185, 0.475, 1.880, 2.018, 3.397};
double y_pos[12] = {2.610, 1.059, 1.807, -0.468, 2.392, 0.324, 0.419, 1.860, 1.207, -0.341, 2.567, 1.032};

double first_lev[3] = {9, 7, 10};
double sec_lev[5] = {1, 8, 2, 6, 4};
double third_lev[3] = {11, 3, 12};
double forth_lev[1] = {5};
double now_alt = 0;

int fly_state = 0;
int fly_tar[2] = {bz_state_1, bz_state_2};
/*----------------------------------------------------------------------------------------------------------------------------------- */

void doPerson(const drone_test::Y_Serial_port::ConstPtr &person_p) // 接收串口数据
{
    // ROS_INFO("%d, %d, %d, %d", person_p->mode, person_p->tar1, person_p->tar2, person_p->if_takeoff);
}

void doOpencv_stage_sub(const drone_test::detection::ConstPtr &msg)
{
    opencv_erro_x = -(double)(msg->erro_x);
    opencv_erro_y = (double)(msg->erro_y);
    opencv_erro_z = -(double)(msg->erro_z);

    opencv_erro_x = opencv_erro_x / 300.0 * max_vel_x; // 相当于归一化
    opencv_erro_y = opencv_erro_y / 300.0 * max_vel_y;
    opencv_erro_z = opencv_erro_z / 300.0 * max_vel_z;

    // 限幅输出
    if (opencv_erro_x >= max_vel_x)
    {
        opencv_erro_x = max_vel_x;
    }
    if (opencv_erro_x <= -1.0 * max_vel_x)
    {
        opencv_erro_x = -1.0 * max_vel_x;
    }
    if (opencv_erro_y >= max_vel_y)
    {
        opencv_erro_y = max_vel_y;
    }
    if (opencv_erro_y <= -1.0 * max_vel_y)
    {
        opencv_erro_y = -1.0 * max_vel_y;
    }
    if (opencv_erro_z >= max_vel_z)
    {
        opencv_erro_z = max_vel_z;
    }
    if (opencv_erro_z <= -1.0 * max_vel_z)
    {
        opencv_erro_z = -1.0 * max_vel_z;
    }
    opencv_flag = msg->flag;
}

void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    opencv_erro_x = -(double)(msg->erro_x);
    opencv_erro_y = -(double)(msg->erro_y);
    opencv_erro_z = -(double)(msg->erro_z);

    opencv_erro_x = opencv_erro_x / 300.0 * max_vel_x; // 相当于归一化
    opencv_erro_y = opencv_erro_y / 300.0 * max_vel_y;
    opencv_erro_z = opencv_erro_z / 300.0 * max_vel_z;

    // 限幅输出
    if (opencv_erro_x >= max_vel_x)
    {
        opencv_erro_x = max_vel_x;
    }
    if (opencv_erro_x <= -1.0 * max_vel_x)
    {
        opencv_erro_x = -1.0 * max_vel_x;
    }
    if (opencv_erro_y >= max_vel_y)
    {
        opencv_erro_y = max_vel_y;
    }
    if (opencv_erro_y <= -1.0 * max_vel_y)
    {
        opencv_erro_y = -1.0 * max_vel_y;
    }
    if (opencv_erro_z >= max_vel_z)
    {
        opencv_erro_z = max_vel_z;
    }
    if (opencv_erro_z <= -1.0 * max_vel_z)
    {
        opencv_erro_z = -1.0 * max_vel_z;
    }
    opencv_flag = msg->flag;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    now_position[0] = current_pose.pose.position.x;
    now_position[1] = current_pose.pose.position.y;
    // ROS_INFO("x: %f y: %f", now_position[0], now_position[1]);
}

void get_speed(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &msg)
{
    current_speed = *msg;
    now_speed[0] = current_speed.twist.twist.linear.x;
    now_speed[1] = current_speed.twist.twist.linear.y;
    now_speed[2] = current_speed.twist.twist.linear.z;
    // ROS_INFO("now_speed_x:%f,now_speed_y:%f", now_speed[0], now_speed[1]);
}

void get_Alt(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_Alt = *msg;
    now_alt = current_Alt.twist.twist.linear.y + 0.06;
    // ROS_INFO("now_alt:%f", now_alt);
}

void PosAlt_state_cb(const mavros_msgs::DebugValue::ConstPtr &msg) // 状态机
{
    PosAlt_atate = *msg;
    Pos_mode = (int)PosAlt_atate.data[0];
    Alt_mode = (int)PosAlt_atate.data[1];
    is_RC_control = (int)PosAlt_atate.data[2];
    // ROS_INFO("Alt_mode:::%d\n",Alt_mode);
}

void state_cb(const mavros_msgs::State::ConstPtr &msg) // 飞控状态
{
    current_state = *msg;
    connected = current_state.connected;
    armed = current_state.armed;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    yaw = tf::getYaw(msg->pose.pose.orientation) * 180.0 / 3.1415926;
    // cout << "Yaw=" << yaw;
    // cout << "\r";
    // ROS_INFO("\r");
    // ROS_INFO("Yaw angle: %f", yaw);
}

// void button_callback(const std_msgs::Byte::ConstPtr &msg) // 按键
// {
//     // pthread_mutex_lock(&mutex_button);
//     bz_button = msg->data;
//     // pthread_mutex_unlock(&mutex_button);
//     // ROS_INFO("button_recived:%d\n", (int)(bz_button));
// }

int takeoff(ros::NodeHandle &nh, double height) // meters
{
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = height;
    if (takeoff_cl.call(srv_takeoff))
    {
        ROS_INFO("Takeoff sent %d", srv_takeoff.response.success);
    }
    else
    {
        ROS_ERROR("Failed Takeoff");
        return -1;
    }
    return 0;
}

int land(ros::NodeHandle &nh)
{
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    if (land_client.call(srv_land) && srv_land.response.success)
        ROS_INFO("land sent %d", srv_land.response.success);
    else
    {
        ROS_ERROR("Landing failed");
        ros::shutdown();
        return -1;
    }
    return 0;
}

int set_pose_body(double x, double y, double z, double yaw) // flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
{
    ROS_INFO("SET: X=%.2lf,Y=%.2lf,Z=%.2lf", x, y, z);
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;

    if (fabs(yaw) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW;
    else
        raw_target.yaw = yaw * 0.01745329;

    raw_target.position.x = x;
    raw_target.position.y = y;
    raw_target.position.z = z;

    raw_target.header.stamp = ros::Time::now();
    set_raw_pub.publish(raw_target);
    ros::spinOnce();
    ros::Duration(0.1).sleep();
    return 0;
}

int set_speed_body(double x, double y, double z, double yaw_rate) // flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
    raw_target.velocity.x = x;
    raw_target.velocity.y = y;
    raw_target.velocity.z = z;
    raw_target.yaw_rate = yaw_rate * 0.01745329;
    ROS_INFO("Speed x=%.2lf,y=%.2lf,z=%.2lf,yaw=%.2lf", x, y, z, yaw_rate);
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_speed_enu(double x, double y, double z, double yaw_rate) // flu meter / s rad / s,must set continously, or the vehicle stops after a few seconds(failsafe feature).yaw_rate = 0 when not used.
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_LOCAL_NED;
    raw_target.type_mask = PositionTarget::IGNORE_PX | PositionTarget::IGNORE_PY | PositionTarget::IGNORE_PZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW;
    if (fabs(yaw_rate) < 1e-6)
        raw_target.type_mask |= PositionTarget::IGNORE_YAW_RATE;
    raw_target.velocity.x = x;
    raw_target.velocity.y = y;
    raw_target.velocity.z = z;
    raw_target.yaw_rate = yaw_rate * 0.01745329;
    set_raw_pub.publish(raw_target);
    return 0;
}

int set_break()
{
    mavros_msgs::PositionTarget raw_target;
    raw_target.coordinate_frame = PositionTarget::FRAME_BODY_OFFSET_NED;
    raw_target.type_mask = PositionTarget::IGNORE_VX | PositionTarget::IGNORE_VY | PositionTarget::IGNORE_VZ | PositionTarget::IGNORE_AFX | PositionTarget::IGNORE_AFY | PositionTarget::IGNORE_AFZ | PositionTarget::IGNORE_YAW_RATE;
    raw_target.position.x = 0;
    raw_target.position.y = 0;
    raw_target.position.z = 0;
    raw_target.yaw = 0;
    set_raw_pub.publish(raw_target);

    ros::spinOnce();
    ros::Duration(0.1).sleep();
    return 0;
}

// 上锁
int arm_hand(ros::NodeHandle &nh)
{
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false; // true解锁 false上锁
    arming_client.call(arm_cmd);
    if (arm_cmd.response.success)
        ROS_INFO("Vehicle armed");
    return arm_cmd.response.success;
}

// 解锁
int arm_drone(ros::NodeHandle &nh)
{
    // arming
    ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    mavros_msgs::CommandBool srv_arm_i;
    srv_arm_i.request.value = true;
    if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
        ROS_INFO("ARM sent %d", srv_arm_i.response.success);
    else
    {
        ROS_ERROR("Failed arming");
        return -1;
    }
    return 0;
}

// 舵机
int set_servo(ros::NodeHandle &nh, int check_code, int state)
{
    ros::ServiceClient servo_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    mavros_msgs::CommandLong msg;
    msg.request.command = mavros_msgs::CommandCode::DO_SET_SERVO;
    msg.request.param1 = check_code;
    msg.request.param2 = state;

    if (servo_client_i.call(msg) && msg.response.success)
        ROS_INFO("servo msg sent");
    else
    {
        ROS_ERROR("Failed servo");
        return -1;
    }
    return 0;
}

// 切换OFFBOARD
void offboard_drone(ros::NodeHandle &nh)
{
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD"; // 设置模式为 OFFBOARD
    ros::spinOnce();
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("Offboard enabled"); // 如果请求服务成功且服务返回执行成功打印消息
    }
}

void set_break_2D_speed()
{
    int flag_stop = 0;
    do
    {
        ros::spinOnce();
        // ROS_INFO("now_speed_x:%f,now_speed_y:%f", now_speed[0], now_speed[1]);
        double break_speed_x = pid_controller_set_break_speed.update(0, now_speed[0]);
        double break_speed_y = pid_controller_set_break_speed.update(0, now_speed[1]);
        // ROS_INFO("break_speed_x:%f,break_speed_y:%f", break_speed_x, break_speed_y);
        set_speed_body(break_speed_x, break_speed_y, 0, 0);
        ros::Duration(0.05).sleep();
        ros::spinOnce();
        if (abs(now_speed[0]) < 0.05 && abs(now_speed[1]) < 0.05)
        {
            flag_stop++;
        }
        else
        {
            flag_stop = 0;
        }
    } while (flag_stop < 10 && ros::ok());
    set_break();
    do
    {
        ros::Duration(0.05).sleep();
        ros::spinOnce();
    } while (ros::ok() && Pos_mode != Position_ControlMode_Position);
}

/*----------------------------------------------------------------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "star_drone");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    pid_controller_set_break_speed.setParameters(0.2, 0.0, 0.02, -0.2, 0.2);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    // 飞控基础信息订阅
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 1, state_cb);
    // ros::Subscriber button_sub = nh.subscribe<std_msgs::Byte>("/button", 10, button_callback); // button
    ros::Subscriber PA_state_sub = nh.subscribe<mavros_msgs::DebugValue>("/mavros/debug_value/debug_vector", 1, PosAlt_state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, odomCallback);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1, pose_cb);
    ros::Subscriber currentSpeed = nh.subscribe<geometry_msgs::TwistWithCovarianceStamped>("/mavros/vision_speed/speed_twist_cov", 1, get_speed);
    ros::Subscriber currentAlt = nh.subscribe<nav_msgs::Odometry>("/mavros/global_position/local", 1, get_Alt);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    // 图像信息订阅
    ros::Subscriber sub = nh.subscribe<drone_test::Y_Serial_port>("serial_chatter", 1, doPerson);
    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_stage1", 1, doOpencv_sub);
    ros::Subscriber detection_sub_stage2 = nh.subscribe<drone_test::detection>("detection_stage2", 1, doOpencv_stage_sub);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    // 发布信息
    ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection_status", 10);
    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
    /*----------------------------------------------------------------------------------------------------------------------------------- */
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    /*------------------------------------------------------飞控自检----------------------------------------------------------------------- */
CHECK:
    ROS_INFO("Start Self-Test");
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && !current_state.connected);
    ROS_INFO("Suc Connect");
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && current_state.mode != "OFFBOARD");
    ROS_INFO("Suc OFFBOARD");
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && !current_state.armed);
    ROS_INFO("Suc Armed");
    ros::Duration(2).sleep();
    ros::spinOnce();
    if (current_state.mode != "OFFBOARD" || !current_state.armed)
    {
        goto CHECK;
    }

    // if (0) // WANGING!!!
    // {
    //     do
    //     {
    //         offboard_drone(nh);
    //         ros::Duration(1).sleep();
    //         ros::spinOnce();
    //     } while (current_state.mode != "OFFBOARD");
    //     ROS_INFO("Suc OFFBOARD");
    //     do
    //     {
    //         ros::Duration(1).sleep();
    //         ros::spinOnce();
    //     } while (ros::ok() && !current_state.connected);
    //     ROS_INFO("Suc Connect");
    //     do
    //     {
    //         arm_drone(nh);
    //         ros::Duration(1).sleep();
    //         ros::spinOnce();
    //     } while (!current_state.armed);
    //     ROS_INFO("Suc Armed");
    // }

    /*-------------------------------------------------------起飞--------------------------------------------------------------------------- */
    ROS_INFO("Ready to Takeoff_1");
    ros::spinOnce();
    ROS_INFO("now_alt:%f", now_alt);

    takeoff(nh, 0.65 - now_alt); // meters
    ros::Duration(0.5).sleep();
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

    if (0)
    {
        ROS_INFO("NO Camera");
        // set_pose_body(1.0, 0.0, 0.0, 0.0);
        // ros::Duration(0.5).sleep();
        // do
        // {
        //     rate.sleep();
        //     ros::spinOnce();
        // } while (ros::ok() && Pos_mode != Position_ControlMode_Position);
        int cnt = 0;
        do
        {
            rate.sleep();
            ros::spinOnce();
            set_speed_body(0.2, 0, 0, 0);
            cnt++;
        } while (ros::ok() && cnt < 60);
        set_break_2D_speed();

        ros::Duration(3).sleep();
    }
    else
    {
        /*----------------------------------------------------------第零阶段-------------------------------------------------------------------- */
        drone_test::detection yopcv;
        yopcv.start_opencv = 1;
        detection_pub.publish(yopcv);
        detection_pub.publish(yopcv);
        detection_pub.publish(yopcv);

        int end_flag = 0;
        do
        {
            rate.sleep();
            ros::spinOnce();
            set_speed_body(0.0, 0.0, 0.0, 18);
            if (abs(opencv_erro_y) > 0.03 || abs(opencv_erro_x) > 3)
                end_flag++;
            else
                end_flag = 0;
        } while (ros::ok() && end_flag <= 5);

        /*----------------------------------------------------------第一阶段-------------------------------------------------------------------- */
        ROS_INFO("Stage 1");
        ros::spinOnce();
        end_flag = 0;
        do
        {
            rate.sleep();
            ros::spinOnce();
            set_speed_body(opencv_erro_y, 0.0, 0.0, -opencv_erro_x);
            if (abs(opencv_erro_y) <= 0.05 && abs(opencv_erro_x) <= 5)
                end_flag++;
            else
                end_flag = 0;
        } while (ros::ok() && end_flag <= 10);

        /*----------------------------------------------------------第二阶段-------------------------------------------------------------------- */
        ROS_INFO("Stage 2");
        ros::spinOnce();
        double yaw_mode = yaw;
        double now_pos_x_now = now_position[0];
        double now_pos_y_now = now_position[1];

        set_pose_body(now_pos_x_now - now_position[0], now_pos_y_now - now_position[1], (1.1 - now_alt), yaw_mode - yaw); // meters
        ros::Duration(0.5).sleep();
        do
        {
            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

        /*----------------------------------------------------------第三阶段-------------------------------------------------------------------- */
        ROS_INFO("Stage 3");
        ros::spinOnce();
        yopcv.start_opencv = 2;
        detection_pub.publish(yopcv);
        detection_pub.publish(yopcv);
        detection_pub.publish(yopcv);

        end_flag = 0;
        do
        {
            rate.sleep();
            ros::spinOnce();
            set_speed_body(opencv_erro_z, opencv_erro_y, 0.0, -opencv_erro_x);
            if (abs(opencv_erro_y) <= 0.03 && abs(opencv_erro_z) <= 0.03 && abs(opencv_erro_x) <= 3)
                end_flag++;
            else
                end_flag = 0;
        } while (ros::ok() && end_flag < 15);

        /*----------------------------------------------------------第四阶段-------------------------------------------------------------------- */
        ROS_INFO("Stage 4");
        ros::spinOnce();
        yaw_mode = yaw;
        now_pos_x_now = now_position[0];
        now_pos_y_now = now_position[1];

        set_pose_body(now_pos_x_now - now_position[0], now_pos_y_now - now_position[1], (1.5 - now_alt), yaw_mode - yaw); // meters
        ros::Duration(0.5).sleep();
        do
        {
            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

        /*----------------------------------------------------------第五阶段-------------------------------------------------------------------- */
        ros::spinOnce();
        set_pose_body(0.5, 0, 0, 0.0); // meters
        ros::Duration(0.5).sleep();
        do
        {
            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && Pos_mode != Position_ControlMode_Position);
    }

    /*---------------------------------------------------------降落--------------------------------------------------------------------- */
    ROS_INFO("Start to Land");
    land(nh);
    ros::Duration(0.5).sleep();
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

    /*---------------------------------------------------------上锁--------------------------------------------------------------------- */
    do
    {
        arm_hand(nh);
        ros::Duration(1).sleep();
        ros::spinOnce();
    } while (ros::ok() && current_state.armed);

    return 0;
}