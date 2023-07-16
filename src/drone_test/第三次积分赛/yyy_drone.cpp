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

using namespace cv;
using namespace std;
using namespace mavros_msgs;

/*----------------------------------------------------------------------------------------------------------------------------------- */
ros::Publisher set_raw_pub;
geometry_msgs::PoseStamped current_pose, pose;
mavros_msgs::State current_state;
mavros_msgs::DebugValue PosAlt_atate;

/*----------------------------------------------------------------------------------------------------------------------------------- */
// 最大速度
double max_vel = 0.2;
double max_yaw_vel = 0.25;
double max_rate = 36;

// 临时速度
double yaw_vel = 0;

// 辅助变量
float posx_t265_local;
float posy_t265_local;
float posz_t265_local;

// 飞控状态
int Alt_mode;
int Pos_mode;
int is_RC_control;
bool connected, armed;

// 按钮状态(Jeston nano)
char bz_button_last = 0, bz_button = 0, state_button = 0;

// 图像消息_速度
double opencv_erro_x = 0;
double opencv_erro_y = 0;

// 图像消息_状态
int opencv_flag = -1;
int start_opencv = 0;

// 旋转角度
double yaw = 0;
double yaw_mode = 0;
double yaw_mode_takeoff = 0;

// 整圈检测辅助变量
int flag_stop_loop = 0;
int flag_stop_loop_1 = 0;

// 第一次杆子的颜色
int flag_1st_color = 0;
int flag_2rd_color = 0;

// 目标深度值
double target = 500 / 1000;

// 计时器
int time_count = 0;

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

void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    opencv_erro_x = (double)(msg->erro_x);
    opencv_erro_y = (double)(msg->erro_y);

    opencv_erro_x = opencv_erro_x / 200.0 * max_rate; // 相当于归一化
    opencv_erro_y = opencv_erro_y / 300.0 * max_vel;

    // 限幅输出
    if (opencv_erro_x >= max_rate)
    {
        opencv_erro_x = max_rate;
    }
    if (opencv_erro_x <= -1.0 * max_rate)
    {
        opencv_erro_x = -1.0 * max_rate;
    }
    if (opencv_erro_y >= max_vel)
    {
        opencv_erro_y = max_vel;
    }
    if (opencv_erro_y <= -1.0 * max_vel)
    {
        opencv_erro_y = -1.0 * max_vel;
    }
    opencv_flag = msg->flag;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_pose = *msg;
    // ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
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

void t265_callback(const nav_msgs::Odometry &msg)
{
    // pthread_mutex_lock(&mutex_t265);
    posx_t265_local = -msg.pose.pose.position.x;
    posy_t265_local = -msg.pose.pose.position.y;
    posz_t265_local = msg.pose.pose.position.z;
    // pthread_mutex_unlock(&mutex_t265);
}

void button_callback(const std_msgs::Byte::ConstPtr &msg) // 按键
{
    // pthread_mutex_lock(&mutex_button);
    bz_button = msg->data;
    // pthread_mutex_unlock(&mutex_button);
    // ROS_INFO("button_recived:%d\n", (int)(bz_button));
}

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
    ROS_INFO("x=%.2lf,y=%.2lf,z=%.2lf,yaw=%.2lf", x, y, z, yaw_rate);
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
/*----------------------------------------------------------------------------------------------------------------------------------- */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "star_drone");
    ros::NodeHandle nh;
    ros::Rate rate(20);

    /*----------------------------------------------------------------------------------------------------------------------------------- */

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    // ros::Subscriber t265_sub = nh.subscribe("/camera/odom/sample", 10, t265_callback);
    // ros::Subscriber button_sub = nh.subscribe<std_msgs::Byte>("/button", 10, button_callback); // button
    ros::Subscriber PA_state_sub = nh.subscribe<mavros_msgs::DebugValue>("/mavros/debug_value/debug_vector", 10, PosAlt_state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 10, odomCallback);
    ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);

    /*----------------------------------------------------------------------------------------------------------------------------------- */

    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection", 1, doOpencv_sub);
    ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection_2", 10);

    /*----------------------------------------------------------------------------------------------------------------------------------- */

    set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
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
    ros::Duration(3).sleep();
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
    //         arm_drone(nh);
    //         ros::Duration(1).sleep();
    //         ros::spinOnce();
    //     } while (!current_state.armed);
    //     ROS_INFO("Suc Armed");
    // }

    ros::spinOnce();
    yaw_mode_takeoff = yaw;
    /*-------------------------------------------------------起飞--------------------------------------------------------------------------- */
    ROS_INFO("Ready to Takeoff_1");
    ros::spinOnce();
    takeoff(nh, 1.0); // meters
    ros::Duration(0.5).sleep();
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (ros::ok() && Alt_mode != Position_ControlMode_Position);

    if (0)
    {
        ROS_INFO("NO Camera");
        set_pose_body(1.0, 0.0, 0.0, 0.0);
        ros::Duration(0.5).sleep();
        do
        {
            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && Pos_mode != Position_ControlMode_Position);
        // time_count = 0;
        // do
        // {
        //     set_speed_enu(0.2, 0.0, 0.0, 0.0);
        //     rate.sleep();
        //     ros::spinOnce();
        // } while (ros::ok && time_count++ < 60);

        // time_count = 0;
        // do
        // {
        //     set_speed_enu(0.0, 0.0, 0.0, 30.0);
        //     rate.sleep();
        //     ros::spinOnce();
        // } while (ros::ok && time_count++ < 60);

        // time_count = 0;
        // do
        // {
        //     set_speed_enu(0.2, 0.0, 0.0, 0.0);
        //     rate.sleep();
        //     ros::spinOnce();
        // } while (ros::ok && time_count++ < 60);
    }
    else
    {
        /*---------------------------------------------------------开启图像--------------------------------------------------------------------- */
        ROS_INFO("Start Pictorial Control");

        do
        {
            drone_test::detection yopcv;
            yopcv.start_opencv = 1;
            detection_pub.publish(yopcv);
            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && (opencv_flag == 0 || opencv_flag == -1));
        flag_1st_color = opencv_flag;

        /*---------------------------------------------------------第一次找杆子--------------------------------------------------------------------- */
        ROS_INFO("Start Forward Pole");

        do
        {
            ros::spinOnce();
            set_speed_body(opencv_erro_y, 0.0, 0.0, opencv_erro_x);
            rate.sleep();
        } while (ros::ok() && abs(opencv_erro_y) >= 0.1);

        /*---------------------------------------------------------第一次绕杆--------------------------------------------------------------------- */
        ROS_INFO("Start Circle");

        // 转到起飞朝向
        ros::spinOnce();
        yaw_mode = yaw;

        // 整圈检测清零
        flag_stop_loop = 0;
        flag_stop_loop_1 = 0;

        // 根据颜色判断转向
        yaw_vel = (flag_1st_color == 1 ? max_yaw_vel : -1 * max_yaw_vel);

        do // 绕杆
        {
            ros::spinOnce();
            set_speed_body(opencv_erro_y, yaw_vel, 0.0, opencv_erro_x); // 0.3 为飞机轴距
            if (abs(yaw_mode - yaw) >= 30)
            {
                flag_stop_loop_1++;
            }
            if (flag_stop_loop_1 != 0 && abs(yaw_mode - yaw) <= 10)
            {
                flag_stop_loop++;
            }
            rate.sleep();
        } while (ros::ok() && flag_stop_loop < 5);

        /*-----------------------------------------------------------刹车&&后退------------------------------------------------------------------------ */
        ROS_INFO("Stop and Back");

        do // 绕杆
        {
            ros::spinOnce();
            set_speed_body(opencv_erro_y, 0.0, 0.0, opencv_erro_x); // 0.3 为飞机轴距
            rate.sleep();
        } while (ros::ok() && abs(opencv_erro_y) >= 0.1);

        /*-------------------------------------------------------旋转找杆子-------------------------------------------------------------------- */
        ROS_INFO("Spin and Find the Pole");

        yaw_mode = yaw;
        flag_2rd_color = (flag_1st_color == 1 ? 2 : 1);
        int flag_find_2rd = 0;

        do
        {
            // 开启第二阶段绕圈
            drone_test::detection yopcv;
            yopcv.start_opencv = 2;
            detection_pub.publish(yopcv);

            ros::spinOnce();
            if (abs(yaw_mode - yaw) > 90 && flag_find_2rd == 0)
                flag_find_2rd++;
            if (flag_find_2rd)
            {
                set_speed_body(0.0, 0.0, 0.0, -18.0);
                if (abs(yaw_mode - yaw) < 10)
                    flag_find_2rd--;
            }
            else
                set_speed_body(0.0, 0.0, 0.0, 18.0);

            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && (opencv_flag == flag_1st_color || opencv_flag == 0));

        /*---------------------------------------------------------第二次找杆子--------------------------------------------------------------------- */
        ROS_INFO("Start Forward Pole_2");
        do
        {
            ros::spinOnce();
            set_speed_body(opencv_erro_y, 0.0, 0.0, opencv_erro_x);
            rate.sleep();
        } while (ros::ok() && abs(opencv_erro_y) >= 0.1);

        /*-----------------------------------------------------------第二次绕圈------------------------------------------------------------------------ */
        ROS_INFO("Start Circle_2");

        // 转到起飞朝向
        ros::spinOnce();
        yaw_mode = yaw_mode_takeoff;

        // 整圈检测清零
        flag_stop_loop = 0;
        flag_stop_loop_1 = 0;

        // 根据颜色判断转向
        yaw_vel = flag_2rd_color == 1 ? max_yaw_vel : -1 * max_yaw_vel;

        do // 绕杆
        {
            ros::spinOnce();
            set_speed_body(opencv_erro_y, yaw_vel, 0.0, opencv_erro_x); // 0.3 为飞机轴距
            if (abs(yaw_mode - yaw) >= 30)
            {
                flag_stop_loop_1++;
            }
            if (flag_stop_loop_1 != 0 && abs(yaw_mode - yaw) <= 10)
            {
                flag_stop_loop++;
            }
            rate.sleep();
        } while (ros::ok() && flag_stop_loop < 5);

        /*-----------------------------------------------------------刹车&&后退------------------------------------------------------------------------ */
        ROS_INFO("Stop and Back");

        do // 绕杆
        {
            ros::spinOnce();
            set_speed_body(opencv_erro_y, 0.0, 0.0, opencv_erro_x); // 0.3 为飞机轴距
            rate.sleep();
        } while (ros::ok() && abs(opencv_erro_y) >= 0.1);

        set_speed_body(0.0, 0.0, 0.0, 0.0);
        rate.sleep();

        set_pose_body(-0.5, 0.0, 0.0, 0.0);
        ros::Duration(0.5).sleep();
        do
        {
            rate.sleep();
            ros::spinOnce();
        } while (ros::ok() && Pos_mode != Position_ControlMode_Position);
    }

    ros::Duration(3).sleep();
    ROS_INFO("Start to Land");
    land(nh);
    ros::Duration(1).sleep();
    return 0;
}
