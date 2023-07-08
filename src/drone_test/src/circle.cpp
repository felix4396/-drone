#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <drone_test/detection.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <drone_test/detection.h>
#include <drone_test/code.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <drone_test/detection.h>
#include <mavros_msgs/CommandBool.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <drone_test/code.h>
using namespace cv;
using namespace std;

Mat color_image;

// 获取相机彩色图像
void colorCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        color_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    waitKey(1);
    // imshow("color", color_image);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_detection");

    ros::NodeHandle nh;
    image_transport::ImageTransport it2(nh);
    // ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection_2", 1000);
    image_transport::Subscriber color_sub = it2.subscribe("/d400/color/image_raw", 2, colorCb);
    // drone_test::detection hhh;
    // hhh.start_opencv = 1;
    ros::Rate rate(30);
    while (ros::ok)
    {
        /* code */
        ros::spinOnce();
        imshow("color2", color_image);
        rate.sleep();
    }

    // int flag = 0;
    // detection_pub.publish(hhh);
    // while (ros::ok())
    // {
    //     flag++;
    //     ROS_INFO("flag = %d", flag);
    //     if (flag > 300)
    //     {
    //         flag = 0;
    //         hhh.start_opencv = 2;
    //     }
    //     detection_pub.publish(hhh);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    return 0;
}

// int minH = 0, maxH = 175;
// int minS = 0, maxS = 49;
// int minV = 0, maxV = 111;
// Mat elemment_l = getStructuringElement(MORPH_RECT, Size(3, 3));
// Mat elemment_h = getStructuringElement(MORPH_RECT, Size(7, 7));

// int start_opencv = 0;
// int dt = 1;
// int code_num = 100;
// class PIDController
// {
// public:
//     PIDController() : Kp(0), Ki(0), Kd(0), min_output(0), max_output(0), last_error(0), integral(0) {}

//     void setParameters(double kp, double ki, double kd, double minoutput, double maxoutput)
//     {
//         Kp = kp;
//         Ki = ki;
//         Kd = kd;
//         min_output = minoutput;
//         max_output = maxoutput;
//     }

//     double update(double target, double feedback)
//     {
//         double error = target - feedback;
//         integral += error * dt;
//         double derivative = (error - last_error) / dt;
//         double output = Kp * error + Ki * integral - Kd * derivative;
//         last_error = error;
//         output = std::max(output, min_output);
//         output = std::min(output, max_output);
//         return output;
//     }

// private:
//     double Kp, Ki, Kd;
//     double min_output, max_output;
//     double last_error;
//     double integral;
// };

// void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
// {
//     start_opencv = msg->start_opencv;
// }

// void docode_num(const drone_test::code::ConstPtr &msg)
// {
//     code_num = 100 - (msg->code) * 5;
// }

// int main(int argc, char **argv)
// {

//     ros::init(argc, argv, "star_detection");

//     ros::NodeHandle nh;

//     ros::Rate rate(30);

//     ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection", 1000);

//     ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_2", 10, doOpencv_sub);
//     ros::Subscriber code_sub = nh.subscribe<drone_test::code>("detection_code", 10, docode_num);

//     drone_test::detection line_erro;
//     drone_test::detection circle_erro;

//     ROS_INFO("ok");

//     Mat frame;
//     Mat hsv, mask, mask_1, mask_green;
//     Mat edge;
//     VideoCapture cap(200);

//     // vector<Vec3f> circles;

//     if (!cap.isOpened())
//     {
//         cout << "Failed to open camera." << endl;
//         return -1;
//     }
//     namedWindow("Trackbars", WINDOW_NORMAL);
//     resizeWindow("Trackbars", 640, 480);

//     createTrackbar("Min H", "Trackbars", &minH, 255);
//     createTrackbar("Max H", "Trackbars", &maxH, 255);
//     createTrackbar("Min S", "Trackbars", &minS, 255);
//     createTrackbar("Max S", "Trackbars", &maxS, 255);
//     createTrackbar("Min V", "Trackbars", &minV, 255);
//     createTrackbar("Max V", "Trackbars", &maxV, 255);

//     // pid
//     PIDController controller_green_circle_x;
//     PIDController controller_green_circle_y;
//     controller_green_circle_x.setParameters(3.0, 0.0, 0.1, -200.0, 200.0);
//     controller_green_circle_y.setParameters(3.0, 0.0, 0.1, -200.0, 200.0);

//     double dt = 1.0;
//     double dx = 0.0;
//     double dy = 0.0;
//     int flag = 0;
//     // do
//     // {
//     //     rate.sleep();
//     //     ros::spinOnce();
//     // } while (start_opencv != 1 && ros::ok);

//     while (ros::ok)
//     {
//         vector<Vec3f> circles;
//         cap >> frame;
//         if (frame.empty())
//         {
//             cout << "Failed to capture frame." << endl;
//             break;
//         }
//         cvtColor(frame, hsv, COLOR_BGR2HSV);
//         // inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
//         // imshow("Mask", mask);
//         inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask_green);
//         dilate(mask_green, mask_green, elemment_l);
//         erode(mask_green, mask_green, elemment_l);
//         dilate(mask_green, mask_green, elemment_h);
//         Canny(mask_green, edge, 100, 200);
//         imshow("mask_green", mask_green);
//         imshow("edge", edge);
//         HoughCircles(edge, circles, HOUGH_GRADIENT, 1, 300000, 100, 5, 100, 200);

//         Point imageCenter(frame.cols / 2, frame.rows / 2);
//         for (size_t i = 0; i < circles.size(); i++)
//         {
//             Point center;

//             center.x = cvRound(circles[i][0]);
//             center.y = cvRound(circles[i][1]);

//             int radius = cvRound(circles[i][2]);
//             circle(frame, center, radius, Scalar(0, 0, 255), 3);
//             ROS_INFO("center.x=%d,center.y=%d", center.x, center.y);
//             dx = double(center.x);
//             dy = double(center.y + code_num);
//             ROS_INFO("dx=%f,dy=%f", dx, dy);
//             // ROS_INFO("radius=%d", radius);
//         }
//         dx = controller_green_circle_x.update(imageCenter.x, dx);
//         dy = controller_green_circle_y.update(imageCenter.y, dy);
//         circle_erro.erro_x = dx;
//         circle_erro.erro_y = dy;
//         circle_erro.flag = 1;
//         detection_pub.publish(circle_erro);
//         // cout << "green_circle_dx=" << dx << endl;
//         // cout << "green_circle_dy=" << dy << endl;
//         if (abs(dx) < 35 && abs(dy) < 35)
//         {
//             flag++;
//             cout << "ok" << endl;
//             if (flag > 10)
//             {
//                 flag = 0;
//                 circle_erro.start_opencv = 3;
//                 cout << "green_circle is finish" << endl;
//                 // wait for sometimes and do somethings
//             }
//         }

//         imshow("frame", frame);
//         rate.sleep();
//         if (waitKey(33) == 'q')
//         {
//             break;
//         }
//     }

//     return 0;
// }