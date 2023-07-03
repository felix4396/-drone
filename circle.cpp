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
using namespace cv;
using namespace std;

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "circle_detection");

//     ros::NodeHandle nh;
//     ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection_2", 1000);
//     drone_test::detection hhh;
//     hhh.start_opencv = 1;
//     ros::Rate rate(30);
//     int flag = 0;
//     while (ros::ok())
//     {
//         flag++;
//         ROS_INFO("flag = %d", flag);
//         if (flag > 1000)
//         {
//             flag = 0;
//             hhh.start_opencv = 2;
//         }
//         detection_pub.publish(hhh);
//         ros::spinOnce();
//         rate.sleep();
//     }

//     return 0;
// }
int minH = 57, maxH = 147;
int minS = 0, maxS = 180;
int minV = 0, maxV = 147;
Mat elemment_l = getStructuringElement(MORPH_RECT, Size(3, 3));
Mat elemment_h = getStructuringElement(MORPH_RECT, Size(7, 7));

int start_opencv = 0;

class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : m_kp(kp), m_ki(ki), m_kd(kd), m_integral(0), m_prevError(0)
    {
    }

    double update(double error, double dt)
    {
        if (error == 0)
        {
            error = m_prevError;
        }
        else if (abs(error - m_prevError) > 100)
        {
            error = 0.4 * m_prevError + 0.6 * error;
        }
        double derivative = (error - m_prevError) / dt;
        m_integral += error * dt;
        double output = m_kp * error + m_ki * m_integral + m_kd * derivative;
        m_prevError = error;
        return output;
    }

private:
    double m_kp;
    double m_ki;
    double m_kd;
    double m_integral;
    double m_prevError;
};

void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    start_opencv = msg->start_opencv;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "star_detection");

    ros::NodeHandle nh;

    ros::Rate rate(30);

    ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection", 1000);

    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_2", 10, doOpencv_sub);

    drone_test::detection line_erro;
    drone_test::detection circle_erro;

    ROS_INFO("ok");

    Mat frame;
    Mat hsv, mask, mask_1, mask_green;
    Mat edge;
    VideoCapture cap(6);

    // vector<Vec3f> circles;

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }
    // namedWindow("Trackbars", WINDOW_NORMAL);
    // resizeWindow("Trackbars", 640, 480);

    // createTrackbar("Min H", "Trackbars", &minH, 255);
    // createTrackbar("Max H", "Trackbars", &maxH, 255);
    // createTrackbar("Min S", "Trackbars", &minS, 255);
    // createTrackbar("Max S", "Trackbars", &maxS, 255);
    // createTrackbar("Min V", "Trackbars", &minV, 255);
    // createTrackbar("Max V", "Trackbars", &maxV, 255);

    // pid
    PIDController controller_green_circle_x(3, 0.0, 0.1);
    PIDController controller_green_circle_y(3, 0.0, 0.1);

    double dt = 1.0;
    double dx = 0.0;
    double dy = 0.0;
    int flag = 0;
    // do
    // {
    //     rate.sleep();
    //     ros::spinOnce();
    // } while (start_opencv != 1 && ros::ok);

    while (ros::ok)
    {
        vector<Vec3f> circles;
        cap >> frame;
        if (frame.empty())
        {
            cout << "Failed to capture frame." << endl;
            break;
        }

        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
        dilate(mask, mask, elemment_l);
        erode(mask, mask, elemment_l);
        dilate(mask, mask, elemment_h);

        imshow("Mask", mask);

        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask_green);
        Canny(mask_1, edge, 100, 200);
        imshow("mask_green", mask_green);
        imshow("edge", edge);
        HoughCircles(edge, circles, HOUGH_GRADIENT, 1, 300000, 100, 15, 60, 130);

        Point imageCenter(frame.cols / 2, frame.rows / 2);

        for (size_t i = 0; i < circles.size(); i++)
        {
            Point center;

            center.x = cvRound(circles[i][0]);
            center.y = cvRound(circles[i][1]);

            int radius = cvRound(circles[i][2]);
            circle(frame, center, radius, Scalar(0, 0, 255), 3);
            ROS_INFO("center.x=%d,center.y=%d", center.x, center.y);
            dx = double(center.x) - double(imageCenter.x);
            dy = double(center.y + 100) - double(imageCenter.y);
            ROS_INFO("dx=%f,dy=%f", dx, dy);
            ROS_INFO("radius=%d", radius);
        }

        double dxoutput = controller_green_circle_x.update(dx, dt);
        double dyoutput = controller_green_circle_y.update(dy, dt);
        dx += dxoutput;
        dy += dyoutput;
        if (dx > 300.0)
        {
            dx = 300.0;
        }
        if (dx < -300.0)
        {
            dx = -300.0;
        }
        if (dy > 300.0)
        {
            dy = 300.0;
        }
        if (dy < -300.0)
        {
            dy = -300.0;
        }
        circle_erro.erro_x = dx;
        circle_erro.erro_y = dy;
        circle_erro.flag = 1;
        detection_pub.publish(circle_erro);
        // cout << "green_circle_dx=" << dx << endl;
        // cout << "green_circle_dy=" << dy << endl;

        if (abs(dx) < 35 && abs(dy) < 35)
        {
            flag++;
            cout << "ok" << endl;
            if (flag > 10)
            {
                flag = 0;
                circle_erro.start_opencv = 1;
                cout << "green_circle is finish" << endl;
                // wait for sometimes and do somethings
            }
        }

        imshow("frame", frame);
        rate.sleep();
        if (waitKey(33) == 'q')
        {
            break;
        }
    }

    return 0;
}