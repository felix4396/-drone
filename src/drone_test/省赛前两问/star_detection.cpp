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
#include <cmath>

using namespace cv;
using namespace std;

// red
int minH = 160, maxH = 190; // maxH=180最好
int minS = 147, maxS = 255;
int minV = 142, maxV = 210;
// blue
int minH_1 = 100, maxH_1 = 130;
int minS_1 = 90, maxS_1 = 190;
int minV_1 = 94, maxV_1 = 180;
// black
int minH_2 = 0, maxH_2 = 180;
int minS_2 = 0, maxS_2 = 80;
int minV_2 = 40, maxV_2 = 92;

int shape_detection(Mat src, int shape);                            // 圆轮廓检测，输入是经hsv处理的二值图、需要检测的图形,输出1、2、3分别为三角形、矩形、圆形,输出0表示没找到
vector<int> trans_flag(int flag_num);                               // 将地图上的数值转换成颜色、形状特征
Mat get_color_mask(Mat src, int color);                             // 根据颜色输出对应的hsv二值图
void doOpencv_sub(const drone_test::detection::ConstPtr &msg);      // 回调函数
void doOpencv_sub_flag(const drone_test::detection::ConstPtr &msg); // 回调函数
void back_position(Mat src);                                        // 返航点检测

Mat frame;
int f1 = 0;
int f2 = 0;                 // 指定位置编号
int color_flag, shape_flag; // 颜色、形状标志位，
int shape_code = 0;         // 形状检测返回值
double dt = 1.0;            // PID控制器采样时间
int start_opencv = 0;       // 检测标志位
double dx = 0.0;
double dy = 0.0; // 机器人需要移动的距离的pid控制量

// PID控制器类
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

// 定义x、y方向的pid控制器
PIDController pid_controller_dx;
PIDController pid_controller_dy;
int main(int argc, char **argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "code_test");
    ros::NodeHandle nh;

    // 创建一个Publisher，发布名为detection的topic，消息类型为detection::detection，队列长度10
    ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection", 1000);
    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_2", 10, doOpencv_sub);
    ros::Subscriber Opencv_sub_flag = nh.subscribe<drone_test::detection>("detection_3", 10, doOpencv_sub_flag);

    // 设置循环的频率
    ros::Rate rate(20);
    drone_test::detection erro;

    // frame = imread("./2022/red_10.jpg");
    VideoCapture cap(200);

    // 设置pid控制器参数，分别为p、i、d、最小输出值、最大输出值
    pid_controller_dx.setParameters(1.5, 0.0, 0.1, -100.0, 100.0);
    pid_controller_dy.setParameters(1.5, 0.0, 0.1, -100.0, 100.0);

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
    // createTrackbar("Min H", "Trackbars", &minH_1, 255);
    // createTrackbar("Max H", "Trackbars", &maxH_1, 255);
    // createTrackbar("Min S", "Trackbars", &minS_1, 255);
    // createTrackbar("Max S", "Trackbars", &maxS_1, 255);
    // createTrackbar("Min V", "Trackbars", &minV_1, 255);
    // createTrackbar("Max V", "Trackbars", &maxV_1, 255);

    while (ros::ok())
    {
        ros::spinOnce();
        // 读取视频帧
        cap >> frame;

        // 模式1，检测图像中的颜色、形状特征输出偏差
        if (start_opencv == 1)
        {
            // 获取颜色、形状特征
            vector<int> flag_1;
            flag_1 = trans_flag(f1);
            color_flag = flag_1[0];
            shape_flag = flag_1[1];
            Mat mask;
            mask = get_color_mask(frame, color_flag);
            // imshow("Mask", mask);
            shape_code = shape_detection(mask, shape_flag);
            ROS_INFO("shape_code=%d", shape_code);
            // cout << "flag=" << shape_code << endl;
        }

        // 模式2，检测图像中的颜色、形状特征输出偏差
        if (start_opencv == 2)
        {
            // 获取颜色、形状特征
            vector<int> flag_2;
            flag_2 = trans_flag(f2);
            color_flag = flag_2[0];
            shape_flag = flag_2[1];

            Mat mask;
            mask = get_color_mask(frame, color_flag);
            // imshow("Mask", mask);

            shape_code = shape_detection(mask, shape_flag);
            ROS_INFO("shape_code=%d", shape_code);
            // cout << "flag=" << shape_code << endl;
        }

        // 模式3，输出返航点的偏差
        if (start_opencv == 3)
        {
            back_position(frame);
            ROS_INFO("back");
        }

        // 获取图像尺寸
        int imageWidth = frame.cols;
        int imageHeight = frame.rows;

        // 计算十字的中心位置
        int centerX = imageWidth / 2;
        int centerY = imageHeight / 2;

        // 绘制十字
        cv::line(frame, cv::Point(centerX, 0), cv::Point(centerX, imageHeight), cv::Scalar(0, 0, 255), 2);
        cv::line(frame, cv::Point(0, centerY), cv::Point(imageWidth, centerY), cv::Scalar(0, 0, 255), 2);
        imshow("src", frame);
        erro.erro_x = dx;
        erro.erro_y = dy;
        erro.flag = shape_code;
        detection_pub.publish(erro);

        if (waitKey(10) == 'q')
        {
            break;
        }
        rate.sleep();
    }
    // waitKey(0);
    return 0;
}

int shape_detection(Mat src, int shape)
{
    int flag = 0;
    Mat structure_close = getStructuringElement(MORPH_RECT, Size(7, 7));
    Mat structure_open = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat close, open;

    // 闭运算，连接特征点
    morphologyEx(src, close, MORPH_CLOSE, structure_close);
    // morphologyEx(close, close, MORPH_CLOSE, structure_close);
    // imshow("close", close);

    // 开运算，消除毛刺和小轮廓
    morphologyEx(close, open, MORPH_OPEN, structure_open);
    // imshow("open", open);

    // cout << countNonZero(open) << endl;
    // 如果二值图面积小，输出0表示没检测到
    if (countNonZero(open) < 300)
    {
        return 0;
    }

    // 进行边缘检测?
    Mat edges;
    Canny(open, edges, 50, 150);
    // imshow("edges", edges);

    // 进行轮廓检测
    vector<vector<Point>> contours;
    findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 遍历轮廓进行形状检测
    for (int i = 0; i < contours.size(); i++)
    {
        // 计算轮廓的周长
        double perimeter = arcLength(contours[i], true);

        // cout << "第" << i << "个轮廓的大小为：" << contourArea(contours[i]) << endl;
        if (contourArea(contours[i]) > 300) // 小于2200的轮廓不进行判断
        {
            // 进行形状匹配
            vector<Point> approx;
            approxPolyDP(contours[i], approx, 0.04 * perimeter, true); // 输入、输出、拟合精度、是否闭合?=
            // cout << "approx.size=" << approx.size() << endl;

            // 判断形状类型
            if (approx.size() == 3)
            {
                // 三角形
                flag = 1;
                drawContours(frame, contours, i, Scalar(0, 0, 255), 2);
            }
            else if (approx.size() == 4)
            {
                // 正方形或矩形
                flag = 2;
                drawContours(frame, contours, i, Scalar(0, 0, 255), 2);
            }
            else if (approx.size() > 5)
            {
                // 圆形
                flag = 3;
                drawContours(frame, contours, i, Scalar(0, 0, 255), 2);
            }
            if (flag == shape)
            {
                // 求解轮廓中心点?
                Moments moments = cv::moments(contours[i]);
                double cx = moments.m10 / moments.m00; // X坐标的中心
                double cy = moments.m01 / moments.m00; // Y坐标的中心
                dx = pid_controller_dx.update(frame.cols * 0.5, cx);
                dy = pid_controller_dy.update(frame.rows * 0.5, cy);
                // cout << "dx=" << dx << endl;
                // cout << "dy=" << dy << endl;
                ROS_INFO("dx=%f", dx);
                ROS_INFO("dy=%f", dy);
                cv::circle(frame, cv::Point(cx, cy), 3, cv::Scalar(0, 255, 0), cv::FILLED);
            }
        }
    }
    return flag;
}

// 第一个值（0为红色、1为蓝色）第二个值（1、2、3分别为三角形、矩形、圆形）
vector<int> trans_flag(int flag_num)
{
    vector<int> flag;
    switch (flag_num)
    {
    case 1:
        flag.push_back(0);
        flag.push_back(1);
        break;
    case 2:
        flag.push_back(0);
        flag.push_back(1);
        break;
    case 3:
        flag.push_back(1);
        flag.push_back(1);
        break;
    case 4:
        flag.push_back(1);
        flag.push_back(1);
        break;
    case 5:
        flag.push_back(0);
        flag.push_back(3);
        break;
    case 6:
        flag.push_back(0);
        flag.push_back(3);
        break;
    case 7:
        flag.push_back(1);
        flag.push_back(3);
        break;
    case 8:
        flag.push_back(1);
        flag.push_back(3);
        break;
    case 9:
        flag.push_back(0);
        flag.push_back(2);
        break;
    case 10:
        flag.push_back(0);
        flag.push_back(2);
        break;
    case 11:
        flag.push_back(1);
        flag.push_back(2);
        break;
    case 12:
        flag.push_back(1);
        flag.push_back(2);
        break;
    default:
        break;
    }
    return flag;
}

Mat get_color_mask(Mat src, int color)
{
    // 转hsv色域、二值化
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    Mat mask;
    switch (color)
    {
    case 0:
        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
        break;
    case 1:
        inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask);
        break;
    default:
        break;
    }
    return mask;
}

void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    start_opencv = int(msg->start_opencv);
    // ROS_INFO("start_opencv=%d", start_opencv);
}

void doOpencv_sub_flag(const drone_test::detection::ConstPtr &msg)
{

    f1 = msg->flag;
    f2 = msg->start_opencv;
}

void back_position(Mat src)
{
    Mat hsv;
    cvtColor(src, hsv, COLOR_BGR2HSV);
    Mat mask, mask_1;
    inRange(hsv, Scalar(minH_2, minS_2, minV_2), Scalar(maxH_2, maxS_2, maxV_2), mask);
    // imshow("Mask", mask);

    Mat close, structure_close;
    structure_close = getStructuringElement(MORPH_RECT, Size(7, 7));
    morphologyEx(mask, close, MORPH_CLOSE, structure_close);
    // imshow("close", close);

    Mat open, structure_open;
    structure_open = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(close, open, MORPH_OPEN, structure_open);
    // imshow("open", open);

    Mat edge;
    Canny(open, edge, 100, 200);
    // imshow("edge", edge);

    // vector<Vec3f> circles;
    // HoughCircles(edge, circles, HOUGH_GRADIENT, 1, 3000, 100, 25, 20, 60); // 75,120

    // 进行轮廓检测
    vector<vector<Point>> contours;
    findContours(edge, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // cout << circles.size() << endl;

    for (size_t i = 0; i < contours.size(); i++)
    {
        // 计算轮廓的周长
        double perimeter = arcLength(contours[i], true);
        ROS_INFO("contours=%f", contourArea(contours[i]));
        if (contourArea(contours[i]) > 5000)
        {
            // 进行形状匹配
            vector<Point> approx;
            approxPolyDP(contours[i], approx, 0.04 * perimeter, true);
            ROS_INFO("approx.size()=%d", approx.size());
            if (approx.size() > 4)
            {
                // 画出轮廓
                drawContours(src, contours, i, Scalar(0, 0, 255), 2);
                Moments moments = cv::moments(contours[i]);
                double cx = moments.m10 / moments.m00; // X坐标的中心
                double cy = moments.m01 / moments.m00; // Y坐标的中心
                dx = pid_controller_dx.update(src.cols * 0.5, cx);
                dy = pid_controller_dy.update(src.rows * 0.5, cy);
                // cout << "dx=" << dx << endl;
                // cout << "dy=" << dy << endl;
                ROS_INFO("dx=%f", dx);
                ROS_INFO("dy=%f", dy);
                cv::circle(src, cv::Point(cx, cy), 3, cv::Scalar(0, 255, 0), cv::FILLED);
            }
        }
    }
}