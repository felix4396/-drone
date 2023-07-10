// 用于场外识别
// 需要调整参数：hsv阈值、开闭运算算子大小、二值图open滤除阈值、canny算子、轮廓大小阈值、颜色检测阈值
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <detection/detection.h>
#include <cmath>

using namespace cv;
using namespace std;

// red
int minH = 0, maxH = 180; // maxH=180最好
int minS = 100, maxS = 150;
int minV = 140, maxV = 190;
// blue
int minH_1 = 90, maxH_1 = 115;
int minS_1 = 30, maxS_1 = 130;
int minV_1 = 100, maxV_1 = 150;

int detection(Mat src);                                                  // 轮廓检测，输入是经hsv处理的二值图,输出1、2、3分别为三角形、矩形、圆形,输出0表示没找到
int detectColors(Mat color_image, vector<vector<int>> color_thresholds); // 检测特定颜色并返回颜色代码(0: 没有检测到, 1: 红色, 2: 蓝色)

Mat frame;
int start_opencv = 0;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "code_test_2");
    ros::NodeHandle nh;
    int shape_flag, color_flag;

    // frame = imread("./2022/red_10.jpg");
    VideoCapture cap(0);

    while (true)
    {
        // 获取图像
        cap >> frame;

        // 设置阈值集合、颜色检测
        vector<vector<int>> color_thresholds =
            {
                {minH, maxH, minS, maxS, minV, maxV},            // 红色阈值
                {minH_1, maxH_1, minS_1, maxS_1, minV_1, maxV_1} // 蓝色阈值
            };
        color_flag = detectColors(frame, color_thresholds);

        // hsv阈值化处理
        Mat hsv, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        switch (color_flag)
        {
        case 0:
            // 如果没检测到红蓝色。。。
            break;
        case 1:
            // 检测到红色
            inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
            break;
        case 2:
            // 检测到蓝色
            inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask);
            break;
        default:
            break;
        }

        // 形状检测
        if (color_flag)
        {
            imshow("Mask", mask);
            shape_flag = detection(mask);
            cout << "color_flag=" << color_flag << endl;
            cout << "shape_flag=" << shape_flag << endl;
        }
        imshow("src", frame);

        if (waitKey(10) == 'q')
        {
            break;
        }
    }

    // waitKey(0);
    return 0;
}

int detection(Mat src)
{
    int flag = 0;
    Mat structure_close = getStructuringElement(MORPH_RECT, Size(7, 7));
    Mat structure_open = getStructuringElement(MORPH_RECT, Size(3, 3));
    Mat close, open;

    // 闭运算，连接特征点
    morphologyEx(src, close, MORPH_CLOSE, structure_close);
    // morphologyEx(close, close, MORPH_CLOSE, structure_close);
    imshow("close", close);

    // 开运算，消除毛刺和小轮廓
    morphologyEx(close, open, MORPH_OPEN, structure_open);
    imshow("open", open);

    // 如果二值图面积小，输出0表示没检测到
    // cout << countNonZero(open) << endl;
    if (countNonZero(open) < 2200)
    {
        return 0;
    }

    // 进行边缘检测
    Mat edges;
    Canny(open, edges, 50, 150);
    imshow("edges", edges);

    // 进行轮廓检测
    vector<vector<Point>> contours;
    findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 遍历轮廓进行形状检测
    for (int i = 0; i < contours.size(); i++)
    {
        // 计算轮廓的周长
        double perimeter = arcLength(contours[i], true);

        // cout << "第" << i << "个轮廓的大小为：" << contourArea(contours[i]) << endl;
        if (contourArea(contours[i]) > 2200) // 小于2200的轮廓不进行判断
        {
            // 进行形状匹配
            vector<Point> approx;
            approxPolyDP(contours[i], approx, 0.04 * perimeter, true); // 输入、输出、拟合精度、是否闭合
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
        }
    }
    // imshow("frame", frame);

    return flag;
}

int detectColors(Mat color_image, vector<vector<int>> color_thresholds)
{
    int color_code = 0;
    vector<int> color_num;
    Mat hsv_image;
    cvtColor(color_image, hsv_image, COLOR_BGR2HSV);
    for (int i = 0; i < color_thresholds.size(); i++)
    {
        int h_min = color_thresholds[i][0];
        int h_max = color_thresholds[i][1];
        int s_min = color_thresholds[i][2];
        int s_max = color_thresholds[i][3];
        int v_min = color_thresholds[i][4];
        int v_max = color_thresholds[i][5];
        Mat mask;
        inRange(hsv_image, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), mask);
        color_num.push_back(countNonZero(mask));
    }
    if (color_num[0] > 2500 | color_num[1] > 2500)
    {
        if (color_num[0] > color_num[1])
        {
            color_code = 1;
        }
        else if (color_num[0] < color_num[1])
        {
            color_code = 2;
        }
    }
    return color_code;
}