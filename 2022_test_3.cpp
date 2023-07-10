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

using namespace std;
using namespace cv;

Mat depth_image;
Mat color_image;
Mat mask_image;
vector<Vec4i> lines;
double target = 500; // 目标深度值
double dt = 1;       // PID控制器采样时间
Mat elemment = getStructuringElement(MORPH_RECT, Size(3, 3));

//黑色hsv值
int minH = 0, maxH = 10;
int minS = 50, maxS = 255;
int minV = 50, maxV = 255;

int cx = 0;
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
        cout << "output:" << output << endl;
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

// 获取相机深度图像
void depthCb(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); // 16位无符号整数单通道图像
        depth_image = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    waitKey(1);
    // imshow("depth", depth_image);
}

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

// 剔除端点横坐标值相差大于20的直线
vector<Vec4i> filterLines(vector<Vec4i> &lines)
{
    vector<Vec4i> filtered_lines;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        if (abs(l[0] - l[2]) <= 20)
        {
            filtered_lines.push_back(l);
        }
    }
    return filtered_lines;
}

// 拟合直线
void getLine(Mat &color_image, vector<Vec4i> &lines)
{
    vector<Point> points;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        points.push_back(Point(l[0], l[1]));
        points.push_back(Point(l[2], l[3]));
    }

    if (points.size() < 2)
    {
        return;
    }

    Vec4f line_params;
    fitLine(points, line_params, DIST_L2, 0, 0.01, 0.01);

    int x1 = 0;
    int y1 = int(line_params[3] - (line_params[2] / line_params[0]) * line_params[1]);
    int x2 = color_image.cols;
    int y2 = int(line_params[3] + (color_image.cols - line_params[1]) * (line_params[2] / line_params[0]));

    line(color_image, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 0), 2);
}

//图像预处理
vector<Vec4i> processImage()
{
    Mat hsv_image;
    vector<Vec4i> lines;
    cvtColor(color_image, hsv_image, COLOR_BGR2HSV);
    // imshow("hsv", hsv_image);

    Mat threshold_image;
    int h_min = minH;
    int h_max = maxH;
    int s_min = minS;
    int s_max = maxS;
    int v_min = minV;
    int v_max = maxV;

    inRange(hsv_image, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), threshold_image);
    imshow("threshold", threshold_image);
    morphologyEx(threshold_image, threshold_image, MORPH_CLOSE, elemment);
    imshow("close", threshold_image);
    morphologyEx(threshold_image, threshold_image, MORPH_OPEN, elemment);
    imshow("open", threshold_image);
    HoughLinesP(threshold_image, lines, 1, CV_PI / 180, 80, 200, 3);

    if (lines.size() > 0)
    {
        lines = filterLines(lines);
        for (size_t i = 0; i < lines.size(); i++)
        {
            Vec4i l = lines[i];
            line(color_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
        }
        // getLine(color_image, lines);
    }
    return lines;
}

// 获取特定颜色直线的平均深度值。
double getLineDepth(vector<Vec4i> lines)
{
    double sum_all = 0;
    // cout<<"1111111"<<endl;
    int line_size = lines.size();
    int middle_y = depth_image.rows / 2; // 获取图像中间水平线的y坐标
    for (size_t a = 0; a < lines.size(); a++)
    {
        double depth_sum = 0; // 深度值的总和
        int valid_count = 0;  // 有效深度值的数量
        Vec4i line = lines[a];
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        // ROS_INFO("x1=%d,y1=%d,x2=%d,y2=%d",x1,y1,x2,y2);
        double avg_depth = 0;
        // 判断直线是否跨越中间水平线
        double y1_y2 = (y1 - middle_y) * (y2 - middle_y);
        // ROS_INFO("y1_y2=%f",y1_y2);
        if (y1_y2 < 0)
        {
            // 计算直线长度和角度
            double line_length = sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
            double line_angle;
            if (x1 == x2)
            {
                line_angle = 3.1415926 / 2;
            }
            else
            {
                line_angle = atan2(y2 - y1, x2 - x1);
            }

            // 获取直线上的点的深度值
            for (double i = 0; i < line_length; i += 1) // 步长
            {
                int x = x1 + i * cos(line_angle);
                int y = y1 + i * sin(line_angle);

                // 判断当前点是否在中间水平线上
                if (abs(y - middle_y) < 1)
                {
                    // 从深度图像中获取深度值
                    int depth = depth_image.at<ushort>(y, x);
                    ROS_INFO("depth_%d:%d", a, depth);
                    // 如果深度值在有效范围内，则累加到depth_sum中
                    if (depth > 0 && depth < 5000)
                    {
                        depth_sum += depth;
                        valid_count++;
                    }
                }
            }
            // 计算平均深度
            avg_depth = valid_count > 0 ? depth_sum / valid_count : 0;

            // // 剔除异常值
            // const double outlier_threshold = 2; // 异常值阈值
            // double sum_of_squares = 0;          // 平方和
            // for (double i = 0; i < line_length; i += 1)
            // {
            //     int x = x1 + i * cos(line_angle);
            //     int y = y1 + i * sin(line_angle);
            //     // 判断当前点是否在中间水平线上
            //     if (y == middle_y)
            //     {
            //         int depth = depth_image.at<ushort>(y, x);
            //         if (depth > 0 && depth < 5000)
            //         {
            //             sum_of_squares += pow(depth - avg_depth, 2);
            //         }
            //     }
            // }
            // double std_dev = sqrt(sum_of_squares / valid_count);
            // for (double i = 0; i < line_length; i += 1)
            // {
            //     int x = x1 + i * cos(line_angle);
            //     int y = y1 + i * sin(line_angle);
            //     // 判断当前点是否在中间水平线上
            //     if (y == middle_y)
            //     {
            //         int depth = depth_image.at<ushort>(y, x);
            //         if (depth > 0 && depth < 5000)
            //         {
            //             if (fabs(depth - avg_depth) > outlier_threshold * std_dev)
            //             {
            //                 valid_count--;
            //                 depth_sum -= depth;
            //             }
            //         }
            //     }
            // }
            // }
            // // 更新平均深度
            // avg_depth = valid_count > 0 ? depth_sum / valid_count : 0;
            sum_all += avg_depth;
        }
        else
        {
            line_size = line_size - 1;
        }
    }
    ROS_INFO("sum_all=%lf", sum_all);
    ROS_INFO("lines.size()=%d", lines.size());
    ROS_INFO("line_size=%d", line_size);
    return sum_all / line_size;
}

PIDController pid_controller_dx;

int main(int argc, char **argv)
{
    cout << "start" << endl;
    ros::init(argc, argv, "line_detection");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    // image_transport::Subscriber depth_sub = it.subscribe("/d400/aligned_depth_to_color/image_raw", 2, depthCb);
    // image_transport::Subscriber color_sub = it.subscribe("/d400/color/image_raw", 2, colorCb);
    image_transport::Subscriber depth_sub = it.subscribe("/camera/aligned_depth_to_color/image_raw", 2, depthCb);
    image_transport::Subscriber color_sub = it.subscribe("/camera/color/image_raw", 2, colorCb);
    ros::Publisher detection_pub = nh.advertise<detection::detection>("detection", 1000);

    ros::Rate rate(30);
    detection::detection erro;
    // 初始化PID控制器
    pid_controller_dx.setParameters(0.1, 0.0, 0.01, -100.0, 100.0);

    namedWindow("Trackbars", WINDOW_NORMAL);
    resizeWindow("Trackbars", 640, 480);

    createTrackbar("Min H", "Trackbars", &minH, 255);
    createTrackbar("Max H", "Trackbars", &maxH, 255);
    createTrackbar("Min S", "Trackbars", &minS, 255);
    createTrackbar("Max S", "Trackbars", &maxS, 255);
    createTrackbar("Min V", "Trackbars", &minV, 255);
    createTrackbar("Max V", "Trackbars", &maxV, 255);

    while (ros::ok())
    {
        ros::spinOnce();
        if (color_image.size().empty() || depth_image.size().empty())
        {
            continue;
        }
        vector<Vec4i> lines = processImage();
        // 获取特定颜色直线的深度值并输入PID控制器
        if (lines.size())
        {
            double depth = getLineDepth(lines);
            //cout << "depth: " << depth << endl;
            double dx = pid_controller_dx.update(320, cx);
            double dy;
            //如果距离大于安全距离输出0，小于安全距离输出安全距离与距离的差值
            dy=max(0,target-depth);
            erro.erro_y = dy;
            erro.erro_x = dx;
            detection_pub.publish(erro);
        }
        imshow("color", color_image);
        imshow("depth", depth_image);
        if (waitKey(33) == 'q')
        {
            break;
        }
        rate.sleep();
    }
    return 0;
}