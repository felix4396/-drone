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
#include <zbar.h>

using namespace std;
using namespace cv;
using namespace zbar;

Mat depth_image;
Mat color_image;
Mat color_image_clone;
Mat mask_image;
vector<Vec4i> lines;
vector<int> threshold_color;
ros::Time last_time;
int frame_count = 0;
int flag = 1;
int barcode = 0;
int qrcode = 0;
double target = 500; // 目标深度值
double dt = 1.0;     // PID控制器采样时间
Mat elemment = getStructuringElement(MORPH_RECT, Size(3, 3));
int code_num = 0;
double last_depth = 0.0;
double last_depth_1 = 0.0;
double last_depth_2 = 0.0;

int minH = 0, maxH = 70;
int minS = 81, maxS = 255;
int minV = 133, maxV = 255;

// int minH_1 = 53, maxH_1 = 86;
// int minS_1 = 42, maxS_1 = 200;
// int minV_1 = 60, maxV_1 = 210;

int minH_1 = 53, maxH_1 = 93;
int minS_1 = 42, maxS_1 = 200;
int minV_1 = 88, maxV_1 = 210;
// int minH_1 = 53, maxH_1 = 93;
// int minS_1 = 42, maxS_1 = 200;
// int minV_1 = 88, maxV_1 = 255;

int minH_2 = 44, maxH_2 = 97;
int minS_2 = 36, maxS_2 = 255;
int minV_2 = 47, maxV_2 = 255;
int cx = 0;

Point center_fitline;
int start_opencv = 0;
int start_opencv_last = 0;
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
        double output = Kp * error + Ki * integral - Kd * derivative;
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
vector<Vec4i> getLine(Mat &color_image, vector<Vec4i> &lines)
{
    vector<Point> points;
    for (size_t i = 0; i < lines.size(); i++)
    {
        Vec4i l = lines[i];
        points.push_back(Point(l[0], l[1]));
        points.push_back(Point(l[2], l[3]));
    }
    Vec4f line_params;
    vector<Vec4i> lines_fitline;
    try
    {
        fitLine(points, line_params, DIST_L2, 0, 0.01, 0.01);
        double k = line_params[1] / line_params[0];

        int x1 = 0;
        int y1 = k * (x1 - line_params[2]) + line_params[3];
        int x2 = color_image.cols;
        int y2 = k * (x2 - line_params[2]) + line_params[3];
        center_fitline.x = (x1 + x2) / 2;
        center_fitline.y = (y1 + y2) / 2;
        cx = line_params[2];
        lines_fitline.push_back(Vec4i(x1, y1, x2, y2));
        line(color_image_clone, Point(x1, y1), Point(x2, y2), Scalar(255, 0, 0), 2);
        if (lines_fitline.size())
        {
            return lines_fitline;
        }
        else
        {
            return lines;
        }
    }
    catch (const std::exception &e)
    {
        cout << e.what() << endl;
        return lines;
    }
}

vector<Vec4i> processImage(vector<int> color_threshold)
{
    Mat hsv_image;
    vector<Vec4i> Lines;
    cvtColor(color_image, hsv_image, COLOR_BGR2HSV);
    // imshow("hsv", hsv_image);

    Mat threshold_image;
    int h_min = color_threshold[0];
    int h_max = color_threshold[1];
    int s_min = color_threshold[2];
    int s_max = color_threshold[3];
    int v_min = color_threshold[4];
    int v_max = color_threshold[5];

    inRange(hsv_image, Scalar(h_min, s_min, v_min), Scalar(h_max, s_max, v_max), threshold_image);
    // imshow("threshold", threshold_image);
    dilate(threshold_image, threshold_image, elemment);
    erode(threshold_image, threshold_image, elemment);
    dilate(threshold_image, threshold_image, elemment);
    imshow("threshold_image", threshold_image);
    HoughLinesP(threshold_image, Lines, 1, CV_PI / 180, 80, 100, 10);

    if (Lines.size() > 0)
    {
        Lines = filterLines(Lines);
        for (size_t i = 0; i < Lines.size(); i++)
        {
            Vec4i l = Lines[i];
            line(color_image_clone, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 4);
        }
        getLine(color_image, Lines);
    }
    return Lines;
}

// 获取特定颜色直线的平均深度值。
double getLineDepth(vector<Vec4i> lines)
{
    double sum_all = 0;
    int line_size = lines.size();
    int middle_y = depth_image.rows / 2; // 获取图像中间水平线的y坐标
    for (size_t a = 0; a < lines.size(); a++)
    {
        int depth_sum = 0;   // 深度值的总和
        int valid_count = 0; // 有效深度值的数量
        Vec4i line = lines[a];
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double avg_depth = 0;
        // 判断直线是否跨越中间水平线
        double y1_y2 = (y1 - middle_y) * (y2 - middle_y);
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
                if (abs(y - middle_y) < 5)
                {
                    // 从深度图像中获取深度值
                    int depth = depth_image.at<ushort>(y, x);
                    // 如果深度值在有效范围内，则累加到depth_sum中
                    if (depth > 0 && depth < 3000)
                    {
                        depth_sum = depth_sum + depth;
                        valid_count = valid_count + 1;
                    }
                }
            }

            // 计算平均深度
            // ROS_INFO("valid_count=%d", valid_count);
            if (valid_count != 0)
            {
                avg_depth = double(depth_sum / valid_count);
            }
            else
            {
                avg_depth = 0.0;
            }

            if (!avg_depth)
            {
                line_size = line_size - 1;
            }
            else
            {
                sum_all += avg_depth;
            }
        }
        else
        {
            line_size = line_size - 1;
        }
    }
    // ROS_INFO("sum_all=%lf", sum_all);
    // ROS_INFO("lines.size()=%d", lines.size());
    ROS_INFO("line_size=%d", line_size);
    if (line_size == 0)
    {
        // double depth_return = double(depth_image.at<ushort>(int(center_fitline.y), int(center_fitline.x)));
        return 0;
    }
    else
    {
        return sum_all / line_size;
    }
}

// 检测特定颜色并返回颜色代码(0: 没有检测到, 1: 红色, 2: 绿色)
int detectColors(Mat color_image, vector<vector<int>> color_thresholds)
{
    int color_code;
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
    else
    {
        color_code = 0;
    }
    return color_code;
}

void doOpencv_sub(const drone_test::detection::ConstPtr &msg)
{
    start_opencv = msg->start_opencv;
}

int detectColorWithSmallerLineDepth(vector<vector<int>> color_thresholds)
{
    // 进行直线检测
    vector<vector<Vec4i>> detectedLines(2);

    detectedLines[0] = processImage(color_thresholds[0]);

    detectedLines[1] = processImage(color_thresholds[1]);
    // 计算直线的平均深度值
    double depth1, depth2;
    // ROS_INFO("detectedLines[0].size()=%d", detectedLines[0].size());
    // ROS_INFO("detectedLines[1].size()=%d", detectedLines[1].size());
    if (detectedLines[0].size())
    {
        depth1 = getLineDepth(detectedLines[0]);
    }
    else
    {
        depth1 = 3000;
    }
    if (detectedLines[1].size())
    {
        depth2 = getLineDepth(detectedLines[1]);
    }
    else
    {
        depth2 = 2999;
    }
    if (depth1 == 0)
    {
        detectedLines[0] = filterLines(detectedLines[0]);
        detectedLines[0] = getLine(color_image, detectedLines[0]);
        depth1 = getLineDepth(detectedLines[0]);
    }
    if (depth2 == 0)
    {
        detectedLines[1] = filterLines(detectedLines[1]);
        detectedLines[1] = getLine(color_image, detectedLines[1]);
        depth2 = getLineDepth(detectedLines[1]);
    }
    // depth1 = getLineDepth(detectedLines[0]);
    // depth2 = getLineDepth(detectedLines[1]);
    double dis = depth1 - last_depth_1;
    // 根据最大速度修改，两帧之间大于500会卡住，且没在转换杆
    if (abs(dis) > 500 & last_depth_1 != 0 & start_opencv_last == start_opencv)
    {
        depth1 = last_depth_1;
    }
    else
    {
        if (start_opencv_last != start_opencv & abs(dis) > 500)
        {
            start_opencv_last = start_opencv;
        }
        last_depth_1 = depth1;
    }
    dis = depth2 - last_depth_2;
    if (abs(dis) > 500 & last_depth_2 != 0 & start_opencv_last == start_opencv)
    {
        depth2 = last_depth_2;
    }
    else
    {
        if (start_opencv_last != start_opencv & abs(dis) > 500)
        {
            start_opencv_last = start_opencv;
        }
        last_depth_2 = depth2;
    }

    ROS_INFO("depth1=%lf", depth1);
    ROS_INFO("depth2=%lf", depth2);
    // 返回深度值较小的直线颜色代码（1: 红色, 2: 绿色)
    return (depth1 < depth2) ? 1 : 2;
}

// int codeprocess(Mat color_image)
// {
//     Mat image;
//     int code = 0;
//     color_image.copyTo(image);
//     // Convert to grayscale
//     cvtColor(image, image, CV_BGR2GRAY);
//     equalizeHist(image, image);
//     int width = image.cols;
//     int height = image.rows;
//     uchar *raw = (uchar *)image.data;

//     Image imageZbar(width, height, "Y800", raw, width * height);
//     scanner.scan(imageZbar); // 扫描条码
//     Image::SymbolIterator symbol = imageZbar.symbol_begin();
//     if (imageZbar.symbol_begin() == imageZbar.symbol_end())
//     {
//         ROS_INFO("zbar_error");
//         return -1;
//     }
//     else
//     {
//         string code_str;
//         for (; symbol != imageZbar.symbol_end(); ++symbol)
//         {
//             ROS_INFO("zbar_type:%s", symbol->get_type_name().c_str());
//             ROS_INFO("zbar_code:%s", symbol->get_data().c_str());
//             code_str = symbol->get_data();
//             code = code_str[0] - '0';
//             if (code < 0 || code > 9)
//             {
//                 code = 0;
//             }
//         }
//         barcode = 1;
//     }
//     return code;
// }

PIDController pid_controller_depth;
PIDController pid_controller_dx;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    double yaw = tf::getYaw(msg->pose.pose.orientation) * 180.0 / 3.1415926;
    // ROS_INFO("Yaw angle: %f", yaw);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_detection");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber depth_sub = it.subscribe("/d400/aligned_depth_to_color/image_raw", 1, depthCb);
    image_transport::Subscriber color_sub = it.subscribe("/d400/color/image_raw", 1, colorCb);
    ros::Publisher detection_pub = nh.advertise<drone_test::detection>("detection", 1000);
    ros::Subscriber Opencv_sub = nh.subscribe<drone_test::detection>("detection_2", 10, doOpencv_sub);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/camera/odom/sample", 1, odomCallback);
    ros::Publisher code_pub = nh.advertise<drone_test::code>("detection_code", 1000);

    ros::Rate rate(20);
    drone_test::detection erro;
    drone_test::code code;
    // 初始化PID控制器
    pid_controller_dx.setParameters(1.0, 0.0, 0.2, -200.0, 200.0);
    pid_controller_depth.setParameters(1.0, 0.0, 0.2, -300.0, 300.0);
    int color_code;

    // ImageScanner scanner;
    // scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    // namedWindow("Trackbars", WINDOW_NORMAL);
    // resizeWindow("Trackbars", 640, 480);
    // createTrackbar("Min H", "Trackbars", &minH, 255);
    // createTrackbar("Max H", "Trackbars", &maxH, 255);
    // createTrackbar("Min S", "Trackbars", &minS, 255);
    // createTrackbar("Max S", "Trackbars", &maxS, 255);
    // createTrackbar("Min V", "Trackbars", &minV, 255);
    // createTrackbar("Max V", "Trackbars", &maxV, 255);
    // createTrackbar("Min H_1", "Trackbars", &minH_1, 255);
    // createTrackbar("Max H_1", "Trackbars", &maxH_1, 255);
    // createTrackbar("Min S_1", "Trackbars", &minS_1, 255);
    // createTrackbar("Max S_1", "Trackbars", &maxS_1, 255);
    // createTrackbar("Min V_1", "Trackbars", &minV_1, 255);
    // createTrackbar("Max V_1", "Trackbars", &maxV_1, 255);

    cout << "Waiting...";
    do
    {
        rate.sleep();
        ros::spinOnce();
    } while (start_opencv != 1 && ros::ok);
    start_opencv_last = start_opencv;

    while (ros::ok())
    {
        ros::spinOnce();
        if (color_image.size().empty() || depth_image.size().empty())
        {
            continue;
        }
        color_image.copyTo(color_image_clone);
        // 输出检测颜色代码
        vector<vector<int>> color_thresholds =
            {
                {minH, maxH, minV, maxV, minS, maxS},            // 红色阈值
                {minH_1, maxH_1, minV_1, maxV_1, minS_1, maxS_1} // 绿色阈值
                                                                 //{minH_2, maxH_2, minV_2, maxV_2, minS_2, maxS_2}
            };
        // int color_code = 1;
        ROS_INFO("start_opencv: %d", start_opencv);

        if (start_opencv == 1)
        {
            color_code = detectColorWithSmallerLineDepth(color_thresholds); // 检测最近的线
            ROS_INFO("Color codes: %d", color_code);
            ROS_INFO("111111111111111");
            // 进行霍夫直线检测
            if (color_code == 1)
            {
                threshold_color = color_thresholds[0];
            }
            else if (color_code == 2)
            {
                threshold_color = color_thresholds[1];
            }
        }
        else if (start_opencv == 2) // 飞完第一圈发2
        {
            ROS_INFO("2222222222222");
            if (color_code == 1)
            {
                threshold_color = color_thresholds[1];
            }
            else if (color_code == 2)
            {
                threshold_color = color_thresholds[0];
            }
        }
        if (color_code)
        {
            lines = processImage(threshold_color);
        }
        // ROS_INFO("lines.size()=%d", lines.size());
        //  获取特定颜色直线的深度值并输入PID控制器
        if (lines.size())
        {

            double depth = getLineDepth(lines);
            if (depth == 0)
            {
                lines = filterLines(lines);
                lines = getLine(color_image, lines);
                depth = getLineDepth(lines);
            }
            if (start_opencv == 2 & flag < 2)
            {
                last_depth = depth;
                flag++;
            }
            double dis = depth - last_depth;
            if (abs(dis) > 500 & last_depth != 0 & start_opencv_last == start_opencv) // 如果深度变化过大，认为是误检,并且增加转换杆的条件
            {
                depth = last_depth;
            }
            else
            {
                if (start_opencv_last != start_opencv & abs(dis) > 500)
                {
                    start_opencv_last = start_opencv;
                }
                last_depth = depth;
            }
            cout << "depth: " << depth << endl;
            double distance = pid_controller_depth.update(target, depth);
            double dx = pid_controller_dx.update(320, cx);
            ROS_INFO("pid: %lf", distance);
            erro.erro_x = dx;
            erro.erro_y = -distance;
            if (start_opencv == 1)
                erro.flag = color_code;
            if (start_opencv == 2)
            {
                if (color_code == 1)
                    erro.flag = 2;
                else if (color_code == 2)
                    erro.flag = 1;
            }
            detection_pub.publish(erro);
        }
        else
        {
            erro.erro_x = 0;
            erro.erro_y = 0;
            erro.flag = 0;
            detection_pub.publish(erro);
        }
        imshow("color", color_image_clone);

        // 计算帧率
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        frame_count++;
        if (dt > 1.0)
        {
            ROS_INFO("fps = %lf", frame_count / dt);
            frame_count = 0;
            last_time = current_time;
        }
        // imshow("depth", depth_image);
        if (waitKey(33) == 'q')
        {
            break;
        }
        rate.sleep();
    }
    return 0;
}