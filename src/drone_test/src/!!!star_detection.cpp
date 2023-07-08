#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
 
using namespace std;

using namespace cv;
 
cv::Mat depth_pic;        //定义全局变量，图像矩阵Ｍat形式
float d_value;

void depthCallback(const sensor_msgs::Image::ConstPtr&msg)
{
    cv_bridge::CvImagePtr Dest ;
    Dest = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_16UC1);

    depth_pic = Dest->image;
    //cout<<"Output some info about the depth image in cv format"<<endl;
    // cout<< "Rows of the depth iamge = "<<depth_pic.rows<<endl;                       //获取深度图的行数height
    // cout<< "Cols of the depth iamge = "<<depth_pic.cols<<endl;                           //获取深度图的列数width
    // cout<< "Type of depth_pic's element = "<<depth_pic.type()<<endl;             //深度图的类型
    ushort d = depth_pic.at<ushort>(depth_pic.rows/2,depth_pic.cols/2);           //读取深度值，数据类型为ushort单位为ｍｍ
    imshow("depth_pic",depth_pic);
    d_value = float(d)/1000 ;      //强制转换
    cout<< "Value of depth_pic's pixel= "<<d_value<<endl;    //读取深度值
    waitKey(1);
    ROS_INFO("depthCallback");
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "stream_pub");               // ros节点初始化
    ros::NodeHandle nh;                                           //创建节点句柄
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image>("/d400/aligned_depth_to_color/image_raw",1,depthCallback);   //订阅深度图像
    //ros::Subscriber element_sub = nh.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw",100,pixelCallback);     //订阅像素点坐标
    // ros::Publisher mode_pub = nh.advertise<realsense_dev::depth_value>("/depth_info", 10);

    ros::Rate rate(20.0);    //设定自循环的频率

    while(ros::ok)
    {
        ros::spinOnce();
        rate.sleep();
    //     command_to_pub.header.stamp      = ros::Time::now();
    //     command_to_pub.Value = d_value;     //depth_pic.rows/2,depth_pic.cols/2  为像素点
    //     mode_pub.publish(command_to_pub);
    }
    
    ros::Duration(10).sleep();    //设定自循环的频率
    return 0 ;
}

