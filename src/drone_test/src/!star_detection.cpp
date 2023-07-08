#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <drone_test/detection.h>

using namespace cv;
using namespace std;

int minH = 57, maxH = 147;
int minS = 0, maxS = 180;
int minV = 0, maxV = 147;

int minH_1 = 48, maxH_1 = 60;
int minS_1 = 61, maxS_1 = 132;
int minV_1 = 146, maxV_1 = 180;

int flag_circle = 0;
int flag_line = 1;
int flag_num = 0;
int flag_frame=0;

double now = 0.0;
Mat elemment_l = getStructuringElement(MORPH_RECT, Size(3, 3));
Mat elemment_h = getStructuringElement(MORPH_RECT, Size(7, 7));

int start_opencv=0;

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

void doOpencv_sub(const drone_test::detection::ConstPtr &msg){
    start_opencv=msg->start_opencv;
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
    Mat hsv, mask, mask_1,mask_green;
    Mat edge;
    VideoCapture cap(200);

    vector<Vec4f> linesP;
    //vector<Vec3f> circles;

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
    PIDController controller_line(3, 0.0, 0.1);
    PIDController controller_circle_x(3, 0.0, 0.1);
    PIDController controller_circle_y(3, 0.0, 0.1);
    PIDController controller_green_circle_x(3, 0.0, 0.1);
    PIDController controller_green_circle_y(3, 0.0, 0.1);

    double dt = 1.0;
    double distance = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    int flag = 0;
    

    do{
        rate.sleep();
        ros::spinOnce();
    }while(start_opencv!=1&&ros::ok);

    while (ros::ok)
    {
        vector<Vec3f> circles;
        cap >> frame;
        //frame = frame(Range(0, 480), Range(180, 460));
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
        if (flag_line == 1)
        {
            HoughLinesP(mask, linesP, 1.0f, CV_PI / 180.0f, 30, 100, 10);
            if(linesP.size()>0){
                //cout << linesP.size() << endl;
                double ave_line_x_0=0,ave_line_y_0=0,ave_line_x_1=0,ave_line_y_1=0;
                int cnt_line_all=(int)(linesP.size());
                int max_vaule_line=linesP[0][0],min_vaule_line=linesP[0][0];
                for (size_t i = 0; i < linesP.size(); i++)
                {
                    Vec4i l = linesP[i];
                    if(abs(l[0]-l[2])>20){
                        --cnt_line_all;
                        continue;
                    }
                    if(l[0]>max_vaule_line)
                        max_vaule_line=l[0];
                    if(l[2]>max_vaule_line)
                        max_vaule_line=l[2];
                    if(l[0]<min_vaule_line)
                        min_vaule_line=l[0];
                    if(l[2]<min_vaule_line)
                        min_vaule_line=l[2];
                   // ROS_INFO("l[0]=%d,l[2]=%d",l[0],l[2]);
                }

                //ROS_INFO("min_vaule_line=%d,max_vaule_line=%d",min_vaule_line,max_vaule_line);

                cnt_line_all=(int)(linesP.size());
                for (size_t i = 0; i < linesP.size(); i++)
                {
                    Vec4i l = linesP[i];
                    if(abs(l[0]-l[2])>20||abs(l[0]-min_vaule_line)>100||abs(l[2]-min_vaule_line)>100){
                        --cnt_line_all;
                        continue;
                    }
                    ave_line_x_0+=l[0];
                    ave_line_y_0+=l[1];
                    ave_line_x_1+=l[2];
                    ave_line_y_1+=l[3];
                    // ROS_INFO("l[0]=%d,l[2]=%d",l[2]-l[0],l[3]-l[1]);
                    //ROS_INFO("l[0]=%d,l[1]=%d",l[0]-int(frame.cols / 2),l[2]-int(frame.cols / 2));
                    line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 2);
                }
                

                if(cnt_line_all!=0){
                    ave_line_x_0=(int)(ave_line_x_0/cnt_line_all);
                    ave_line_y_0=(int)(ave_line_y_0/cnt_line_all);
                    ave_line_x_1=(int)(ave_line_x_1/cnt_line_all);
                    ave_line_y_1=(int)(ave_line_y_1/cnt_line_all);
                    ROS_INFO("x_0=%d,y_0=%d,x_1=%d,y_1=%d",(int)ave_line_x_0,(int)ave_line_y_0,(int)ave_line_x_1,(int)ave_line_y_1);  
                    int cx = (ave_line_x_0 + ave_line_x_1) / 2;
                    distance = double(cx) - double(frame.cols / 2);
                }
                line(frame, Point(ave_line_x_0, ave_line_y_0), Point(ave_line_x_1, ave_line_y_1), Scalar(255, 0, 0), 2);
            }else{
                distance=0;
            }

        }

        if (flag_circle == 0)
        {
            inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask_1);
        }
        if (flag_circle == 1)
        {
            inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask_1);
        }
        inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask_green);
        Canny(mask_1, edge, 100, 200);
        imshow("mask_green", mask_green);
        if(!flag_frame)
        {
            edge=edge(Range(0,260),Range(70,570));
        }
        
        imshow("edge", edge);
        HoughCircles(edge, circles, HOUGH_GRADIENT, 1, 300000, 100, 15, 60,130);
        
        Point imageCenter(frame.cols / 2, frame.rows / 2);
        ros::Time right_now = ros::Time::now();
        if (flag_circle == 1 && countNonZero(mask_green)>10)
        {
            flag_num = 1;
            //ROS_INFO("yyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy");
        }
        else
        {
            flag_num = 0;
            //ROS_INFO("NX");
        }

        if (!flag_num)
        {
            for (size_t i = 0; i < circles.size(); i++)
            {
                Point center;
                if(flag_frame)
                {
                    center.x=cvRound(circles[i][0]);
                    center.y=cvRound(circles[i][1]);
                }
                else
                {
                    center.x=cvRound(circles[i][0]+70);
                    center.y=cvRound(circles[i][1]);
                }
                int radius = cvRound(circles[i][2]);
                circle(frame, center, radius, Scalar(0, 0, 255),3);
                ROS_INFO("center.x=%d,center.y=%d",center.x,center.y);
                dx = double(center.x) - double(imageCenter.x);
                dy = double(center.y+100) - double(imageCenter.y);
                ROS_INFO("dx=%f,dy=%f",dx,dy);
                ROS_INFO("radius=%d",radius);
                if(radius<120&&radius>75){
                    flag_line = 0;
                    flag_frame=1;
                    //ROS_INFO("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
                }
            }
        }

    
        if (flag_line == 1)
        {
            // output pid of line
            double distanceOutput = controller_line.update(distance, dt);
            distance += distanceOutput;
            if (distance > 300.0)
            {
                distance = 300.0;
            }
            if (distance < -300.0)
            {
                distance = -300.0;
            }
            cout << "distance=" << distance << endl;
            line_erro.erro_x = distance;
            line_erro.erro_y = 0;
            line_erro.flag = 0;
            detection_pub.publish(line_erro);
        }
        
        if (flag_line == 0)
        {
            if (flag_circle == 0)
            {
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
                        flag_line = 1;
                        flag_circle = 1;
                        now = right_now.toSec();
                        flag_frame=0;
                        cout << "green_circle is finish" << endl;
                        // wait for sometimes and do somethings
                    }
                }
            }

            if (flag_circle == 1)
            {
                double dxoutput = controller_circle_x.update(dx, dt);
                double dyoutput = controller_circle_y.update(dy, dt);
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
                circle_erro.flag = 2;
                detection_pub.publish(circle_erro);
               // cout << "circle_dx=" << dx << endl;
               // cout << "circle_dy=" << dy << endl;
                if (abs(dx) < 35 && abs(dy) < 35)
                {
                    flag++;
                    cout << "ok" << endl;
                    if (flag > 10)
                    {
                        circle_erro.erro_x = 0;
                        circle_erro.erro_y = 0;
                        circle_erro.flag = -5;
                        detection_pub.publish(circle_erro);
                        circle_erro.erro_x = 0;
                        circle_erro.erro_y = 0;
                        circle_erro.flag = -5;
                        detection_pub.publish(circle_erro);
                        circle_erro.erro_x = 0;
                        circle_erro.erro_y = 0;
                        circle_erro.flag = -5;
                        detection_pub.publish(circle_erro);
                        circle_erro.erro_x = 0;
                        circle_erro.erro_y = 0;
                        circle_erro.flag = -5;
                        detection_pub.publish(circle_erro);
                        cout << "circle is finish" << endl;
                        break;
                    }
                }
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
