#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <cmath>


using namespace cv;
using namespace std;

int minH = 45, maxH = 129;
int minS = 0, maxS = 180;
int minV = 0, maxV = 147;

int minH_1 = 48, maxH_1 = 60;
int minS_1 = 61, maxS_1 = 132;
int minV_1 = 146, maxV_1 = 180;

int flag_circle = 0;
int flag_line = 1;

Mat elemment=getStructuringElement(MORPH_RECT,Size(5,5));

class PIDController
{
public:
    PIDController(double kp, double ki, double kd)
        : m_kp(kp), m_ki(ki), m_kd(kd), m_integral(0), m_prevError(0)
    {
    }

    double update(double error, double dt)
    {
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

int main()
{
    Mat frame;
    Mat hsv, mask, mask_1;
    Mat edge;
    VideoCapture cap(-1);

    vector<Vec4f> linesP;
    vector<Vec3f> circles;

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }
    namedWindow("Trackbars",WINDOW_NORMAL);
    resizeWindow("Trackbars",640,480);

    createTrackbar("Min H", "Trackbars", &minH, 255);
    createTrackbar("Max H", "Trackbars", &maxH, 255);
    createTrackbar("Min S", "Trackbars", &minS, 255);
    createTrackbar("Max S", "Trackbars", &maxS, 255);
    createTrackbar("Min V", "Trackbars", &minV, 255);
    createTrackbar("Max V", "Trackbars", &maxV, 255);

    //pid
    PIDController controller_line(1.0,0.0, 0.01);
    PIDController controller_circle_x(1.0, 0.0, 0.01);
    PIDController controller_circle_y(1.0, 0.0, 0.01);
    PIDController controller_green_circle_x(1.0, 0.0, 0.01);
    PIDController controller_green_circle_y(1.0, 0.0, 0.01);
    double dt = 1.0;
    double distance = 0.0;
    double dx = 0.0;
    double dy = 0.0;
    int flag=0;

    while (true)
    {
        cap >> frame;
        frame = frame(Range(0,480), Range(180,460));
        if (frame.empty())
        {
            cout << "Failed to capture frame." << endl;
            break;
        }

        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
        dilate(mask,mask,elemment);
        imshow("Mask", mask);
        if(flag_line==1)
        {
            HoughLinesP(mask, linesP, 1.0f, CV_PI / 180.0f, 100, 100, 10);
            cout<<linesP.size()<<endl;
            for (size_t i = 0; i < linesP.size(); i++)
            {
            // if (linesP.size()>=3)
            // {
            //     break;
            // }
                Vec4i l = linesP[i];
                line(frame, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3);

                int cx = (l[0] + l[2]) / 2;

                distance = double(cx)-double(frame.cols);
            }
        }
        


        if (flag_circle==0)
        {
            inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask_1);
        }
        else
        {
            inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask_1);
        }

        Canny(mask_1, edge, 100, 200);
        imshow("Mask_1", mask_1);
        imshow("edge", edge);

        // ���ͼ���е�Բ
        HoughCircles(edge, circles, HOUGH_GRADIENT, 1, 300000, 100, 25, 50);

        // ����ͼ������ĵ�
        Point imageCenter(frame.cols / 2, frame.rows / 2);

   
        for (size_t i = 0; i < circles.size(); i++)
        {

            Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // ����Բ����
            circle(frame, center, radius, Scalar(0, 0, 255));

            // ����Բ����ͼ�����ĵ�ƫ��ֵ
            dx = double(center.x) - double(imageCenter.x);
            dy = double(center.y) - double(imageCenter.y);
            flag_line = 0;
        }


        
        imshow("frame", frame);
        if (flag_line==1)
        {
            //output pid of line
            double distanceOutput = controller_line.update(distance, dt);
            distance += distanceOutput;
            if (distance > 150.0)
            {
                distance = 150.0;
            }
            if (distance < -150.0)
            {
                distance = -150.0;
            }
            cout << "distance=" << distance << endl;
        }
        if(flag_line==0)
        {
            if (flag_circle==0)
            {
                double dxoutput = controller_green_circle_x.update(dx, dt);
                double dyoutput = controller_green_circle_y.update(dy, dt);
                dx += dxoutput;
                dy += dyoutput;
                if (dx > 150.0)
                {
                    dx = 150.0;
                }
                if (dx < -150.0)
                {
                    dx = -150.0;
                }
                if (dy > 150.0)
                {
                    dy = 150.0;
                }
                if (dy < -150.0)
                {
                    dy = -150.0;
                }
                cout << "green_circle_dx=" << dx << endl;
                cout << "green_circle_dy=" << dy << endl;
                if (abs(dx) < 10 && abs(dy) < 10)
                {
                    flag++;
                    cout << "ok" << endl;
                    if (flag>2)
                    {
                        flag = 0;
                        flag_line = 1;
                        flag_circle = 1;
                        cout << "green_circle is finish" << endl;
                        //wait for sometimes and do somethings
                    }
                }
            }
            if(flag_circle==1)
            {
                double dxoutput = controller_circle_x.update(dx, dt);
                double dyoutput = controller_circle_y.update(dy, dt);
                dx += dxoutput;
                dy += dyoutput;
                if (dx > 150.0)
                {
                    dx = 150.0;
                }
                if (dx < -150.0)
                {
                    dx = -150.0;
                }
                if (dy > 150.0)
                {
                    dy = 150.0;
                }
                if (dy < -150.0)
                {
                    dy = -150.0;
                }
                cout << "circle_dx=" << dx << endl;
                cout << "circle_dy=" << dy << endl;
                if (abs(dx) < 10 && abs(dy) < 10)
                {
                    flag++;
                    cout << "ok" << endl;
                    if (flag>2)
                    {
                        cout << "circle is finish" << endl;
                    }

                }
            }
        }
        if (waitKey(33) == 'q')
        {
            break;
        }
    }

    return 0;
}

