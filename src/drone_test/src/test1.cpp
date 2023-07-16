#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>
#include <chrono>

using namespace cv;
using namespace std;

//black
int minH = 0, maxH = 180;
int minS = 0, maxS = 80;
int minV = 40, maxV = 92;
//blue
int minH_1 = 90, maxH_1 = 115;
int minS_1 = 30, maxS_1 = 130;
int minV_1 = 100, maxV_1 = 150;

int main()
{
    VideoCapture cap(1);
    Mat frame;
    frame = imread("./2022/circle1.jpg");
    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }

    namedWindow("Trackbars", WINDOW_NORMAL);
    resizeWindow("Trackbars", 640, 480);

    createTrackbar("Min H", "Trackbars", &minH, 255);
    createTrackbar("Max H", "Trackbars", &maxH, 255);
    createTrackbar("Min S", "Trackbars", &minS, 255);
    createTrackbar("Max S", "Trackbars", &maxS, 255);
    createTrackbar("Min V", "Trackbars", &minV, 255);
    createTrackbar("Max V", "Trackbars", &maxV, 255);

    //createTrackbar("Min H", "Trackbars", &minH_1, 255);
    //createTrackbar("Max H", "Trackbars", &maxH_1, 255);
    //createTrackbar("Min S", "Trackbars", &minS_1, 255);
    //createTrackbar("Max S", "Trackbars", &maxS_1, 255);
    //createTrackbar("Min V", "Trackbars", &minV_1, 255);
    //createTrackbar("Max V", "Trackbars", &maxV_1, 255);


    while (true)
    {
        auto start_all = std::chrono::high_resolution_clock::now();
        cap >> frame;

        if (frame.empty())
        {
            cout << "Failed to capture frame." << endl;
            break;
        }

        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        Mat mask, mask_1;
        inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
        imshow("Mask", mask);

        Mat close,structure_close;
        structure_close = getStructuringElement(MORPH_RECT, Size(7, 7));
        morphologyEx(mask, close, MORPH_CLOSE, structure_close);
        imshow("close", close);
        
        Mat open, structure_open;
        structure_open = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(close, open, MORPH_OPEN, structure_open);
        imshow("open", open);

        Mat edge;
        Canny(open, edge, 100, 200);
        imshow("edge", edge);

        // 获取当前时间点
        auto start = std::chrono::high_resolution_clock::now();

        vector<Vec3f> circles;
        HoughCircles(edge, circles, HOUGH_GRADIENT, 1, 3000, 100, 20, 75,120);

        // 获取当前时间点
        auto end = std::chrono::high_resolution_clock::now();
        // 计算函数执行的时间
        std::chrono::duration<double> duration = end - start;
        double timeInSeconds = duration.count();
        std::cout << "函数执行时间：" << timeInSeconds << " 秒" << std::endl;

        //cout << circles.size() << endl;
        if (circles.size())
        {
            for (size_t i = 0; i < circles.size(); i++)
            {

                Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
                int radius = cvRound(circles[i][2]);
                // 绘制圆轮廓
                circle(frame, center, radius, Scalar(0, 0, 255));
                cout << "radius:" << radius << endl;
            }
        }
        //inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask_1);
        //imshow("Mask", mask_1);
        imshow("src", frame);

        if (waitKey(10) == 'q')
        {
            break;
        }
        // 获取当前时间点
        auto end_all = std::chrono::high_resolution_clock::now();
        // 计算函数执行的时间
        std::chrono::duration<double> duration_all = end_all - start_all;
        double timeInSeconds_all = duration.count();
        std::cout << "帧率" << 1/timeInSeconds_all << std::endl;
    }
    //waitKey(0);
    return 0;
}
