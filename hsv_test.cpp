#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

using namespace cv;
using namespace std;

//red
int minH = 0, maxH = 20;
int minS = 119, maxS = 181;
int minV = 171, maxV = 230;
//blue
int minH_1 = 93, maxH_1 = 130;
int minS_1 = 90, maxS_1 = 160;
int minV_1 = 142, maxV_1 = 202;

int main()
{
    //VideoCapture cap(1);
    Mat frame;
    frame = imread("./2022/blue.jpg");
    //if (!cap.isOpened())
    //{
    //    cout << "Failed to open camera." << endl;
    //    return -1;
    //}

    namedWindow("Trackbars", WINDOW_NORMAL);
    resizeWindow("Trackbars", 640, 480);

    //createTrackbar("Min H", "Trackbars", &minH, 255);
    //createTrackbar("Max H", "Trackbars", &maxH, 255);
    //createTrackbar("Min S", "Trackbars", &minS, 255);
    //createTrackbar("Max S", "Trackbars", &maxS, 255);
    //createTrackbar("Min V", "Trackbars", &minV, 255);
    //createTrackbar("Max V", "Trackbars", &maxV, 255);

    createTrackbar("Min H", "Trackbars", &minH_1, 255);
    createTrackbar("Max H", "Trackbars", &maxH_1, 255);
    createTrackbar("Min S", "Trackbars", &minS_1, 255);
    createTrackbar("Max S", "Trackbars", &maxS_1, 255);
    createTrackbar("Min V", "Trackbars", &minV_1, 255);
    createTrackbar("Max V", "Trackbars", &maxV_1, 255);


    while (true)
    {
        //cap >> frame;
        
        //if (frame.empty())
        //{
        //    cout << "Failed to capture frame." << endl;
        //    break;
        //}

        Mat hsv;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        Mat mask,mask_1;
        //inRange(hsv, Scalar(minH, minS, minV), Scalar(maxH, maxS, maxV), mask);
        //imshow("Mask", mask);
        inRange(hsv, Scalar(minH_1, minS_1, minV_1), Scalar(maxH_1, maxS_1, maxV_1), mask_1);
        imshow("Mask", mask_1);
        imshow("src", frame);

        if (waitKey(10) == 'q')
        {
            break;
        }
    }
        //waitKey(0);
    return 0;
}
