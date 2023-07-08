#include<iostream>
#include<opencv2/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
using namespace std;
using namespace cv;


//找到二维码所在的矩形区域
void Find_QR_Rect(Mat src, vector<Mat>&ROI_Rect)
{
	Mat gray;
	cvtColor(src, gray, COLOR_BGR2GRAY);

	Mat blur;
	GaussianBlur(gray, blur, Size(3, 3), 0);

	Mat bin;
	threshold(blur, bin, 0, 255, THRESH_BINARY_INV | THRESH_OTSU);//自动阈值、反转阈值

	//通过Size（5，1）开运算消除边缘毛刺
	Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 1));
	Mat open;
	morphologyEx(bin, open, MORPH_OPEN, kernel);
	//通过Size（21，1）闭运算能够有效地将矩形区域连接 便于提取矩形区域
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(21, 1));
	Mat close;
	morphologyEx(open, close, MORPH_CLOSE, kernel1);


	//使用RETR_EXTERNAL找到最外轮廓
	vector<vector<Point>>MaxContours;
	findContours(close, MaxContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);//简单近似

	for (int i = 0; i < MaxContours.size(); i++)
	{
		Mat mask = Mat::zeros(src.size(), CV_8UC3);
		mask = Scalar::all(255);

		double area = contourArea(MaxContours[i]);

		//通过面积阈值找到二维码所在矩形区域
		if (area > 6000 && area < 100000)
		{
			//计算最小外接矩形
			RotatedRect MaxRect = minAreaRect(MaxContours[i]);
			//计算最小外接矩形宽高比
			double ratio = MaxRect.size.width / MaxRect.size.height;

			if (ratio > 0.8 && ratio < 1.2)
			{
				Rect MaxBox = MaxRect.boundingRect();

				//rectangle(src, Rect(MaxBox.tl(), MaxBox.br()), Scalar(255, 0, 255), 2);
				//将矩形区域从原图抠出来
				Mat ROI = src(Rect(MaxBox.tl(), MaxBox.br()));

				ROI.copyTo(mask(MaxBox));

				ROI_Rect.push_back(mask);

			}

		}

	}

}


//对找到的矩形区域进行识别是否为二维码
int Dectect_QR_Rect(Mat src,Mat &canvas,vector<Mat>&ROI_Rect)
{
	//用于存储检测到的二维码
	vector<vector<Point>>QR_Rect;
	
	//遍历所有找到的矩形区域
	for (int i = 0; i < ROI_Rect.size(); i++)
	{
		Mat gray;
		cvtColor(ROI_Rect[i], gray, COLOR_BGR2GRAY);

		Mat bin;
		threshold(gray, bin, 0, 255, THRESH_BINARY_INV|THRESH_OTSU);

		//通过hierarchy、RETR_TREE找到轮廓之间的层级关系
		vector<vector<Point>>contours;
		vector<Vec4i>hierarchy;
		findContours(bin, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);

		//父轮廓索引
		int ParentIndex = -1;
		int cn = 0;

		//用于存储二维码矩形的三个“回”
		vector<Point>rect_points;
		for (int i = 0; i < contours.size(); i++)
		{
			//hierarchy[i][2] != -1 表示该轮廓有子轮廓  cn用于计数“回”中第几个轮廓
			if (hierarchy[i][2] != -1 && cn == 0)
			{
				ParentIndex = i;
				cn++;
			}
			else if (hierarchy[i][2] != -1 && cn == 1)
			{
				cn++;
			}
			else if (hierarchy[i][2] == -1)
			{
				//初始化
				ParentIndex = -1;
				cn = 0;
			}

			//如果该轮廓存在子轮廓，且有2级子轮廓则认定找到‘回’
			if (hierarchy[i][2] != -1 && cn == 2)
			{
				drawContours(canvas, contours, ParentIndex, Scalar::all(255), -1);

				RotatedRect rect;

				rect = minAreaRect(contours[ParentIndex]);

				rect_points.push_back(rect.center);

			}

		}

		//将找到地‘回’连接起来
		for (int i = 0; i < rect_points.size(); i++)
		{
			line(canvas, rect_points[i], rect_points[(i + 1) % rect_points.size()], Scalar::all(255), 5);
		}

		QR_Rect.push_back(rect_points);

	}

	
	return QR_Rect.size();

}

int main()
{

	Mat src = imread("6.png");

	if (src.empty())
	{
		cout << "No image data!" << endl;
		system("pause");
		return 0;
	}

	vector<Mat>ROI_Rect;
	Find_QR_Rect(src, ROI_Rect);

	Mat canvas = Mat::zeros(src.size(), src.type());
	int flag = Dectect_QR_Rect(src, canvas, ROI_Rect);
	//imshow("canvas", canvas);

	if (flag <= 0)
	{
		cout << "Can not detect QR code!" << endl;	
		//system("pause");
		return 0;
	}

	cout << "检测到" << flag << "个二维码。" << endl;


	//框出二维码所在位置
	Mat gray;
	cvtColor(canvas, gray, COLOR_BGR2GRAY);

	vector<vector<Point>>contours;
	findContours(gray, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	Point2f points[4];

	for (int i = 0; i < contours.size(); i++)
	{
		RotatedRect rect = minAreaRect(contours[i]);
		
		rect.points(points);

		for (int j = 0; j < 4; j++)
		{
			line(src, points[j], points[(j + 1) % 4], Scalar(0, 255, 0), 2);
		}

	}


	imshow("source", src);
	waitKey(0);
	destroyAllWindows();

	system("pause");
	return 0;
}

