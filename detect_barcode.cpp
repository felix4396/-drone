#include<opencv2/opencv.hpp>
#include<opencv2/core/mat.hpp>
#include<iostream>
#include<vector>
#include<zbar.h>
 
using namespace std;
using namespace cv;
using namespace zbar;

void find_barcode(Mat src,Mat dst,vector<Mat>&ROI_Rect)
{
    Mat src_gray;
	cvtColor(src, src_gray, CV_RGB2GRAY);
	imshow("gray", src_gray);
 
	GaussianBlur(src_gray, src_gray, Size(3, 3), 0, 0);
	Mat canny;
	Sobel(src_gray, canny, CV_32F, 1, 0, 3);//检测x方向的边缘
	convertScaleAbs(canny, canny);//将其转化为8位图像
	blur(canny, canny, Size(5, 5));//模糊处理
	threshold(canny, canny, 40, 255, THRESH_BINARY | THRESH_OTSU);
	imshow("Sobel", canny);
 
	Mat open;
	Mat structure = getStructuringElement(MORPH_RECT, Size(9, 9), Point(-1, -1));
	Mat getStructuringElement(int shape, Size esize, Point anchor = Point(-1, -1));
 
	morphologyEx(canny, open, CV_MOP_OPEN, structure);
	imshow("open", open);
 
	Mat close;
	morphologyEx(open, close, CV_MOP_CLOSE, structure);
	imshow("close", close);
 
	Mat cont = close;
	vector<vector<Point>> contours;
	vector<Vec4i> hierachy;
	vector<Point> convexs;       
	findContours(cont, contours, hierachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0, 0));
 
	
	RotatedRect Out_Rect,rect;
	Point2f ptr[4];
	for (size_t i = 0; i < contours.size(); i++)
	{
		rect = minAreaRect(contours[i]);
		if (rect.size.width>src.cols/4 && rect.size.height > src.rows/4 && rect.size.width/ rect.size.height >=1 && rect.size.width / rect.size.height <= 2)//长宽比例在1-2之间
		{
			Out_Rect = rect;
		}
 
	}
	
	Out_Rect.points(ptr);
	for (int i = 0; i < 4; i++)
	{
		line(dst, ptr[i], ptr[(i + 1) % 4], Scalar(0, 255, 0), 5, 8);
	}
	imshow("output", dst);
 
	Mat Result = src_gray(Rect(Out_Rect.center.x - Out_Rect.size.width*0.65, Out_Rect.center.y - Out_Rect.size.height*0.6, Out_Rect.size.width*1.3, Out_Rect.size.height*1.2));
	imshow("Result", Result);
    //ROI_Rect.push_back(Result);
}

int main()
{
 
	Mat src, dst;
	src = imread("C:/Users/LBJ/Desktop/条形码识别/条形码.jpg");
	if (!src.data)
	{
		cout << "The iamge is empty" << endl;
		return -1;
	}
	src.copyTo(dst);
 
	ImageScanner scanner;
	scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
	int width = src_gray.cols;
	int height = src_gray.rows;
	uchar *raw = (uchar *)src_gray.data;
	Image imageZbar(width, height, "Y800", raw, width * height);
	scanner.scan(imageZbar); //扫描条码    
	Image::SymbolIterator symbol = imageZbar.symbol_begin();
 
	if (imageZbar.symbol_begin() == imageZbar.symbol_end())
	{
		cout << "查询条码失败，请检查图片！" << endl;
	}
	for (; symbol != imageZbar.symbol_end(); ++symbol)
	{
		cout << "类型：" << endl << symbol->get_type_name() << endl << endl;
		cout << "条码：" << endl << symbol->get_data() << endl << endl;
 
	}
 
	waitKey(0);
	return 0;
}