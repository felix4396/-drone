#include <opencv2/opencv.hpp>
#include <iostream>    
#include <opencv2\core\core.hpp>
#include <stdio.h>
#include <string>
#include <sstream>
#include <zbar.h>

using namespace cv;
using namespace std;
using namespace zbar;

// 用于矫正
Mat transformCorner(Mat src, RotatedRect rect);
// 用于判断角点
bool IsQrPoint(vector<Point>& contour, Mat& img);
bool isCorner(Mat& image);
double Rate(Mat& count);

int main()
{
		VideoCapture cap;
		Mat src;
		cap.open(1);                             //打开相机，电脑自带摄像头一般编号为0，外接摄像头编号为1，主要是在设备管理器中查看自己摄像头的编号。

		//cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);  //设置捕获视频的宽度
		//cap.set(CV_CAP_PROP_FRAME_HEIGHT, 400);  //设置捕获视频的高度

		if (!cap.isOpened())                         //判断是否成功打开相机
		{
			cout << "摄像头打开失败!" << endl;
			return -1;
		}
	
		//Mat src;
		//src = imread("./qrcode.jpg");
		while (true)
		{
			cap >> src;                                //从相机捕获一帧图像


			Mat srcCopy = src.clone();
			
			//canvas为画布 将找到的定位特征画出来
			Mat canvas;
			canvas = Mat::zeros(src.size(), CV_8UC3);

			Mat srcGray;

			//center_all获取特性中心
			vector<Point> center_all;

			// 转化为灰度图
			cvtColor(src, srcGray, COLOR_BGR2GRAY);
			Mat qrcode = srcGray;
			// 计算直方图
			convertScaleAbs(src, src);
			equalizeHist(srcGray, srcGray);

			// 设置阈值根据实际情况 如视图中已找不到特征 可适量调整
			threshold(srcGray, srcGray, 0, 255, THRESH_BINARY | THRESH_OTSU);
			//imshow("threshold", srcGray);

			Mat structure_close = getStructuringElement(MORPH_RECT, Size(3, 3));
			Mat structure_open = getStructuringElement(MORPH_RECT, Size(5, 5));
			Mat close, open;

			//闭运算，连接特征点
			morphologyEx(srcGray, close, MORPH_CLOSE, structure_close);
			//imshow("close", close);

			//开运算，消除毛刺和小轮廓
			morphologyEx(close, open, MORPH_OPEN, structure_open);
			//imshow("open", open);

			/*contours是第一次寻找轮廓*/
			/*contours2是筛选出的轮廓*/
			vector<vector<Point>> contours;

			//	用于轮廓检测
			vector<Vec4i> hierarchy;
			findContours(open, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

			// 小方块的数量
			int numOfRec = 0;

			// 检测方块
			int ic = 0;
			int parentIdx = -1;
			for (int i = 0; i < contours.size(); i++)
			{
				//判断是否是父轮廓
				if (hierarchy[i][2] != -1 && ic == 0)
				{
					parentIdx = i;
					ic++;
				}
				//判断是否有子轮廓
				else if (hierarchy[i][2] != -1)
				{
					ic++;
				}
				//重置
				else if (hierarchy[i][2] == -1)
				{
					parentIdx = -1;
					ic = 0;
				}
				//如果有上述二级结构，进一步判断
				if (ic >= 2 && ic <= 2)
				{
					//判断是否是回字
					if (IsQrPoint(contours[parentIdx], src)) {
						RotatedRect rect = minAreaRect(Mat(contours[parentIdx]));

						// 画图部分
						Point2f points[4];
						rect.points(points);
						for (int j = 0; j < 4; j++) {
							line(src, points[j], points[(j + 1) % 4], Scalar(0, 255, 0), 2);
						}
						cv::drawContours(canvas, contours, parentIdx, Scalar(0, 0, 255), -1);

						// 如果满足条件则存入
						center_all.push_back(rect.center);
						numOfRec++;
					}
					ic = 0;
					parentIdx = -1;
				}
			}
			// 连接三个正方形的部分
			for (int i = 0; i < center_all.size(); i++)
			{
				line(canvas, center_all[i], center_all[(i + 1) % center_all.size()], Scalar(255, 0, 0), 3);
			}

			vector<vector<Point>> contours3;
			Mat canvasGray;
			cvtColor(canvas, canvasGray, COLOR_BGR2GRAY);
			findContours(canvasGray, contours3, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
			//imshow("canvas_gray", canvasGray);


			//vector<Point> maxContours;
			//double maxArea = 0;
			// 在原图中画出二维码的区域
			for (int i = 0; i < contours3.size(); i++)
			{
				RotatedRect rect = minAreaRect(contours3[i]);
				//imshow("rect", rect);
				Point2f boxpoint[4];
				rect.points(boxpoint);
				for (int i = 0; i < 4; i++)
					line(src, boxpoint[i], boxpoint[(i + 1) % 4], Scalar(0, 0, 255), 3);
				double r = 0.5 * sqrt(pow(rect.size.width, 2) + pow(rect.size.height, 2));
				double cor_x, cor_y;
				cor_x = rect.center.x - r;
				cor_y = rect.center.y - r;
				if (cor_x< 0)
				{
					cor_x = 0;
				}
				else if(rect.center.x + r> srcGray.size().width)
				{
					cor_x = srcGray.size().width - 2 * r;
				}
				if (cor_y < 0)
				{
					cor_y = 0;
				}
				else if (rect.center.y + r > srcGray.size().height)
				{
					cor_y = srcGray.size().height - 2 * r;
				}
				Rect max_rect(cor_x, cor_y, 2 * r, 2 * r);
				qrcode = srcGray(max_rect);
				//double area = contourArea(contours3[i]);
				//if (area > maxArea) {
				//	maxContours = contours3[i];
				//	maxArea = area;
				//}
			}
			imshow("src", src);
			//imshow("roi", qrcode);

			zbar::ImageScanner scanner;
			scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

			//cvtColor(srcCopy, srcCopy, COLOR_BGR2GRAY);
			int width = qrcode.cols;
			int height = qrcode.rows;
			Image image(width, height, "Y800", qrcode.data, width * height);  // 图片格式转换
			scanner.scan(image);
			Image::SymbolIterator symbol = image.symbol_begin();
			if (image.symbol_begin() == image.symbol_end())
			{
				cout << "查询二维码失败，请检查图片！" << endl;
			}
			for (; symbol != image.symbol_end(); ++symbol)
			{
				cout << "类型：" << endl << symbol->get_type_name() << endl << endl;
				cout << "条码：" << endl << symbol->get_data() << endl << endl;

				cout << "x:" << symbol->get_location_x(0) << endl << endl;
				cout << "y:" << symbol->get_location_y(0) << endl << endl;
				//定位信息，
				int x = symbol->get_location_x(0);
				int y = symbol->get_location_y(0);

				//根据zbar 返回的多边形的像素点位置 计算宽高 
				//默认二维码是垂直水平的
				int min_x = 0, min_y = 0, max_x = 0, max_y = 0;
				Symbol::PointIterator pt = symbol->point_begin();
				Symbol::Point p = *pt;

				min_x = p.x;
				min_y = p.y;
				max_x = p.x;
				max_y = p.y;

				for (; pt != (Symbol::PointIterator)symbol->point_end(); ++pt) {
					p = *pt;
					min_x = min_x < p.x ? min_x : p.x;
					max_x = min_x > p.x ? max_x : p.x;

					min_y = min_y < p.y ? min_y : p.y;
					max_y = max_y > p.y ? max_y : p.y;
				}
			}
			//if (numOfRec < 3) {
			//	waitKey(10);

			//}
			//imshow("canvas", canvas);
			if (waitKey(1) == 'q')
			{
				break;
			}
		}
	return 0;
}

Mat transformCorner(Mat src, RotatedRect rect)
{
	// 获得旋转中心
	Point center = rect.center;
	// 获得左上角和右下角的角点，而且要保证不超出图片范围，用于抠图
	Point TopLeft = Point(cvRound(center.x), cvRound(center.y)) - Point(rect.size.height / 2, rect.size.width / 2);  //旋转后的目标位置
	TopLeft.x = TopLeft.x > src.cols ? src.cols : TopLeft.x;
	TopLeft.x = TopLeft.x < 0 ? 0 : TopLeft.x;
	TopLeft.y = TopLeft.y > src.rows ? src.rows : TopLeft.y;
	TopLeft.y = TopLeft.y < 0 ? 0 : TopLeft.y;

	int after_width, after_height;
	if (TopLeft.x + rect.size.width > src.cols) {
		after_width = src.cols - TopLeft.x - 1;
	}
	else {
		after_width = rect.size.width - 1;
	}
	if (TopLeft.y + rect.size.height > src.rows) {
		after_height = src.rows - TopLeft.y - 1;
	}
	else {
		after_height = rect.size.height - 1;
	}
	// 获得二维码的位置
	Rect RoiRect = Rect(TopLeft.x, TopLeft.y, after_width, after_height);

	//	dst是被旋转的图片 roi为输出图片 mask为掩模
	double angle = rect.angle;
	Mat mask, roi, dst;
	Mat image;
	// 建立中介图像辅助处理图像

	vector<Point> contour;
	// 获得矩形的四个点
	Point2f points[4];
	rect.points(points);
	for (int i = 0; i < 4; i++)
		contour.push_back(points[i]);

	vector<vector<Point>> contours;
	contours.push_back(contour);
	// 再中介图像中画出轮廓
	cv::drawContours(mask, contours, 0, Scalar(255, 255, 255), -1);
	// 通过mask掩膜将src中特定位置的像素拷贝到dst中。
	src.copyTo(dst, mask);
	// 旋转
	Mat M = getRotationMatrix2D(center, angle, 1);
	cv::warpAffine(dst, image, M, src.size());
	// 截图
	roi = image(RoiRect);

	return roi;
}

// 该部分用于检测是否是角点，与下面两个函数配合
bool IsQrPoint(vector<Point>& contour, Mat& img) {
	double area = contourArea(contour);
	// 角点不可以太小
	if (area < 30)
		return 0;
	RotatedRect rect = minAreaRect(Mat(contour));
	double w = rect.size.width;
	double h = rect.size.height;
	double rate = min(w, h) / max(w, h);
	if (rate > 0.7)
	{
		// 返回旋转后的图片，用于把“回”摆正，便于处理
		Mat image = transformCorner(img, rect);
		if (isCorner(image))
		{
			return 1;
		}
	}
	return 0;
}

// 计算内部所有白色部分占全部的比率
double Rate(Mat& count)
{
	int number = 0;
	int allpixel = 0;
	for (int row = 0; row < count.rows; row++)
	{
		for (int col = 0; col < count.cols; col++)
		{
			if (count.at<uchar>(row, col) == 255)
			{
				number++;
			}
			allpixel++;
		}
	}
	//cout << (double)number / allpixel << endl;
	return (double)number / allpixel;
}

// 用于判断是否属于角上的正方形
bool isCorner(Mat& image)
{
	// 定义mask
	Mat imgCopy, dstCopy;
	Mat dstGray;
	imgCopy = image.clone();
	// 转化为灰度图像
	cvtColor(image, dstGray, COLOR_BGR2GRAY);
	// 进行二值化

	threshold(dstGray, dstGray, 0, 255, THRESH_BINARY | THRESH_OTSU);
	dstCopy = dstGray.clone();  //备份

	// 找到轮廓与传递关系
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(dstCopy, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);


	for (int i = 0; i < contours.size(); i++)
	{
		//cout << i << endl;
		if (hierarchy[i][2] == -1 && hierarchy[i][3])
		{
			Rect rect = boundingRect(Mat(contours[i]));
			rectangle(image, rect, Scalar(0, 0, 255), 2);
			// 最里面的矩形与最外面的矩形的对比
			if (rect.width < imgCopy.cols * 2 / 7)      //2/7是为了防止一些微小的仿射
				continue;
			if (rect.height < imgCopy.rows * 2 / 7)      //2/7是为了防止一些微小的仿射
				continue;
			// 判断其中黑色与白色的部分的比例
			if (Rate(dstGray) > 0.20)
			{
				return true;
			}
		}
	}
	return  false;
}
