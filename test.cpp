#include <iostream>
#include <opencv2/opencv.hpp>
#include <zbar.h>

using namespace std;
using namespace cv;
using namespace zbar;

int code_scan(Mat color_image)
{
    Mat image;
    int code=0;
    color_image.copyTo(image);
    // Convert to grayscale
    cvtColor(image, image, CV_BGR2GRAY);
    equalizeHist(image, image);
    int width = image.cols;
    int height = image.rows;
    uchar *raw = (uchar *)image.data;

    Image imageZbar(width, height, "Y800", raw, width * height);
    scanner.scan(imageZbar); //扫描条码
    Image::SymbolIterator symbol = imageZbar.symbol_begin();
    if (imageZbar.symbol_begin() == imageZbar.symbol_end())
    {
        ROS_INFO("zbar_error");
        return -1;
    }
    else
    {
        string code_str;
        for (; symbol != imageZbar.symbol_end(); ++symbol)
        {
            ROS_INFO("zbar_type:%s", symbol->get_type_name().c_str());
            ROS_INFO("zbar_code:%s", symbol->get_data().c_str());
            code_str = symbol->get_data();
            code = code_str[0] - '0';
            if(code<0 || code>9)
            {
                code=0;
            }
        }
        barcode=1;
    }
    return code;
}

int main()
{

    return 0;
}