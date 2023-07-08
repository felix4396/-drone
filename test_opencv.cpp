#include <opencv2/opencv.hpp>


int main() {
    cv::VideoCapture cap(0);  // 创建VideoCapture对象，参数为摄像头索引号，0表示默认摄像头

    if (!cap.isOpened()) {
        std::cout << "无法打开摄像头" << std::endl;
        return -1;
    }

    cv::namedWindow("Camera", cv::WINDOW_NORMAL);  // 创建一个窗口

    while (true) {
        cv::Mat frame;
        cap.read(frame);  // 读取摄像头图像

        if (frame.empty()) {
            std::cout << "无法读取摄像头图像" << std::endl;
            break;
        }

        cv::imshow("Camera", frame);  // 在窗口中显示图像

        // 按下 'q' 键退出循环
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    cv::destroyAllWindows();  // 关闭窗口

    return 0;
}
