import cv2

def main():
    # 创建VideoCapture对象，参数为摄像头索引号，0表示默认摄像头
    cap = cv2.VideoCapture(0)

    while True:
        # 读取摄像头图像
        ret, frame = cap.read()

        # 检查图像是否读取成功
        if not ret:
            print("无法读取摄像头图像")
            break

        # 在窗口中显示图像
        cv2.imshow('Camera', frame)

        # 按下 'q' 键退出循环
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 释放摄像头资源和关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
