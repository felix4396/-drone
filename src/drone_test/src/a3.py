#!/usr/bin/python
import cv2
import numpy as np

# 读取图像
cap = cv2.VideoCapture(0)

while True:
    ret, img = cap.read()
    
    yyy=0
    
    height, width, _ = img.shape

    # 将图像转换为灰度图像
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 应用Canny边缘检测算法
    edges = cv2.Canny(gray, 50, 150, apertureSize=3)
    
    # 定义绿色范围并寻找圆形
    green_lower = np.array([30, 100, 100])
    green_upper = np.array([70, 255, 255])

    # 将图像转换为HSV颜色空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 使用掩模滤波器查找绿色圆形
    mask = cv2.inRange(hsv, green_lower, green_upper)

    circles1 = cv2.HoughCircles(mask,cv2.HOUGH_GRADIENT,1,100000,param1=100,param2=30,minRadius=0,maxRadius=0)

    if circles1 is not None:   
    
        yyy=1 

        circles = circles1[0,:,:]

        circles = np.uint16(np.around(circles))

        for i in circles[:]:

            cv2.circle(img,(i[0],i[1]),i[2],(255,0,0),5)

            cv2.circle(img,(i[0],i[1]),2,(255,0,255),10)

            cv2.rectangle(img,(i[0]-i[2],i[1]+i[2]),(i[0]+i[2],i[1]-i[2]),(255,255,0),5)

            offset_x = i[0] - width / 2
            offset_y = height / 2 - i[1]
            print(offset_x,'   ',offset_y'   ',i[2])
    else:
        yyy=0
        

    if yyy==0:
    # 使用霍夫直线变换在图像中查找较粗的黑线
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi/180, threshold=100, minLineLength=100, maxLineGap=10)

        if lines is not None:
            # 取第一条直线作为较粗的黑线
            line = lines[0][0]
            x1, y1, x2, y2 = line
            # 计算黑线的中心坐标
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            # 计算黑线与图像中心的偏差距离
            img_center_x = img.shape[1] / 2
            img_center_y = img.shape[0] / 2
            deviation_x = img_center_x - center_x
            deviation_y = img_center_y - center_y
            print("Black line deviation: ({})".format(deviation_x))


    # 以中心点为原点，向右和向下分别绘制横线和竖线
    
    cv2.line(img, (width // 2, 0), (width // 2, height), (0, 0, 255), 1)
    cv2.line(img, (0, height // 2), (width, height // 2), (0, 0, 255), 1)

    cv2.imshow('image', img)

    if cv2.waitKey(33)=='q':
        cv2.destroyAllWindows()
        break 
    	

