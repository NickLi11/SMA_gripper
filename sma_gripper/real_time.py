import serial
import cv2
import math
import numpy as np
import time
def prose(img):
    final = []
    draw = []
    img = cv2.resize(img, (400, 400), interpolation=cv2.INTER_CUBIC)
    cv2.imshow("original", img)
    blurred = cv2.GaussianBlur(img, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    # 定义红色无图的HSV阈值
    lower_red = np.array([0, 50, 120])
    upper_red = np.array([7, 211, 255])
    # 对图片进行二值化处理
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # 腐蚀操作
    # mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，先腐蚀后膨胀以滤除噪声
    # mask = cv2.dilate(mask, None, iterations=2)
    #cv2.imshow('mask', mask)
    # circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 20,
    #              param1=100,
    #              param2=30,
    #              minRadius=0,
    #              maxRadius=0)
    # print(circles)
    _, contours, h = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rect = None
    max_area = 0
    img2 = img
    for cnt in contours:
        approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
        # if cv2.contourArea(approx) > 700 and cv2.contourArea(approx) < 10000:
        max_area = cv2.contourArea(approx)
        cv2.drawContours(img2, [approx], 0, (0, 255, 0), -1)
        x = 0
        y = 0
        for i in cnt:
            x = x + i[0][0]
            y = y + i[0][1]
        final.append([x / len(cnt), y / len(cnt)])
    # for j in final:
    # cv2.circle(img2, (int(j[0]), int(j[1])), 10, (0, 0, 255), 3)
    if(len(final)==3):

        if ((final[0][0] < final[1][0]) & (final[0][0] < final[2][0])):
            draw.append(final[0])
            if (final[1][0] < final[2][0]):
                draw.append(final[1])
                draw.append(final[2])
            else:
                draw.append(final[2])
                draw.append(final[1])
        if ((final[1][0] < final[0][0]) & (final[1][0] < final[2][0])):
            draw.append(final[1])
            if (final[0][0] < final[2][0]):
                draw.append(final[0])
                draw.append(final[2])
            else:
                draw.append(final[2])
                draw.append(final[0])
        else:
            draw.append(final[2])
            if (final[1][0] < final[0][0]):
                draw.append(final[1])
                draw.append(final[0])
            else:
                draw.append(final[0])
                draw.append(final[1])
        u = 0
        for j in draw:
            u = u + 1
            cv2.circle(img2, (int(j[0]), int(j[1])), 5, (0, 0, 255), 2)
        cv2.line(img2, (int(draw[0][0]), int(draw[0][1])), (int(draw[1][0]), int(draw[1][1])), (155, 155, 0), 2)
        cv2.line(img2, (int(draw[1][0]), int(draw[1][1])), (int(draw[2][0]), int(draw[2][1])), (155, 155, 0), 2)
        l1 = math.sqrt(pow(draw[1][0] - draw[0][0], 2) + pow(draw[1][1] - draw[0][1], 2))
        l2 = math.sqrt(pow(draw[2][0] - draw[1][0], 2) + pow(draw[2][1] - draw[1][1], 2))
        l3 = math.sqrt(pow(draw[2][0] - draw[0][0], 2) + pow(draw[2][1] - draw[0][1], 2))
        ctheta = (pow(l1, 2) + pow(l2, 2) - pow(l3, 2)) / (2 * l1 * l2)
        theta = math.acos(ctheta) * 360 / (2 * 3.1415926)
        return img,img2,mask,theta
    else:

        for q in final:
            cv2.circle(img2, (int(q[0]), int(q[1])), 5, (0, 0, 255), 2)
        theta=0
        return img2,img2,img2,theta


if __name__ == '__main__':

    cap = cv2.VideoCapture(1) # 使用第5个摄像头（我的电脑插了5个摄像头）
    ArduinoUnoSerial = serial.Serial('com4', 9600)
    ava=[]
    last=0
    flag=0
    while(True):
        ret, frame = cap.read() # 读取一帧的图像
        p1,p2,p3,gama=prose(frame)
        if(gama!=0):
            if(len(ava)<20):
                ava.append(gama)
            if(len(ava)>=20):
                x=0
                y=0
                for i in ava:
                    x=x+i
                y=x/20
                if(((abs(last-y)<=10)&(last!=0))|(flag==0)):
                    last=y
                    flag=1
                    print(time.clock(),",",180-y,";")
                    ArduinoUnoSerial.write(str(180-y).encode())
                    #print("time:%s", time.clock())
                    ava=[]
                else:
                    ava=[]
        cv2.imshow(' Recognition3', p2)
        if cv2.waitKey(1) & 0xFF == ord('q'):
             break

    cap.release() # 释放摄像头
    cv2.destroyAllWindows()