import cv2
import numpy as np
import math

final=[]
draw=[]
img=cv2.imread("C:/Users/licon/PycharmProjects/untitled2/venv/cuttt.jpg")
img=cv2.resize(img,(400,400),interpolation=cv2.INTER_CUBIC)
cv2.imshow("original",img)
blurred = cv2.GaussianBlur(img, (11, 11), 0)
hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
# 定义红色无图的HSV阈值
lower_red = np.array([0, 50, 120])
upper_red = np.array([7, 211, 255])
# 对图片进行二值化处理
mask = cv2.inRange(hsv, lower_red, upper_red)
cv2.imshow("m1",mask)
# 腐蚀操作
mask = cv2.erode(mask, None, iterations=2)
# 膨胀操作，先腐蚀后膨胀以滤除噪声
mask = cv2.dilate(mask, None, iterations=2)
cv2.imshow('mask', mask)
cv2.waitKey(0)
cv2.destroyAllWindows()