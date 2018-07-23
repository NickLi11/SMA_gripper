# Import the modules
import cv2
from sklearn.externals import joblib
from skimage.feature import hog
import numpy as np

# Load the classifier
clf = joblib.load("digits_cls.pkl")

# Read the input image 
im = cv2.imread("006.png")

# Convert to grayscale and apply Gaussian filtering
im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

# Threshold the image

#kernel = np.ones((5,5),np.uint8)    
ret, im_th = cv2.threshold(im_gray, 115, 255, cv2.THRESH_BINARY_INV)
cv2.imwrite("thresh01.png",im_th)   
#im_th= cv2.erode(im_th,kernel,iterations = 1)
#im_th = cv2.dilate(im_th,kernel,iterations = 1)
#im_th = cv2.erode(im_th,kernel,iterations = 1)

#ret, im_th = cv2.threshold(im_gray, 125, 255, cv2.THRESH_BINARY_INV)
cv2.imshow("thresh", im_th)
cv2.waitKey()

# Find contours in the image
ctrs, hier = cv2.findContours(im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(im, ctrs, -1, (255,0,0), 3)

# Get rectangles contains each contour
rects = [cv2.boundingRect(ctr) for ctr in ctrs if cv2.contourArea(ctr) > 50 and cv2.contourArea(ctr) < 1200]

# For each rectangular region, calculate HOG features and predict
# the digit using Linear SVM.
for rect in rects:
    # Draw the rectangles
    cv2.rectangle(im, (rect[0], rect[1]), (rect[0] + rect[2], rect[1] + rect[3]), (0, 255, 0), 3) 
    # Make the rectangular region around the digit
    leng = int(rect[3] * 1.6)
    pt1 = int(rect[1] + rect[3] // 2 - leng // 2)
    pt2 = int(rect[0] + rect[2] // 2 - leng // 2)


    cv2.circle(im,(rect[0] + rect[2]/2,rect[1] + rect[3]/2), 5, (0,0,255), -1)

    roi = im_th[pt1:pt1+leng, pt2:pt2+leng]
    if len(roi) < 1:
        continue
    if len(roi[0]) < 1:
        continue
    # Resize the image
    roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
    roi = cv2.dilate(roi, (3, 3))


    # Calculate the HOG features
    roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(14, 14), cells_per_block=(1, 1), visualise=False)
    nbr = clf.predict(np.array([roi_hog_fd], 'float64'))
    cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]),cv2.FONT_HERSHEY_DUPLEX, 1, (0, 255, 255), 3)

cv2.imshow("Resulting Image with Rectangular ROIs", im)
cv2.waitKey()