#!/usr/bin/env python
import roslib
import sys
import rospy
import numpy as np
from lab4_cam.srv import ImageSrv, ImageSrvResponse
import cv2, time, sys
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from scipy.misc.pilutil import imresize
import cv2
from skimage.feature import hog

#sys arg 1
#TRAIN_DATA_IMG = 'digits.png'

#sys arg 2
#USER_IMG = 'test_image.png'

DIGIT_DIM = 20 # size of each digit is SZ x SZ
CLASS_N = 10 # 0-9
bridge = CvBridge()
#This method splits the input training image into small cells (of a single digit) and uses these cells as training data.
#The default training image (MNIST) is a 1000x1000 size image and each digit is of size 20x20. so we divide 1000/20 horizontally and 1000/20 vertically.
#If you are going to use a custom digit training image, then adjust the code below so that it properly captures the digits in your image.
#Also, change the labelling scheme in line 41 to correspond to your image.
class KNN_MODEL():                #can also define a custom model in a similar class wrapper with train and predict methods
    def __init__(self, k = 3):
        self.k = k
        self.model = cv2.KNearest()

    def train(self, samples, responses):
        self.model = cv2.KNearest()
        self.model.train(samples, responses)

    def predict(self, samples):
        retval, results, neigh_resp, dists = self.model.find_nearest(samples, self.k)
        return results.ravel()

class digit_processing:
    
    def __init__(self):
        self.bridge = CvBridge()
    def split2d(self,img, cell_size, flatten=True):
        h, w = img.shape[:2]
        sx, sy = cell_size
        cells = [np.hsplit(row, w//sx) for row in np.vsplit(img, h//sy)]
        cells = np.array(cells)
        #cv2.imshow('thresh021.png', cells[1]) 
        if flatten:
            cells = cells.reshape(-1, sy, sx)
        return cells

    def load_digits(self,fn):
        print 'loading "%s for training" ...' % fn
        digits_img = cv2.imread(fn, 0)
        digits = self.split2d(digits_img, (DIGIT_DIM, DIGIT_DIM))
        cv2.imwrite('thresh023.png', digits[15]) 
        labels = np.repeat(np.arange(CLASS_N), len(digits)/CLASS_N)
        #2500 samples in the digits.png so repeat 0-9 2500/10(0-9 - no. of classes) times.
        return digits, labels
    def contains(self,r1, r2):
        r1_x1 = r1[0]
        r1_y1 = r1[1]
        r2_x1 = r2[0]
        r2_y1 = r2[1]
        r1_x2 = r1[0]+r1[2]
        r1_y2 = r1[1]+r1[3]
        r2_x2 = r2[0]+r2[2]
        r2_y2 = r2[1]+r2[3]
        #does r1 contain r2?
        return r1_x1 < r2_x1 < r2_x2 < r1_x2 and r1_y1 < r2_y1 < r2_y2 < r1_y2


    def pixels_to_hog_20(self,pixel_array):
        hog_featuresData = []
        for img in pixel_array:
            #img = 20x20
            print "imgimgimgimgimgimgimgimgimgimgimgimgimgimgimigmimgimig"
            print img
            fd = hog(img, orientations=9, pixels_per_cell=(10,10),cells_per_block=(1,1), visualise=False)
            hog_featuresData.append(fd)
        hog_features = np.array(hog_featuresData, 'float64')
        return np.float32(hog_features)


    def get_digits(self,contours):
        digit_rects = [cv2.boundingRect(ctr) for ctr in contours]   
        rects_final = digit_rects[:]

        for r in digit_rects:
            x,y,w,h = r
            if w < 50 and h < 50 :        #too small, remove it
                rects_final.remove(r)
            if w > 100 or h>100 :
                rects_final.remove(r)    
    
        for r1 in digit_rects:
            for r2 in digit_rects:
                if (r1[1] != 1 and r1[1] != 1) and (r2[1] != 1 and r2[1] != 1):  #if the rectangle is not the page-bounding rectangle,
                    if self.contains(r1,r2) and (r2 in rects_final):
                        rects_final.remove(r2)
        return rects_final
    def get_data(self):
        return self.save


    def proc_user_img(self,fn,model):
    #print 'loading "%s for digit recognition" ...' % fn
        im = fn
        im_original = fn
        blank_image = np.zeros((im.shape[0],im.shape[1],3), np.uint8)
        blank_image.fill(255)
        imgray = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
        kernel = np.ones((5,5),np.uint8)
        #ret,thresh = cv2.threshold(imgray,127,255,0)
        ret,thresh = cv2.threshold(imgray,50,255,0)
        cv2.imwrite('thresh022.png', thresh)       
        thresh = cv2.erode(thresh,kernel,iterations = 1)
        thresh = cv2.dilate(thresh,kernel,iterations = 1)
        thresh = cv2.erode(thresh,kernel,iterations = 1)
        #for opencv 3.0.x
        #_,contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        #for opencv 2.4.x
        contours,hierarchy = cv2.findContours(thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        digits_rect = self.get_digits(contours)  #rectangles of bounding the digits in user image
    
        for rect in digits_rect:
            x,y,w,h = rect
            _ = cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)
            im_digit = im_original[y:y+h,x:x+w]
            sz = 28
            im_digit = imresize(im_digit,(sz,sz))
            for i in range(sz):        #need to remove border pixels
                    im_digit[i,0] = 255
                    im_digit[i,1] = 255
                    im_digit[0,i] = 255
                    im_digit[1,i] = 255
            
            thresh = 210
            im_digit = cv2.cvtColor(im_digit,cv2.COLOR_BGR2GRAY)
            im_digit = cv2.threshold(im_digit, thresh, 255, cv2.THRESH_BINARY)[1]
            #im_digit = cv2.adaptiveThreshold(im_digit,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C ,cv2.THRESH_BINARY,11,2)
            im_digit = (255-im_digit)
            #print "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
            #print im_digit
            im_digit = imresize(im_digit,(20,20))
            #print "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
            #print im_digit
            hog_img_data = self.pixels_to_hog_20([im_digit])  
            pred = model.predict(hog_img_data)
            _ = cv2.putText(im, str(int(pred[0])), (x,y),cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 0, 0), 3)
            _ = cv2.putText(blank_image, str(int(pred[0])), (x,y),cv2.FONT_HERSHEY_SIMPLEX, 3, (255, 0, 0), 5)
        print im
        cv2.imwrite('original_overlay_001.png', im)
        cv2.imshow('final_digits_001.png', blank_image)
        cv2.destroyAllWindows()            
def ros_to_np_img(ros_img_msg):
  return np.array(bridge.imgmsg_to_cv2(ros_img_msg,'bgr8'))

def main(args):
    print __doc__
    rospy.wait_for_service('last_image')
    last_image_service = rospy.ServiceProxy('last_image', ImageSrv)
    ic= digit_processing()
    img_data=last_image_service().image_data

    USER_IMG=ros_to_np_img(img_data)
    count=0
    resize = cv2.resize(USER_IMG, (640, 480)) 
    cv2.imwrite("%03d.jpg" % count, resize)
    TRAIN_DATA_IMG = args[1]    
    #USER_IMG = sys.argv[2] 
    digits, labels = ic.load_digits(TRAIN_DATA_IMG)
    print labels
    print 'training ....'
    # shuffle digits
    rand = np.random.RandomState(123)
    shuffle_index = rand.permutation(len(digits))
    digits, labels = digits[shuffle_index], labels[shuffle_index]
    train_digits_data = ic.pixels_to_hog_20(digits)
    train_digits_labels = labels
    print 'training KNearest...'  #gets 80% in most user images
    model = KNN_MODEL(k = 4)
    model.train(train_digits_data, train_digits_labels)
    ic.proc_user_img(USER_IMG,model)
    rospy.init_node('digits_converter', anonymous=True)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)