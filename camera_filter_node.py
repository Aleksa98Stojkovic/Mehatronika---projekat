#!/usr/bin/env python3.8

from cv2 import normalize
from numpy.core.defchararray import array
import rospy
from sensor_msgs.msg import Image   
#from camera_filter import camera_filter_src
import std_msgs.msg as msg 
from std_msgs.msg import String
import os
import time
#!/usr/bin/env python3.8


from cv_bridge.core import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
import cv2 
import cv_bridge as cvb
import numpy as np
from numpy.lib.stride_tricks import as_strided
import std_msgs.msg as msg 
import sys

pub = rospy.Publisher('directions1', String, queue_size=10) 


class Camerafilter:


    def __init__(self):
        self.cvb = cvb.CvBridge()


    def filter_image(self, data):

        try: 
            cv_image = self.cvb.imgmsg_to_cv2(data, desired_encoding='passthrough') ##prebacujemo sliku iz poruke u openCV
        except CvBridgeError as e:
            print(e)

        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) ##prebacivanje slike u HSV format
        
        lowerYellow = np.array([25, 50, 50]) 
        upperYellow = np.array([30, 250, 250]) 

        lowerRed = np.array([0, 50, 50 ]) 
        upperRed = np.array([15, 250, 250]) 

        lowerGreen = np.array([50, 50, 50 ]) 
        upperGreen = np.array([60, 250, 250]) 

        lowerPurple = np.array([130, 50, 50])
        upperPurple = np.array([150, 250, 250])

        maskYellow = cv2.inRange(hsv_image, lowerYellow, upperYellow) 
        resultYellow = cv2.bitwise_and(cv_image, cv_image, mask=maskYellow) 
        
        maskRed = cv2.inRange(hsv_image, lowerRed, upperRed) 
        resultRed = cv2.bitwise_and(cv_image, cv_image, mask=maskRed) 

        maskGreen = cv2.inRange(hsv_image, lowerGreen, upperGreen) 
        resultGreen = cv2.bitwise_and(cv_image, cv_image, mask=maskGreen) 

        maskPurple = cv2.inRange(hsv_image, lowerPurple, upperPurple)
        resultPurple = cv2.bitwise_and(cv_image, cv_image, mask = maskPurple)

        # IZDVAJANJE BOJA
        maskYellow1 = self.pool2d(maskYellow, 20, 20, 0, 'max')
        maskRed1 = self.pool2d(maskRed, 20, 20, 0, 'max')
        maskGreen1 = self.pool2d(maskGreen, 20, 20, 0, 'max')
        maskPurple1 = self.pool2d(maskPurple, 20, 20, 0, 'max')

        cv2.imshow('obstacles', resultYellow)
        cv2.imshow('start' , resultGreen)
        cv2.imshow('finish', resultRed)
        cv2.imshow('object', resultPurple)

        n1 = -(1/255)
        maskYellow1 = n1 * maskYellow1
        n2 = (1/255)
        maskRed1 = n2 * maskRed1
        n3 = (281/255)
        maskGreen1 = n3 * maskGreen1
        n4 = (4/255)
        maskPurple1 = n4*maskPurple1
        
        matrix = maskYellow1 + maskRed1 + maskGreen1 + maskPurple1   
        print("Matrix generated")

        matrix[matrix == 1] = '3' #finish
        matrix[matrix == -1] = '1' #prepreka
        matrix[matrix == 281] = '2' #start
        matrix[matrix == 282] = '2' #start
        matrix[matrix == 280] = '2' #start
        matrix[matrix == 4] = '4' #object

        for i in range(0, 29, 1):
            for j in range(0, 43, 1):
                if(matrix[i][j] == 1 and matrix[i][j - 1] == 0 and i > 0 and j > 0):
                    matrix[i][j - 1] = '1'

                    break
        for i in range(0, 29, 1):
            for j in range(0, 43, 1):
                if(matrix[i][j] == 1 and matrix[i][j + 1] == 0 and i > 0 and j > 0):
                    matrix[i][j + 1] = '1'

                    break
        for j in range(0, 43, 1):
            for i in range(0, 29, 1):
                if(matrix[i][j] == 1 and matrix[i - 1][j] == 0 and i > 0 and j > 0):
                    matrix[i - 1][j] = '1'

                    break            
        
        for j in range(0, 43, 1):
            for i in range(0, 29, 1):
                if(matrix[i][j] == 1 and matrix[i + 1][j] == 0 and i > 0 and j > 0):
                    matrix[i + 1][j] = '1'

                    break
        
        np.savetxt('/home/milos/meh_ws/src/camera_filter/nodes/matrix.txt', matrix, fmt = '%.2d')
        os.system("python3 /home/milos/meh_ws/src/camera_filter/nodes/path_finder.py")
        print("Path generated")
        rospy.signal_shutdown("task_done")
        cv2.waitKey(10)

    def pool2d(self, A, kernel_size, stride, padding, pool_mode):
        '''
        2D Pooling

        Parameters:
            A: input 2D array
            kernel_size: int, the size of the window
            stride: int, the stride of the window
            padding: int, implicit zero paddings on both sides of the input
            pool_mode: string, 'max' or 'avg'
        '''
         # Padding
        A = np.pad(A, padding, mode='constant')

        # Window view of A
        output_shape = ((A.shape[0] - kernel_size)//stride + 1,
                        (A.shape[1] - kernel_size)//stride + 1)
        kernel_size = (kernel_size, kernel_size)
        A_w = as_strided(A, shape = output_shape + kernel_size, 
                        strides = (stride*A.strides[0],
                                   stride*A.strides[1]) + A.strides)
        A_w = A_w.reshape(-1, *kernel_size)

        # Return the result of pooling
        if pool_mode == 'max':
            return A_w.max(axis=(1,2)).reshape(output_shape)
        elif pool_mode == 'avg':
            return A_w.mean(axis=(1,2)).reshape(output_shape)



class CameraNode: 

    def __init__(self):


        rospy.init_node('camera_filter_node')
        rospy.loginfo("Starting 'camera_filter_node'")
        
        self.camera_sub = rospy.Subscriber("/webots_camera", Image,  self.callback_camera)
        self.cf_class = Camerafilter()


    def callback_camera(self, Image_data):
    
        self.cf_class.filter_image(Image_data)
        
       

    def run(self):
        rospy.spin()   



if __name__ == "__main__":

    node = CameraNode()
    node.run()
    
    
