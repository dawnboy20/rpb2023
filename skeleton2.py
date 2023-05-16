import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header


class DetermineColor:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
        self.bridge = CvBridge()
        self.count = 0
        self.diff = []
        self.val_tmp = []
        self.pub_msg = '0'


    def callback(self,data):
        try:
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            oth, blue, red = self.calculate_color(image)
            cv2.imshow('Image', image)
            cv2.waitKey(1)

            #prepare rotate_cmd msg
            msg = Header()
            msg = data.header
            msg.frame_id = '0'
            self.determine_str(oth, blue, red)

            msg.frame_id = self.pub_msg
            # publish color_state
            self.color_pub.publish(msg)

        except CvBridgeError as e:
            print(e)
    
    def calculate_color(self, image):
        imval = []
        diff_tmp = []

        height, width, col = image.shape
        oth = 0
        blue = 0
        red = 0

        if len(self.val_tmp) == 0:
            for i in range(0, height, 10):
                arr_tmp = []
                for j in range(0, width, 10):
                    arr_tmp.append(list(image[i,j]))
                imval.append(arr_tmp)
            self.val_tmp = imval
        
        else:
            for i in range(0, height, 10):
                arr_tmp = []
                for j in range(0, width, 10):
                    tmp = list(image[i,j])
                    sub_tmp = list(self.val_tmp[int(i/10)][int(j/10)])
                    
                    if (abs(int(tmp[0]) - int(sub_tmp[0])) + abs(int(tmp[1]) - int(sub_tmp[1])) + abs(int(tmp[2]) - int(sub_tmp[2]))) >= 200:
                        diff_tmp.append([i, j])

                    arr_tmp.append(tmp)
                imval.append(arr_tmp)

            if len(diff_tmp) > len(self.diff):
                self.diff = diff_tmp
            for arr in self.diff:
                tmp = list(image[arr[0], arr[1]])
                if tmp[0] >= 200 and tmp[1] >= 200 and tmp[2] >= 200:
                    oth += 1
                elif tmp[2] >= 200 and tmp[0] < 200 and tmp[1] < 200:
                    red += 1
                elif tmp[0] >= 200 and tmp[1] < 200 and tmp[2] < 200:
                    blue += 1
                else:
                    if tmp[0] < 200 and tmp[1] < 200 and tmp[2] < 200:
                        if tmp[0] > 100 and tmp[0] > tmp[2]:
                            blue += 1
                        else:
                            oth += 1
                    else:
                        oth += 1
        return oth, blue, red
    
    def determine_str(self, oth, blue, red):
    	if len(self.diff) > 10:
    		max_num = max([blue, red, oth])
    		if max_num == blue:
    			self.pub_msg = '1'
    		elif max_num == red:
    			self.pub_msg = '-1'
    		else:
    			self.pub_msg = '0'
    	print(self.pub_msg)

    
    def rospy_shutdown(self, signal, frame):
        rospy.signal_shutdown("shut down")
        sys.exit(0)

if __name__ == '__main__':
    detector = DetermineColor()
    rospy.init_node('CompressedImages1', anonymous = False)
    rospy.spin()
