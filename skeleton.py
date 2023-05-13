import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

def screen_detection(img):
    # Load the image

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply edge detection
    edges = cv2.Canny(gray, 100, 200)

    # Find contours
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour that meets the criteria
    largest_contour = None
    largest_area = 0.0
    for cnt in contours:
        x,y,w,h = cv2.boundingRect(cnt)
        aspect_ratio = float(w)/h
        if aspect_ratio > 0.5 and aspect_ratio < 2.0:
            area = cv2.contourArea(cnt)
            if area > largest_area:
                largest_area = area
                largest_contour = cnt

    # Create a black canvas
    canvas = np.zeros_like(img)

    # Draw the largest contour onto the canvas
    if largest_contour is not None:
        cv2.drawContours(canvas, [largest_contour], 0, (255, 255, 255), -1)

    # Mask the original image with the canvas
    masked = cv2.bitwise_and(img, canvas)

    # Show the results
    pub_message = '0'
    bgr=[0,0,0]
    for i in range(80):
        for j in range(100):
            b,g,r=masked[int((i/80)*masked.shape[0]*16/27+masked.shape[0]*5/42),int((j/100)*masked.shape[1]*13/18+masked.shape[1]*1/9)]
            if (b>=180 and b<=255) and (r>=0 and r<=60):
                bgr[0]+=1
            elif (b>=0 and b<=60) and (g>=0 and g<=150) and (r>=190 and r<=255):
                bgr[1]+=1
            else:
                bgr[2]+=1
    if np.argmax(bgr)==0:
        pub_message='1'
    elif np.argmax(bgr)==1:
        pub_message='-1'# red background
    else:
        pub_message='0' # other color background
    return pub_message


class DetermineColor:
  def __init__(self):
    self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
    self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
    self.bridge = CvBridge()
    self.count = 0


  def callback(self,data):
    try:
      # listen image topic
      image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
      cv2.imshow('Image', image)
      cv2.waitKey(1)

      #prepare rotate_cmd msg
      msg = Header()
      msg = data.header
      msg.frame_id = screen_detection(image)

      # determine background color
      # if color_result == "red":


      # publish color_state
      self.color_pub.publish(msg)

    except CvBridgeError as e:
      print(e)
  
  def rospy_shutdown(self, signal, frame):
    rospy.signal_shutdown("shut down")
    sys.exit(0)

if __name__ == '__main__':
  rospy.init_node('CompressedImages1', anonymous = False)
  detector = DetermineColor()
  rospy.spin()

