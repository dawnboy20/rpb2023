import rospy
import numpy as np
import cv2

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

def detect_screen_color(img):
    pub_message = '0'
    bgr=[0,0,0]
    for i in range(80):
        for j in range(100):
            b,g,r=img[int((i/80)*img.shape[0]*16/27+img.shape[0]*5/42),int((j/100)*img.shape[1]*13/18+img.shape[1]*1/9)]
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


def screen_detection(img):
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
    cv2.imshow("Screen Extraction", masked)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    return masked


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

      #prepare rotate_cmd msg
      msg = Header()
      msg = data.header
      msg.frame_id = '0'

      # determine background color
      extracted = screen_detection(image)
      msg.frame_id = detect_screen_color(extracted)
      self.color_pub.publish(msg)
      # if color_result == "red":


      # publish color_state
      cv2.imshow('Image', image)
      cv2.waitKey(1)

    except CvBridgeError as e:
      print(e)
  
  def rospy_shutdown(self, signal, frame):
    rospy.signal_shutdown("shut down")
    sys.exit(0)

if __name__ == '__main__':
  detector = DetermineColor()
  rospy.init_node('CompressedImages1', anonymous = False)
  rospy.spin()

