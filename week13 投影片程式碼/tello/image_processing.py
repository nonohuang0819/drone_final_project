### import client library: rclpy, and Node object
import rclpy
from rclpy.node import Node

### from std_msgs's msg import interface Float64MultiArray
from std_msgs.msg import Float64MultiArray

### from sensor_msgs's msg import interface Image
from sensor_msgs.msg import Image

### using cvbridge, cv2, numpy to process image
from cv_bridge import CvBridge
from datetime import datetime
import cv2
import numpy as np

H_low = 0
H_high = 179
S_low= 140
S_high = 255
V_low= 50
V_high = 190

### create class IP_Node, inherits from Node
class IP_Node(Node):

    def __init__(self):
        super().__init__('tello_Image_Processing')
        
        ### parameter inint
        self.bridge = CvBridge()
        self.img = np.array([])
        self.start_pub = True
        
        ### create publisher
        self.inform_publisher = self.create_publisher(Float64MultiArray, "/ip_inform", 1)
        
        ### create subscriber
        self.img_subscriber = self.create_subscription(Image, "/camera", self.img_callback, 1)
        
        ### video writer
        title = datetime.now().strftime("%Y-%m-%d_%H_%M_%S")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = cv2.VideoWriter(title+".mp4", fourcc, 20, (1920, 720))
           
    def img_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            m = self.findMask(self.img)
            center, max_c, r_data = self.findCenter(m)
            print("center: ", center)
            show_image = cv2.cvtColor(np.zeros(self.img.shape[:2], dtype=np.uint8), cv2.COLOR_GRAY2BGR)
            
            if len(max_c) != 0:
              cv2.drawContours(show_image, [max_c], -1, (0,0,255), -1)
              cv2.circle(show_image, (int(center[0]), int(center[1])), 10, (0, 50, 175), -1)
              cv2.putText(show_image, str((r_data[2]*r_data[3])/(960*720.)), (10, 40), 5, 2, (255, 255, 0))
              
              if self.start_pub:
                if (r_data[2]*r_data[3]) / (960*720.) >= 0.25:
                  self.inform_publisher.publish(Float64MultiArray(data = [center[0], center[1], 1]))
                  #self.start_pub = False
                else:
                  self.inform_publisher.publish(Float64MultiArray(data = [center[0], center[1], 0]))                  
            combine_img = np.concatenate((self.img, show_image), axis = 1)
            self.out.write(combine_img)
            cv2.imshow("result", cv2.resize(combine_img, (640, 240)))
            cv2.waitKey(1)
        except Exception as e:
            print(e)
    
    def findMask(self, img):
      #convert sourece image BGR to HSV color mode
      hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
      
      # create HSV low array H_low, S_low, V_low  
      hsv_low = np.array([H_low, S_low, V_low], np.uint8)
      # create HSV high array H_high, S_high, V_high 
      hsv_high = np.array([H_high, S_high, V_high], np.uint8)
      
      #making mask for hsv range
      mask = cv2.inRange(hsv, hsv_low, hsv_high)
      return mask
    
    def findCenter(self, mask):
      center = [-1, -1]
      max_c = []
      x, y, w, h = -1, -1, -1, -1
      c_c, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      if len(c_c) != 0:
        max_c = max(c_c, key=cv2.contourArea)
        
        # Approximate
        x, y, w, h = cv2.boundingRect(max_c)
        center = (x+w*0.5, y+h*0.5)
      return center, max_c, [x, y, w, h]
         
### main function
def main(args=None):
    
    ### rclpy library is initialized
    rclpy.init(args=args)
    
    ### create IP_Node
    ipn = IP_Node()
    
    ### spin the to node, so its callback are called 
    rclpy.spin(ipn)
    
    ### after ctrl-c or the other things to close node
    ipn.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()  

