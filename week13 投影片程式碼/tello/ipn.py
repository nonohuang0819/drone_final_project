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

# HSV 顏色參數 (紅色/橘色框框)
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
        
        ### AprilTag Init (新增部分)
        # 使用 36h11 字典
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.aruco_params = cv2.aruco.DetectorParameters()
        
        ### create publisher
        self.inform_publisher = self.create_publisher(Float64MultiArray, "/ip_inform", 1)
        
        ### create subscriber
        self.img_subscriber = self.create_subscription(Image, "/camera", self.img_callback, 1)
        
        ### video writer
        title = datetime.now().strftime("%Y-%m-%d_%H_%M_%S")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.out = cv2.VideoWriter(title+".mp4", fourcc, 20, (1920, 720)) # 左右拼接後的解析度
            
    def img_callback(self, msg):
        try:
            # 1. 取得影像
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8") # 建議用 bgr8 比較符合 OpenCV 預設
            
            # 初始化顯示用的圖
            show_image = np.zeros(self.img.shape, dtype=np.uint8)
            
            # 2. 偵測 AprilTag (優先權最高)
            gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            tag_detected = False
            tag_center = [0.0, 0.0]

            if ids is not None:
                for i in range(len(ids)):
                    if ids[i][0] == 0: # 偵測 ID 為 0 的 Tag
                        tag_detected = True
                        c = corners[i][0]
                        # 計算中心點
                        cx = (c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4
                        cy = (c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4
                        tag_center = [cx, cy]
                        
                        # 畫出 Tag 框框
                        cv2.aruco.drawDetectedMarkers(show_image, corners)
                        cv2.circle(show_image, (int(cx), int(cy)), 10, (0, 255, 0), -1) # 綠點
                        cv2.putText(show_image, "Tag detected!", (10, 40), 5, 2, (0, 255, 0))
                        
                        # === 發布訊號：狀態 2 (代表降落) ===
                        if self.start_pub:
                             # data format: [cx, cy, status] -> status 2 means LAND
                            self.inform_publisher.publish(Float64MultiArray(data = [cx, cy, 2.0]))
                        break
            
            # 3. 如果沒看到 Tag，才去偵測顏色框框 (Hoop)
            if not tag_detected:
                m = self.findMask(self.img)
                center, max_c, r_data = self.findCenter(m)
                # print("center: ", center)
                
                if len(max_c) != 0:
                    cv2.drawContours(show_image, [max_c], -1, (0,0,255), -1) # 紅色輪廓
                    cv2.circle(show_image, (int(center[0]), int(center[1])), 10, (0, 50, 175), -1)
                    
                    # 計算面積比例
                    area_ratio = (r_data[2]*r_data[3])/(960*720.)
                    cv2.putText(show_image, f"Area: {area_ratio:.2f}", (10, 40), 5, 2, (255, 255, 0))
                    
                    if self.start_pub:
                        # === 發布訊號 ===
                        if area_ratio >= 0.25:
                            # 狀態 1：夠近了，可以衝過去
                            self.inform_publisher.publish(Float64MultiArray(data = [center[0], center[1], 1.0]))
                        else:
                            # 狀態 0：還太遠，繼續對準
                            self.inform_publisher.publish(Float64MultiArray(data = [center[0], center[1], 0.0]))                  
            
            # 4. 顯示與錄影
            # 確保兩張圖大小一致才能接起來
            if show_image.shape != self.img.shape:
                 show_image = cv2.resize(show_image, (self.img.shape[1], self.img.shape[0]))

            combine_img = np.concatenate((self.img, show_image), axis = 1)
            self.out.write(combine_img)
            cv2.imshow("result", cv2.resize(combine_img, (640, 240)))
            cv2.waitKey(1)
            
        except Exception as e:
            print(f"Error: {e}")
            import traceback
            traceback.print_exc()
    
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