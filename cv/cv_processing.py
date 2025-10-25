#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time
import cv2
import numpy as np
from cv_bridge import CvBridge


class TelloCVNode(Node):
    def __init__(self):
        super().__init__('tello_cv_node')
        self.get_logger().info("Tello CV 處理節點已啟動...")

        # --- ROS 2 訂閱者與發布者 ---
        # 影像主題 (請根據您的 Tello 驅動修改)
        image_topic = "/image_raw" 
        
        # 發布者 (使用您提供的模板)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/land', 10)

        # 訂閱者
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        
        self.bridge = CvBridge()

        # --- CV 處理相關參數 ---
        # 影像中心 (將在 image_callback 中更新)
        self.center_x = 0
        self.center_y = 0

        # 紅色的 HSV 範圍 (如投影片 3 所示的 trackbar)
        # 注意：紅色在 HSV 中跨越 0 度，所以我們需要兩個範圍
        # ‼️ 可以換顏色
        self.lower_red_1 = np.array([0, 120, 70])
        self.upper_red_1 = np.array([10, 255, 255])
        self.lower_red_2 = np.array([170, 120, 70])
        self.upper_red_2 = np.array([180, 255, 255])

        # --- P 控制器參數 (*** 這些值需要根據實際情況進行調校 ***) ---
        self.P_YAW = -0.002  # 偏航角速度 P 增益
        self.P_ALT = -0.002  # 高度線速度 P 增益
        self.P_FWD = 0.00002 # 前後線速度 P 增益

        self.TARGET_AREA = 60000  # 目標物體在畫面中的期望面積 (像素)
        self.MIN_AREA = 1000      # 忽略小於此面積的輪廓

        self.is_flying = False # 標記是否已起飛

    def takeoff(self):
        """發送起飛命令"""
        self.get_logger().info('發送起飛命令...')
        self.takeoff_pub.publish(Empty())
        self.is_flying = True
        time.sleep(5) # 等待 Tello 完成起飛動作

    def land(self):
        """發送降落命令"""
        self.get_logger().info('發送降落命令...')
        self.land_pub.publish(Empty())
        self.is_flying = False
        time.sleep(3) # 等待 Tello 降落

    def draw_debug_window(self, frame_shape, contour=None, center_x=0, center_y=0):
        """
        建立一個黑色的偵錯(debug)視窗，並在上面繪製偵測到的
        最大輪廓 (紅色填滿) 及其中心點。
        """
        # 建立一個與原始影像相同大小的黑色畫布 (3通道, BGR)
        h, w, _ = frame_shape
        debug_image = np.zeros((h, w, 3), dtype=np.uint8)

        if contour is not None:
            # 畫出輪廓 (紅色, -1 表示填滿)
            # 這對應您第一張投影片 (2.16.28.png) 的程式碼
            cv2.drawContours(debug_image, [contour], -1, (0, 0, 255), -1) 
            
            # 畫出中心點 (綠色圓形)
            cv2.circle(debug_image, (center_x, center_y), 10, (0, 255, 0), 2) 

            # 寫上座標文字
            cv2.putText(debug_image, f"Center: ({center_x}, {center_y})", 
                        (center_x - 50, center_y - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return debug_image

    def image_callback(self, msg):
        """影像串流的回調函數"""
        if not self.is_flying:
            return

        try:
            # 將 ROS Image 訊息轉換為 OpenCV 影像 (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge 轉換失敗: {e}')
            return

        # 獲取影像中心
        h, w, _ = cv_image.shape
        self.center_x = w // 2
        self.center_y = h // 2

        # 處理影像並產生控制命令
        self.process_frame(cv_image)

        # 顯示影像 (可選)
        cv2.imshow("Tello View", cv_image)
        cv2.waitKey(1)

    def process_frame(self, frame):
        """
        核心 CV 處理邏輯
        1. 轉換到 HSV
        2. 建立紅色遮罩
        3. 尋找最大輪廓 (如投影片)
        4. 計算中心點 (如投影片)
        5. 計算並發布 Twist 命令
        """
        # 1. 轉換到 HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. 建立紅色遮罩
        mask1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        mask = mask1 + mask2
        
        # (可選) 顯示遮罩
        # cv2.imshow("Mask", mask) 

        # 3. 尋找輪廓 (如投影片)
        # 使用 RETR_EXTERNAL 只找最外層輪廓
        # 使用 CHAIN_APPROX_SIMPLE 壓縮輪廓點
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        twist_msg = Twist()

        if contours:
            # 4. 尋找最大輪廓 (如投影片 2)
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            # 忽略太小的輪廓 (避免噪聲)
            if area > self.MIN_AREA:
                # 5. 計算質心 (如投影片 2)
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                else:
                    cX, cY = 0, 0

                # 在畫面上繪製輪廓和中心點
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                cv2.circle(frame, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(frame, f"({cX}, {cY})", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # 6. 計算控制命令 (P 控制器)
                
                # 誤差 = 偵測值 - 目標值
                # X 軸誤差 (左右)
                err_x = cX - self.center_x
                # Y 軸誤差 (上下)
                err_y = cY - self.center_y
                # 面積誤差 (前後)
                err_area = self.TARGET_AREA - area

                # 偏航 (Yaw): 根據 X 誤差旋轉
                # err_x > 0 (在右邊) -> 順時針轉 (angular.z < 0)
                twist_msg.angular.z = self.P_YAW * float(err_x)

                # 高度 (Altitude): 根據 Y 誤差升降
                # err_y > 0 (在下面) -> 需下降 (linear.z < 0)
                twist_msg.linear.z = self.P_YAW * float(err_y)

                # 前後 (Forward): 根據面積誤差前進後退
                # err_area > 0 (太小) -> 需前進 (linear.x > 0)
                twist_msg.linear.x = self.P_FWD * float(err_area)
                self.cmd_vel_pub.publish(twist_msg)
                return self.draw_debug_window(frame.shape, c, cX, cY)
        
        # 如果沒有找到輪廓，發布停止命令 (懸停)
        self.cmd_vel_pub.publish(twist_msg)
        return self.draw_debug_window(frame.shape) # 回傳全黑的偵錯影像

def main(args=None):
    rclpy.init(args=args)
    node = TelloCVNode()

    try:
        # 起飛
        node.takeoff()

        """ main entry
        a for loop for image_callback + processing frame
        """
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('接收到 Ctrl+C，準備降落...')
    finally:
        # 確保降落
        node.land()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()