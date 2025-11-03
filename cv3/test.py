#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
import time
import cv2
import numpy as np
from cv_bridge import CvBridge

class TelloCVNode(Node):
    def __init__(self):
        super().__init__('tello_cv_node')
        self.get_logger().info("Tello CV 狀態機節點已啟動...")

        image_topic = "/camera" 
        
        # --- ROS 介面 (來自 TelloCVNode) ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/control', 10)
        self.takeoff_pub = self.create_publisher(Empty, '/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/land', 10)
        self.image_processed_pub = self.create_publisher(Image, '/image_processed', 10)
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10)
        
        self.bridge = CvBridge()

        # --- CV 偵測參數 (來自 TelloCVNode) ---
        self.center_x = 0
        self.center_y = 0
        self.frame_width = 0
        self.frame_height = 0
        self.lower_red_1 = np.array([0, 120, 70])
        self.upper_red_1 = np.array([10, 255, 255])
        self.lower_red_2 = np.array([170, 120, 70])
        self.upper_red_2 = np.array([180, 255, 255])
        self.MIN_AREA = 500

        # --- 狀態機變數 (來自圖片) ---
        # 訂閱 /ip_inform 以獲取 CV 偵測結果
        self.inform_pub = self.create_publisher(Float64MultiArray, '/ip_inform', 10)
        self.inform_sub = self.create_subscription(Float64MultiArray, '/ip_inform', self.inform_callback, 10)
        
        self.cx, self.cy, self.can_pass = -1, -1, 0
        self.check = False # 來自圖片 1 的邏輯
        
        # 任務流程: 0=起飛, 1=對準, 1.5=穿過, 2=降落, 3=結束
        self.task_index = 0
        self.task_flow = [False, False, False, False]
        
        # 用於 case 1.5 (穿過) 的計時器
        self.pass_start_time = None 
        
        # 狀態機控制迴圈 (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_timer_callback)
        self.get_logger().info('控制迴圈已啟動...')

    def inform_callback(self, msg: Float64MultiArray):
        try:
            self.cx = int(msg.data[0])
            self.cy = int(msg.data[1])
            self.can_pass = int(msg.data[2])
        except Exception:
            self.cx, self.cy, self.can_pass = -1, -1, 0

    def image_callback(self, msg):
        # 1. 轉換影像
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge 轉換失敗: {e}')
            return

        # 2. 初始化影像中心點 (只執行一次)
        if self.frame_width == 0:
            h, w, _ = cv_image.shape
            self.frame_width = w
            self.frame_height = h
            self.center_x = w // 2
            self.center_y = h // 2
            self.get_logger().info(f'影像中心點初始化: ({self.center_x}, {self.center_y})')

        # 3. 處理影像 (偵測紅球並發布 /ip_inform)
        processed_image = self.process_frame(cv_image)

        # 4. 發布處理過的影像 (用於偵錯)
        try:
            processed_msg = self.bridge.cv2_to_imgmsg(processed_image, encoding='bgr8')
            self.image_processed_pub.publish(processed_msg)
        except Exception as e:
            self.get_logger().error(f'發布處理影像失敗: {e}')

    def process_frame(self, frame):
        # 偵測紅色物體 (來自 TelloCVNode)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv, self.lower_red_2, self.upper_red_2)
        mask = mask1 + mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cX, cY, can_pass_val = -1.0, -1.0, 0.0

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)

            if area > self.MIN_AREA:
                M = cv2.moments(c)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["00"])
                else:
                    cX, cY = 0, 0
                
                # *** 核心變更 ***
                # 計算誤差 (dx, dy)
                dx = cX - self.center_x
                dy = cY - self.center_y
                
                # (來自圖片 1) 檢查是否對準
                if abs(dx) < 24 and abs(dy) < 24:
                    can_pass_val = 1.0

                # 繪製
                cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
                cv2.circle(frame, (int(cX), int(cY)), 7, (255, 255, 255), -1)

        # *** 核心變更 ***
        # 發布偵測結果給 control_timer_callback
        info = Float64MultiArray()
        info.data = [float(cX), float(cY), can_pass_val]
        self.inform_pub.publish(info)
        
        # 繪製中心點
        cv2.line(frame, (self.center_x, 0), (self.center_x, self.frame_height), (0, 0, 255), 1)
        cv2.line(frame, (0, self.center_y), (self.frame_width, self.center_y), (0, 0, 255), 1)

        return frame

    def control_timer_callback(self):

        # --- case 0: 起飛 ---
        if self.task_index == 0:
            if not self.task_flow[0]:
                self.get_logger().info('Case 0: 起飛')
                self.takeoff_pub.publish(Empty())
                self.task_flow[0] = True
                self.task_index = 1 # 進入下一階段
            return

        # --- case 1: 對準與穿過 ---
        if self.task_index == 1:
            if self.frame_width == 0:
                self.get_logger().info('Case 1: 等待影像...')
                return
            
            if self.cx == -1:
                self.get_logger().info('Case 1: 未偵測到物體，懸停 (可在此加入搜索)')
                self.cmd_vel_pub.publish(Twist()) # 懸停
                return
            
            if self.can_pass == 1:
                self.get_logger().info('Case 1: 已對準 (can_pass=1)! 準備穿過...')
                self.pass_start_time = self.get_clock().now() # 記錄開始時間
                self.task_index = 1.5 # 進入 "穿過" 狀態
                
                msg = Twist()
                # (修正自圖片 2: y=50.0 -> x=0.5)
                msg.linear.x = 20.0 # 向前
                self.cmd_vel_pub.publish(msg)
                return
            
            dx = self.cx - self.center_x
            dy = self.cy - self.center_y
            msg = Twist()
            
            if not self.check:
                if abs(dx) < 24 and abs(dy) < 24: # (應與 can_pass 同步)
                    self.check = True
                    msg.linear.x = 0.2 # 緩慢前進
                else:
                    # Tello (ROS 1) cmd_vel: linear.y 是左右平移, angular.z 是旋轉
                    # Tello (ROS 2, tello_driver): linear.y 是左右, linear.z 是上下, angular.z 是旋轉
                    # 圖片邏輯 linear.x = 10.0, 猜測是左右平移
                    msg.linear.y = np.sign(dx) * 0.15 # 左右平移
                    msg.linear.z = -1 * np.sign(dy) * 0.15 # 上下
            else: # (self.check == True)
                if abs(dx) > 60 or abs(dy) > 60:
                    self.check = False
                    msg.linear.y = np.sign(dx) * 0.15 # 左右
                    msg.linear.z = -1 * np.sign(dy) * 0.15 # 上下
                else:
                    # (修正自圖片 1: y=20.0 -> x=0.2)
                    msg.linear.x = 0.2 # 緩慢前進

            self.get_logger().info(f'Case 1: 對準中... dx:{dx}, dy:{dy}, check:{self.check}')
            self.cmd_vel_pub.publish(msg)
            return

        # --- case 1.5: 正在穿過 (修復 time.sleep(3)) ---
        if self.task_index == 1.5:
            duration = (self.get_clock().now() - self.pass_start_time).nanoseconds / 1e9
            if duration < 3.0: # (來自圖片 2 的 3 秒)
                self.get_logger().info(f'Case 1.5: 穿過中... {duration:.1f}s')
                msg = Twist()
                msg.linear.x = 0.5 # (修正自 y=50.0)
                self.cmd_vel_pub.publish(msg)
            else:
                self.get_logger().info('Case 1.5: 穿過完成。')
                self.cmd_vel_pub.publish(Twist()) # 停止
                self.task_flow[1] = True
                self.task_index = 2 # 進入降落
            return

        # --- case 2: 降落 ---
        if self.task_index == 2:
            if not self.task_flow[2]:
                self.get_logger().info('Case 2: 降落')
                self.land_pub.publish(Empty())
                self.task_flow[2] = True
                self.task_index = 3
            return

        # --- case 3: 結束 (懸停) ---
        if self.task_index == 3:
            if not self.task_flow[3]:
                self.get_logger().info('Case 3: 任務結束，懸停。')
                self.cmd_vel_pub.publish(Twist())
                self.task_flow[3] = True
            return

def main(args=None):
    rclpy.init(args=args)
    node = TelloCVNode()

    try:
        # *** 變更 ***
        # 不再呼叫 node.takeoff()，
        # 而是讓 control_timer_callback 處理 case 0
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('偵測到 Ctrl+C，準備降落...')
    finally:
        node.get_logger().info('執行最終降落...')
        # 確保發送降落指令
        node.land_pub.publish(Empty()) 
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()