#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

# --- 假設 Tello 驅動程式的訊息類型 ---
# 再次提醒：您需要有 'tello_msgs' 套件，
# 否則請將 'TelloStatus' 替換為您驅動程式實際使用的訊息類型。
try:
    from tello_msgs.msg import TelloStatus
    STATUS_MSG_TYPE = TelloStatus
    BATTERY_FIELD = 'battery_percentage'
except ImportError:
    print("警告：無法導入 'tello_msgs.msg.TelloStatus'。")
    print("將嘗試使用 'sensor_msgs.msg.BatteryState'...")
    try:
        from sensor_msgs.msg import BatteryState
        STATUS_MSG_TYPE = BatteryState
        BATTERY_FIELD = 'percentage'
    except ImportError:
        print("錯誤：也找不到 'sensor_msgs.msg.BatteryState'。")
        print("請檢查您的 Tello 驅動程式並修改此腳本的 STATUS_MSG_TYPE。")
        exit(1)
# ---

class PreFlightCheckNode(Node):
    def __init__(self):
        super().__init__('tello_preflight_check')
        self.get_logger().info("Tello 飛行前檢查節點已啟動...")

        # --- Tello 驅動程式的主題名稱 (請根據您的驅動進行修改) ---
        status_topic = 'status' # 用於讀取電池
        control_topic = 'control'  # 用於發布 Twist (速度)
        takeoff_topic = 'takeoff'
        land_topic = 'land'
        # ---

        # 狀態變數
        self.battery = 0
        self.battery_check_ok = False
        
        # 訂閱者
        self.status_sub = self.create_subscription(
            STATUS_MSG_TYPE,
            status_topic,
            self.status_callback,
            10)
            
        # 發布者
        self.control_publisher = self.create_publisher(Twist, control_topic, 10)
        self.takeoff_publisher = self.create_publisher(Empty, takeoff_topic, 10)
        self.land_publisher = self.create_publisher(Empty, land_topic, 10)

    def status_callback(self, msg):
        """從 Tello 狀態訊息中讀取電池電量"""
        try:
            # 根據 STATUS_MSG_TYPE 讀取正確的欄位
            if hasattr(msg, BATTERY_FIELD):
                if STATUS_MSG_TYPE == TelloStatus:
                    self.battery = msg.battery_percentage
                else:
                    self.battery = int(msg.percentage * 100) # BatteryState 是 0.0-1.0
                
                if not self.battery_check_ok: # 只顯示一次
                    self.get_logger().info(f"成功接收到 Tello 狀態：電量 {self.battery}%")
                    self.battery_check_ok = True
            else:
                self.get_logger().warn(f"狀態訊息中沒有 '{BATTERY_FIELD}' 欄位。")
                
        except Exception as e:
            self.get_logger().error(f"狀態回調出錯: {e}")

    def run_check(self):
        """
        執行一系列的檢查動作
        """
        self.get_logger().info("--- [步驟 1/6] 檢查 Tello 連線與電池 ---")
        
        # 等待 status_callback 至少成功執行一次
        wait_cycles = 0
        while not self.battery_check_ok and rclpy.ok() and wait_cycles < 10:
            self.get_logger().info("等待 Tello 狀態訊息...")
            rclpy.spin_once(self, timeout_sec=1.0) # 處理回調
            wait_cycles += 1
            
        if not self.battery_check_ok:
            self.get_logger().error("檢查失敗：10 秒內未收到 Tello 狀態訊息。")
            self.get_logger().error("請確認 Tello 驅動節點 ('tello/node.py') 正在運行。")
            return

        if self.battery < 20:
            self.get_logger().error(f"檢查失敗：電池電量過低 ({self.battery}%)。請充電。")
            return
            
        self.get_logger().info(f"電池電量 {self.battery}%，檢查通過。")

        # 建立一個全 0 的 Twist 訊息 (懸停)
        hover_msg = Twist()
        
        # 建立一個測試用的 Twist 訊息
        # (速度值 30 是基於您 'simple_test.py' 的範例)
        move_msg = Twist()

        try:
            # --- 測試序列 ---
            self.get_logger().info("--- [步驟 2/6] 測試：起飛 ---")
            self.takeoff_publisher.publish(Empty())
            time.sleep(5) # 等待起飛完成

            self.get_logger().info("--- [步驟 3/6] 測試：懸停 (3 秒) ---")
            self.control_publisher.publish(hover_msg)
            time.sleep(3)

            self.get_logger().info("--- [步驟 4/6] 測試：向前 (1 秒) ---")
            move_msg.linear.x = 30.0 # 向前 (使用 30% 功率)
            self.control_publisher.publish(move_msg)
            time.sleep(1)
            move_msg.linear.x = 0.0 # 停止

            self.get_logger().info("--- [步驟 5/6] 測試：向右旋轉 (1 秒) ---")
            move_msg.angular.z = 40.0 # 向右旋轉 (使用 40% 功率)
            self.control_publisher.publish(move_msg)
            time.sleep(1)
            move_msg.angular.z = 0.0 # 停止

            self.get_logger().info("--- [步驟 6/6] 測試：降落 ---")
            # 發布懸停指令，然後降落
            self.control_publisher.publish(hover_msg)
            time.sleep(1)
            self.land_publisher.publish(Empty())
            time.sleep(3) # 等待降落完成

            self.get_logger().info("=== 飛行前檢查完畢，所有功能正常！ ===")

        except Exception as e:
            self.get_logger().error(f"測試過程中斷：{e}")
            self.get_logger().error("將觸發緊急降落。")

def main(args=None):
    rclpy.init(args=args)
    node = PreFlightCheckNode()
    
    try:
        # 執行檢查
        node.run_check()
        
    except KeyboardInterrupt:
        node.get_logger().warn("測試被使用者中斷！")
    finally:
        # --- 安全降落機制 ---
        # 無論測試成功、失敗或被中斷，都必須發送降落指令
        node.get_logger().info("--- [安全機制] 發送最終降落指令 ---")
        node.land_publisher.publish(Empty())
        time.sleep(2) # 給指令一點時間發送
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()