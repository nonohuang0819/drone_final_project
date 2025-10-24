#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

# -----------------------------------------------------------------------------
# 注意：
# 程式碼片段中顯示了 'self.yaw', 'self.height', 'self.battery', 'self.flight_time'
# 這些變數需要透過訂閱 Tello 驅動節點的「狀態」主題來更新。
#
# 這裡我們假設您使用的 Tello 驅動程式 (如 'tello_ros') 
# 會發布一個 'tello_msgs/msg/TelloStatus' 類型的訊息到 'status' 主題上。
#
# 如果您的驅動程式使用不同的主題或訊息類型（例如，多個獨立的主題
# 如 /battery, /height, /yaw），您需要修改 'status_callback' 函式
# 並增加額外的訂閱者 (Subscriber)。
# -----------------------------------------------------------------------------

# 假設的 TelloStatus 訊息導入。
# 您需要確保您的 ROS 2 工作區中有 'tello_msgs' 這個套件。
try:
    from tello_msgs.msg import TelloStatus
except ImportError:
    print("--------------------------------------------------")
    print("錯誤：無法導入 'tello_msgs.msg.TelloStatus'")
    print("請確認您已經安裝並編譯了 'tello_msgs' 套件。")
    print("若您的 Tello 驅動程式使用不同訊息，請修改此程式碼。")
    print("--------------------------------------------------")
    # 暫時使用一個假的 Class 以便程式能被讀取，但無法執行
    class TelloStatus:
        pass


class SimpleTestNode(Node):
    def __init__(self):
        super().__init__('simple_test')
        self.get_logger().info('RCL Python Tello Test Node 已啟動.')

        # --- 從簡報中推斷出的屬性 ---
        self.yaw = 0.0
        self.height = 0.0
        self.battery = 0
        self.flight_time = 0.0
        
        # 狀態機相關屬性
        self.task_index = 0
        self.task_record = -1.0  # 用於記錄任務開始的時間
        self.init_height = -1.0 # 用於記錄降落前的高度

        # 任務流程 (根據簡報中的 case 0~4)
        # 0: Takeoff
        # 1: Ascend (上升)
        # 2: Move Forward (前進)
        # 3: Move Forward + Rotate (前進並旋轉)
        # 4: Land (降落)
        # 🌟 control tasks
        self.task_flow = [False, True, True, True, True]

        # --- 發布者 (Publisher) ---
        # (Topic name 'control' 來自簡報中的 create_timer 程式碼片段)
        self.takeoff_publisher = self.create_publisher(Empty, 'takeoff', 10)
        self.land_publisher = self.create_publisher(Empty, 'land', 10)
        self.control_publisher = self.create_publisher(Twist, 'control', 10)

        # --- 訂閱者 (Subscriber) ---
        # 訂閱 Tello 狀態
        self.status_subscriber = self.create_subscription(
            TelloStatus,
            'status',
            self.status_callback,
            10)

        # --- 計時器 (Timer) ---
        # (來自簡報中的 create timer 程式碼片段)
        task_time_period = 1.0  # 任務計時器，每 1.0 秒檢查一次
        self.task_timer = self.create_timer(task_time_period, self.task_timer_callback)

        # 雖然簡報中顯示了 control_timer，但在 task_timer_callback 邏輯中並未使用
        # control_timer_period = 0.1
        # self.control_timer = self.create_timer(control_timer_period, self.control_timer_callback)
        # def control_timer_callback(self):
        #    pass # 簡報中未顯示此函式的實作

    def status_callback(self, msg):
        """
        更新 Tello 的狀態。
        注意：這裡的 'msg.' 屬性 (如 yaw_deg) 是基於 'tello_msgs.msg.TelloStatus' 的猜測。
        請根據您實際的訊息定義來調整。
        """
        try:
            self.yaw = msg.yaw_deg
            self.height = msg.height_m
            self.battery = msg.battery_percentage
            self.flight_time = msg.flight_time_sec
        except Exception as e:
            if isinstance(msg, TelloStatus): # 避免在 TelloStatus 導入失敗時重複報錯
                self.get_logger().warn(f"狀態回調錯誤：{e}。您的 TelloStatus 訊息定義可能不同。")

    def task_timer_callback(self):
        """
        主要的任務流程狀態機 (State Machine)。
        此函式完整複製了您簡報中 'task_timer_callback' 內的所有邏輯。
        """
        
        # ### every is to show variable
        print("-----------------------")
        print(f"yaw: {self.yaw}")
        print(f"height: {self.height}")
        print(f"battery: {self.battery}")
        print(f"flight_time: {self.flight_time}")

        # 尋找下一個未完成的任務
        for i in range(len(self.task_flow)):
            if not self.task_flow[i]:
                print(f"[{i}, not finish]")
                self.task_index = i
                break
        else:
            # 如果所有任务都完成了
            self.get_logger().info('所有任務已完成，停止計時器。')
            self.task_timer.cancel()
            return

        # 狀態機
        msg = Twist()

        # --- Case 0: Takeoff (起飛) ---
        # (此邏輯基於簡報中 "takeoff" 和 "case 1" 的片段推斷)
        if self.task_index == 0:
            self.get_logger().info('Case 0: Takeoff')
            # 持續發送起飛指令，直到偵測到高度 > 0.1 (公尺)
            if self.height < 0.1:
                self.takeoff_publisher.publish(Empty())
            else:
                self.get_logger().info('Takeoff complete.')
                self.task_flow[self.task_index] = True
                self.task_record = -1.0 # 重置計時器
            return

        # --- Case 1: Ascend (上升) ---
        # (來自簡報中 case 1 的程式碼片段)
        if self.task_index == 1:
            self.get_logger().info('Case 1: Ascend')
            if self.task_record == -1.0:
                self.task_record = self.flight_time
            
            msg.linear.z = 50.0  # Tello 驅動通常接受 -100 到 100 的值
            self.control_publisher.publish(msg)

            # 上升 1.0 秒
            if self.flight_time - self.task_record > 1.0:
                self.task_flow[self.task_index] = True
                self.task_record = -1.0
            return

        # --- Case 2: Move Forward (前進) ---
        # (來自簡報中 case 2 的模糊程式碼片段)
        if self.task_index == 2:
            self.get_logger().info('Case 2: Move Forward')
            if self.task_record == -1.0:
                self.task_record = self.flight_time
            
            msg.linear.x = 50.0
            self.control_publisher.publish(msg)

            # 前進 1.0 秒
            if self.flight_time - self.task_record > 1.0:
                self.task_flow[self.task_index] = True
                self.task_record = -1.0
            return
            
        # --- Case 3: Move forward + rotate clockwise ---
        # (來自簡報中 case 3 的程式碼片段)
        if self.task_index == 3:
            self.get_logger().info('Case 3: Move forward + rotate clockwise')
            if self.task_record == -1.0:
                self.task_record = self.flight_time
            
            msg = Twist()
            msg.linear.x = 30.0
            msg.angular.z = 30.0
            self.control_publisher.publish(msg)

            # 執行 1.0 秒
            if self.flight_time - self.task_record > 1.0:
                self.task_flow[self.task_index] = True
                self.task_record = -1.0
            return

        # --- Case 4: Land (降落) ---
        # (來自簡報中 case 4 的程式碼片段)
        if self.task_index == 4:
            self.get_logger().info('Case 4: Land')
            if self.task_record == -1.0:
                self.task_record = 1.0 # 設置為 1 進入下一個狀態
                self.init_height = self.height # 記錄開始降落時的高度

            if self.task_record == 1.0:
                self.land_publisher.publish(Empty())
                # 檢查是否已接近地面 (例如：低於初始高度的 20%)
                if self.init_height > 0 and self.height < self.init_height * 0.2:
                    self.get_logger().info('Landing complete.')
                    self.task_flow[self.task_index] = True
                    self.task_record = -1.0
            return

        # --- 其他 Cases (5, 6, 7...) ---
        # 簡報中顯示了 case 5, 6, 7 但沒有實作，這裡省略
        if self.task_index > 4:
            self.get_logger().info('All tasks finished.')
            self.task_timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("節點已關閉。")
    finally:
        # 在節點關閉前發送最後的降落指令 (安全起見)
        node.get_logger().info('Sending final land command before shutdown...')
        node.land_publisher.publish(Empty())
        time.sleep(1) # 給予指令發送的時間
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()