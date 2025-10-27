#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
import cv2
import numpy as np

class TelloController(Node):
    def __init__(self):
        super().__init__('tello_controller')

        # --- publishers & subscribers (修改 topic 名稱以符合你的系統) ---
        self.img_subscriber = self.create_subscription(Image, '/camera', self.img_callback, 1)
        self.takeoff_pub = self.create_publisher(Empty, '/tello/takeoff', 10)
        self.land_pub = self.create_publisher(Empty, '/tello/land', 10)
        self.control_pub = self.create_publisher(Twist, '/tello/cmd_vel', 10)

        # Image subscriber
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/tello/camera/image_raw', self.image_callback, 10)

        # --- timers ---
        self.task_timer_period = 1.0  # 每秒檢查任務流程
        self.control_timer_period = 0.1  # 控制頻率
        self.task_timer = self.create_timer(self.task_timer_period, self.task_timer_callback)
        self.control_timer = self.create_timer(self.control_timer_period, self.control_timer_callback)

        # --- control state ---
        self.current_twist = Twist()
        self.executing_task = False
        self.task_index = 0
        self.task_start_time = None

        # 🌟 在這裡設定動作
        self.task_flow = [
            {"name": "takeoff", "duration": 2.0},
            # {"name": "ascend",  "duration": 2.0},
            # {"name": "forward", "duration": 3.0},
            # {"name": "rotate",  "duration": 2.0},
            # {"name": "hover",   "duration": 2.0},
            # {"name": "land",    "duration": 2.0},
        ]

        # tuning parameters for velocities
        self.ascend_speed = 0.6     # m/s
        self.forward_speed = 0.6    # m/s
        self.rotate_speed = 0.9     # rad/s

        self.get_logger().info('TelloController initialized. Waiting to run task flow...')

    def img_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "rgb8")
            cv2.imshow("img", self.img)
            cv2.waitKey(1)
        except Exception as e:
            print(e)

    # ---------------- Image callback ----------------
    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        # 簡單顯示影像（非阻塞）
        try:
            cv2.imshow('Tello Camera', cv_image)
            # OpenCV 的 waitKey 需要在主線程/定時器週期呼叫以處理 GUI 事件
            # 但這裡只是供檢視用，實際控制不依賴 cv2.waitKey
            cv2.waitKey(1)
        except Exception as e:
            # 若在 headless 環境則忽略
            pass

    # ---------------- control timer ----------------
    def control_timer_callback(self):
        # 定期 publish 當前 Twist（若為 zero 表示懸停）
        try:
            self.control_pub.publish(self.current_twist)
        except Exception as e:
            self.get_logger().error(f'Failed to publish control: {e}')

    # ---------------- task/state machine ----------------
    def task_timer_callback(self):
        # Called every 1s: 驗證並啟動/結束任務
        if self.task_index >= len(self.task_flow):
            # 任務完成
            self.get_logger().info('All tasks finished. Setting hover and stopping timers.')
            self.current_twist = Twist()  # stop
            # 這裡不自動關閉 node，讓使用者決定何時 shutdown
            return

        current_task = self.task_flow[self.task_index]
        name = current_task['name']
        duration = current_task.get('duration', 1.0)

        now = self.get_clock().now().to_sec()

        if not self.executing_task:
            # 開始新的任務
            self.get_logger().info(f"Starting task {self.task_index}: {name} ({duration}s)")
            self.task_start_time = now
            self.executing_task = True
            # 立即執行該 task 的 action
            self._start_task_action(name)
            return

        # 若正在執行該任務，檢查是否超時
        elapsed = now - (self.task_start_time or now)
        if elapsed >= duration:
            # 結束本任務並進入下一個
            self.get_logger().info(f"Task {self.task_index} ({name}) finished after {elapsed:.2f}s")
            self._end_task_action(name)
            self.task_index += 1
            self.executing_task = False
            self.task_start_time = None
        else:
            # 繼續執行：部分任務在 _start_task_action 已設置 current_twist
            self.get_logger().debug(f"Executing {name}: {elapsed:.2f}/{duration:.2f}s")

    # ---------------- task actions ----------------
    def _start_task_action(self, name: str):
        if name == "takeoff":
            # publish takeoff once
            try:
                self.takeoff_pub.publish(Empty())
                self.get_logger().info("Published takeoff message.")
                # keep hover initially
                self.current_twist = Twist()
            except Exception as e:
                self.get_logger().error(f"Failed to publish takeoff: {e}")

        elif name == "ascend":
            t = Twist()
            t.linear.z = float(self.ascend_speed)
            self.current_twist = t
            self.get_logger().info(f"Ascending at {self.ascend_speed} m/s")

        elif name == "forward":
            t = Twist()
            t.linear.x = float(self.forward_speed)
            self.current_twist = t
            self.get_logger().info(f"Moving forward at {self.forward_speed} m/s")

        elif name == "rotate":
            t = Twist()
            t.angular.z = float(self.rotate_speed)
            self.current_twist = t
            self.get_logger().info(f"Rotating at {self.rotate_speed} rad/s")

        elif name == "hover":
            self.current_twist = Twist()
            self.get_logger().info("Hovering (zero velocities)")

        elif name == "land":
            try:
                # prepare to stop movement first
                self.current_twist = Twist()
                # publish land once
                self.land_pub.publish(Empty())
                self.get_logger().info("Published land message.")
            except Exception as e:
                self.get_logger().error(f"Failed to publish land: {e}")

        else:
            self.get_logger().warning(f"Unknown task '{name}', hovering.")
            self.current_twist = Twist()

    def _end_task_action(self, name: str):
        # 清理該任務所設定的狀態（若需要）
        # 目前策略：每個任務結束都先設置為 hover（0 velocity）
        self.current_twist = Twist()
        # 若 land 已完成，給使用者提示
        if name == "land":
            self.get_logger().info("Landed. Task flow completed (or continuing if more tasks).")

def main(args=None):
    rclpy.init(args=args)
    node = TelloController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down...')
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        try:
            # ensure zero velocity before exit
            node.current_twist = Twist()
            node.control_pub.publish(node.current_twist)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
        # close OpenCV windows (if any)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == '__main__':
    main()