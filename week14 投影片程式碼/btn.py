### import client library: rclpy, and Node object
import rclpy
from rclpy.node import Node

### from geometry_msgs's msg import interface Twist
from geometry_msgs.msg import Twist

### from std_msgs's msg import interface Empty, Float64MultiArray
from std_msgs.msg import Empty, Float64MultiArray

### from tello_msg's msg import interface TelloStatus
from tello_msg.msg import TelloStatus

import numpy as np
import time

from behave import *

class TelloPassBT:
    
    def __init__(self, node):
        super().__init__()
        
        ### defined bt
        # 修改後的行為樹邏輯：
        # 1. 沒起飛 -> 起飛
        # 2. 沒收到數據 (或丟失目標) -> 懸停
        # 3. 收到降落訊號 (Tag) -> 對準 -> 降落
        # 4. 收到過框訊號 (Hoop Close) -> 對準 -> 衝刺
        # 5. 一般搜尋/靠近 (Hoop Far) -> 對準 -> 前進
        
        self.tree = (
            (self.notStart >> self.Takeoff)
            | (self.isNotDataReceived >> self.Hover)
            # 優先權 1: 看到 AprilTag (Status 2) -> 嘗試降落
            | (self.isLandingSignal >> ( (self.isCorrect >> self.Land) | self.Correction ) )
            # 優先權 2: 看到紅框且夠近 (Status 1) -> 嘗試衝刺
            | (self.canPass >> ( (self.isCorrect >> self.AddSp) | self.Correction ) )
            # 優先權 3: 看到紅框但還遠 (Status 0) -> 修正與前進
            | ( (self.isCorrect >> self.Forward) | self.Correction )
        )
        self.instance = self.tree.blackboard(node)
    
    ### def condition and action
    
    @condition    
    def isNotDataReceived(node):
        # cx = -1 代表視覺端什麼都沒看到
        # print("C: isNotDataReceived.")
        return node.cx == -1 or node.cy == -1
    
    @action
    def Hover(node):
        print("A: Hover (Searching).")
        msg = Twist()
        node.control_publisher.publish(msg)
    
    @condition
    def isLandingSignal(node):
        # 對應 ipn 發出的 status 2.0 (AprilTag)
        if node.can_pass == 2:
            print("C: Landing Signal Detected (AprilTag).")
            return True
        return False

    @condition
    def canPass(node):
        # 對應 ipn 發出的 status 1.0 (過框)
        if node.can_pass == 1:
            print("C: Can Pass (Close to Hoop).")
            return True
        return False
    
    @condition
    def isCorrect(node):
        # 判斷是否對準中心
        # 注意：這裡沿用你原本的 (cx-480, cy-200) 設定
        # 如果降落時覺得對不準，可能要把 200 改成 360 (畫面中心)
        print("C: isCorrect checking...")
        return abs(node.cx - 480) < 40 and abs(node.cy - 200) < 40
    
    @action
    def AddSp(node):
        print("A: AddSp (Passing Hoop).")
        msg = Twist()
        msg.linear.y = 50.0 # Tello 向前衝
        node.control_publisher.publish(msg)
        time.sleep(2.5) # 衝刺時間
        # 注意：這裡移除了 Land，衝完後會回到迴圈繼續看下一個目標(找照片)
        
    @action
    def Land(node):
        print("A: Land (Mission Complete).")
        # 先懸停一下穩住機身
        msg = Twist()
        node.control_publisher.publish(msg)
        time.sleep(1)

        node.land_publisher.publish(Empty())
        time.sleep(5) # 等待降落完成
        node.isFinished = True
    
    @condition
    def notStart(node):
        print("C: notStart.")
        return node.start == False
    
    @action
    def Takeoff(node):
        print("A: Takeoff.")
        node.takeoff_publisher.publish(Empty())
        time.sleep(5) # 等待起飛穩
        node.start = True
        
    @action
    def Forward(node):
        print("A: Forward (Approaching).")
        msg = Twist()
        msg.linear.y = 15.0 # 慢慢靠近
        node.control_publisher.publish(msg) 
    
    @action
    def Correction(node):
        print("A: Correction")
        dx = node.cx - 480
        dy = node.cy - 200
        
        msg = Twist()
        
        # X軸修正 (左右)
        if abs(dx) > 20:
          msg.linear.x = np.sign(dx) * 10.0
          
        # Z軸修正 (上下)
        # 注意：dy是畫面座標差，dy>0代表目標在畫面下方，無人機要降低(linear.z < 0)
        if abs(dy) > 20:
          msg.linear.z = -1 * np.sign(dy) * 15.0
          
        node.control_publisher.publish(msg)
    
### create class BTc node, inherits from Node
class BTc(Node):

    def __init__(self):
        super().__init__('BehaviorTree_control')
        
        ### parameter inint
        self.init_height = -99
        self.height = -1
        self.battery = 100.0
        self.yaw = 0.0
        self.flight_time = -1
  
        self.cx = -1
        self.cy = -1
        self.can_pass = 0 # 0:Far, 1:Close(Hoop), 2:Tag(Land)
        
        self.start = False
        self.isFinished = False
        
        ### create publisher
        self.takeoff_publisher = self.create_publisher(Empty, "/takeoff", 1)
        self.land_publisher = self.create_publisher(Empty, "/land", 1)
        self.control_publisher = self.create_publisher(Twist, "/control", 10)
        
        ### create subscriber
        self.status_subscriber = self.create_subscription(TelloStatus, "/status", self.status_callback, 10)
        self.inform_subscriber = self.create_subscription(Float64MultiArray, "/ip_inform", self.inform_callback, 10)
        
        self.bt = TelloPassBT(self)
        
        ### create timer for control input 
        # 建議縮短 control loop 時間以獲得更靈敏的反應
        control_time_period = 0.1 
        self.control_timer = self.create_timer(control_time_period, self.control_timer_callback)
     
    def status_callback(self, msg):
        if self.init_height == -99:
          self.init_height = msg.barometer
          print("ini height: ", self.init_height)
        self.height = msg.barometer - self.init_height
        self.battery = msg.battery
        self.yaw = msg.yaw
        self.flight_time = msg.flight_time # 注意：原本你的拼字是 fligth_time，請確認 msg 定義
    
    def inform_callback(self, msg):
        self.cx = msg.data[0]
        self.cy = msg.data[1]
        self.can_pass = msg.data[2]

    def control_timer_callback(self):
        # 移除高度檢查，避免模擬環境或室內氣壓計不準導致不啟動
        # if self.init_height == -99:
        #   return
        
        if self.isFinished:
            # 任務結束
            return
        
        # 每次 tick 不需要重新建立 TelloPassBT 物件，使用 self.bt 即可
        # 這裡原本的寫法會重置 bt 狀態，導致狀態無法延續，建議改為：
        state = self.bt.instance.tick()
        # print("state = %s" % state)
        
        # 如果需要 tick 到非 RUNNING 為止 (behave 預設行為)
        while state == RUNNING:
            state = self.bt.instance.tick()
        
        if state == SUCCESS and self.isFinished:
            self.get_logger().info("Mission Complete. Shutting down.")
            rclpy.shutdown()
        
### main function
def main(args=None):
    
    ### rclpy library is initialized
    rclpy.init(args=args)
    
    ### create BTc 
    btc = BTc()
    
    try:
        ### spin the btc node, so its callback are called 
        rclpy.spin(btc)
    except KeyboardInterrupt:
        pass
    finally:
        ### after ctrl-c or the other things to close node
        btc.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()