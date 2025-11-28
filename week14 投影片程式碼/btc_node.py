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
        self.tree = (
            (self.notStart >> self.Takeoff)
            | (self.isNotDataReceived >> self.Hover)
            | (self.canPass >> self.isCorrect >> self.AddSp >> self.Land)
            | ( (self.isCorrect >> self.Forward) | self.Correction )
        )
        self.instance = self.tree.blackboard(node)
    
    ### def condition and action
    
    @condition    
    def isNotDataReceived(node):
        print("C: isNotDataReceived.")
        return node.cx == -1 or node.cy == -1
    
    @action
    def Hover(node):
        print("A: Hover.")
        msg = Twist()
        node.control_publisher.publish(msg)
    
    @condition
    def canPass(node):
        print("C: canPass.")
        return node.can_pass == 1
    
    @condition
    def isCorrect(node):
        print("C: isCorrect.")
        return (node.cx - 480) < 30 and (node.cy - 200) < 30
    
    @action
    def AddSp(node):
        print("A: AddSp.")
        msg = Twist()
        msg.linear.y = 50.0
        node.control_publisher.publish(msg)
        time.sleep(3)
        
    @action
    def Land(node):
        print("A: Land.")
        msg = Twist()
        node.control_publisher.publish(msg)
        time.sleep(3)

        node.land_publisher.publish(Empty())
        node.isFinished = True
    
    @condition
    def notStart(node):
        print("C: notStart.")
        return node.start == False
    
    @action
    def Takeoff(node):
        print("A: Takeoff.")
        node.takeoff_publisher.publish(Empty())
        time.sleep(3)
        node.start = True
        
    @action
    def Forward(node):
        print("A: Forward.")
        msg = Twist()
        msg.linear.y = 20.0
        node.control_publisher.publish(msg) 
    
    @action
    def Correction(node):
        print("A: Correction")
        dx = node.cx - 480
        dy = node.cy - 200
        
        msg = Twist()
        if dx != 0:
          msg.linear.x = np.sign(dx) * 10.0
        if dy != 0:
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
        self.can_pass = 0
        
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
        control_time_period = 0.5
        self.control_timer = self.create_timer(control_time_period, self.control_timer_callback)
     
    def status_callback(self, msg):
        if self.init_height == -99:
          self.init_height = msg.barometer
          print("ini height: ", self.init_height)
        self.height = msg.barometer - self.init_height
        self.battery = msg.battery
        self.yaw = msg.yaw
        self.flight_time = msg.fligth_time
    
    def inform_callback(self, msg):
        self.cx = msg.data[0]
        self.cy = msg.data[1]
        self.can_pass = msg.data[2]

    def control_timer_callback(self):
        if self.init_height == -99:
          return
        
        if self.isFinished:
            rclpy.shutdown()
            return
        
        self.bt = TelloPassBT(self)  
        state = self.bt.instance.tick()
        print("state = %s\n" % state)
        while state == RUNNING:
    
            state = self.bt.instance.tick()
            print("state = %s\n" % state)
        assert state == SUCCESS or state == FAILURE
        
### main function
def main(args=None):
    
    ### rclpy library is initialized
    rclpy.init(args=args)
    
    ### create BTc 
    btc = BTc()
    
    ### spin the btc node, so its callback are called 
    rclpy.spin(btc)
    
    ### after ctrl-c or the other things to close node
    btc.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()  

