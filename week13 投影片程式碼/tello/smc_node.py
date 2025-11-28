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

from statemachine import StateMachine, State

class TelloPassMachine(StateMachine):
    
    ### defined States
    onLand = State('onland', initial=True)
    Takeoff = State('takeoff')
    Hover = State('hover')
    Forward = State('forward')
    Correction = State('correction')
    AddSp = State('addsp')
    Land = State('land', final=True)

    ### defined transition
    start = onLand.to(Takeoff)
    wait_data = Takeoff.to(Hover) | Hover.to(Hover) | Forward.to(Hover) | Correction.to(Hover)
    f = Hover.to(Forward) | Forward.to(Forward) | Correction.to(Forward)
    c = Hover.to(Correction) | Forward.to(Correction) | Correction.to(Correction)
    a = Hover.to(AddSp)
    l = AddSp.to(Land)
    
    def __init__(self, node):
        super().__init__()
        self.node = node
    
    def on_enter_onLand(self):
        print("Enter the onLand")

    def on_exit_onLand(self):
        print("Exit the onLand")

    def on_enter_Takeoff(self):
        print("Enter the Takeoff.")
        self.node.takeoff_publisher.publish(Empty())
        time.sleep(3)
        self.wait_data()

    def on_exit_Takeoff(self):
        print("Exit the Takeoff")
        
    def on_enter_Hover(self):
        print("Enter the Hover")
        msg = Twist()
        self.node.control_publisher.publish(msg)

    def on_exit_Hover(self):
        print("Exit the Hover")
        
    def on_enter_Forward(self):
        print("Enter the Forward")
        msg = Twist()
        msg.linear.y = 20.0
        self.node.control_publisher.publish(msg) 
    
    def on_exit_Forward(self):
        print("Exit the Forward")
        
    def on_enter_Correction(self):
        print("Enter the Correction")
        dx = self.node.cx - 480
        dy = self.node.cy - 200
        
        msg = Twist()
        if dx != 0:
          msg.linear.x = np.sign(dx) * 10.0
        if dy != 0:
          msg.linear.z = -1 * np.sign(dy) * 15.0
        self.node.control_publisher.publish(msg)
        
    def on_exit_Correction(self):
        print("Exit the Correction")
        
    def on_enter_AddSp(self):
        print("Enter the AddSp")
        msg = Twist()
        msg.linear.y = 50.0
        self.node.control_publisher.publish(msg)
        time.sleep(3)
        
        self.l()
    
    def on_exit_AddSp(self):
        print("Exit the AddSp")

    def on_enter_Land(self):
        
        msg = Twist()
        self.node.control_publisher.publish(msg)
        time.sleep(3)
        
        print("Enter the Land")
        self.node.land_publisher.publish(Empty())
    
    def on_exit_Land(self):
        print("Exit the Land")

### create class SMc node, inherits from Node
class SMc(Node):

    def __init__(self):
        super().__init__('statemachine_control')
        
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
        
        ### create publisher
        self.takeoff_publisher = self.create_publisher(Empty, "/takeoff", 1)
        self.land_publisher = self.create_publisher(Empty, "/land", 1)
        self.control_publisher = self.create_publisher(Twist, "/control", 10)
        
        ### create subscriber
        self.status_subscriber = self.create_subscription(TelloStatus, "/status", self.status_callback, 10)
        self.inform_subscriber = self.create_subscription(Float64MultiArray, "/ip_inform", self.inform_callback, 10)
        
        self.mc = TelloPassMachine(self)
        
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
        
        if self.start == False:
            img_path = "./smc_ini.png"
            self.mc._graph().write_png(img_path)
            
            self.mc.send("start")
            self.start = True
        
        if self.cx == -1 or self.cy == -1:
            self.mc.send("wait_data")
            return
              
        dx = self.cx - 480
        dy = self.cy - 200
        print(dx, dy, self.can_pass)
        
        if self.mc.Land.is_active:
            time.sleep(5)
            rclpy.shutdown()
        
        if self.mc.AddSp.is_active:
            return 
        
        if self.can_pass == 1:
            if dx >= 30 or dy >= 30:
                self.mc.send("c")
            else:
                self.mc.send("wait_data")
            
            if self.mc.Hover.is_active: 
                print("now add speed to pass rectangle")
                self.mc.send("a")                
        else:
            if dx >= 30 or dy >= 30:
                self.mc.send("c") 
            else:
                self.mc.send("f")
        
### main function
def main(args=None):
    
    ### rclpy library is initialized
    rclpy.init(args=args)
    
    ### create SMc 
    smc = SMc()
    
    ### spin the smc node, so its callback are called 
    rclpy.spin(smc)
    
    ### after ctrl-c or the other things to close node
    smc.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()  

