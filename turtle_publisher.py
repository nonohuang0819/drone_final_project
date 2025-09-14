import rclpy
from rclpy.node import Node

### from geometry_msgs' msg import interface Twist
from geometry_msgs.msg import Twist

class TurtlesimPublisher(Node):

    def __init__(self):
        super().__init__('turtle_publisher')
        
        ### create publisher to publish Twist interface to topic /turtle1/cmd_vel
        ### 10 is a publish queue size
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        ### set time period 0.1s and create timer to run self.timer_callback, every 0.1s do
        time_period = 0.1
        self.timer = self.create_timer(time_period, self.timer_callback)
        
    ### timer_callback function, create Twist interface and using self.publisher to publish
    ### after publish, print msg
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 1.6 # 速度
        msg.angular.z = -0.7 # 角度
        self.publisher_.publish(msg)
        print(msg)


def main(args=None):

    ### rclpy library is initialized
    rclpy.init(args=args)
    
    ### create TurtlesimPublisher object tp
    tp = TurtlesimPublisher()
    
    ### spin the tp node, so its callback are called
    rclpy.spin(tp)
    
    ### after ctrl-c or the other things to close node
    tp.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()