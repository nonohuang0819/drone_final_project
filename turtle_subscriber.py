from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtlesimSubscriber(Node):

    def __init__(self):
        super().__init__('turtle_subscriber')
        
        # subscribe /turtle1/cmd_vel topic
        # data type:  Twist
        # use self.listen_cmd_vel_callback when receive msg
        self.cmd_vel_subscriber = self.create_subscription(Twist, "/turtle1/cmd_vel", self.listen_cmd_vel_callback, 10)
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.listen_pose_callback, 10)


    def listen_cmd_vel_callback(self, msg):
        print("--------------")
        print(f"cmd vel x: {msg.linear.x}")
        print(f"cmd vel y: {msg.angular.z}")
        print("--------------")
        
    
    def listen_pose_callback(self, msg):
        print("--------------")
        print(f"pose x: {msg.x}")
        print(f"pose y: {msg.y}")
        print(f"pose theta: {msg.theta}")
        print("--------------")
        print(f"pose linear velocity: {msg.linear_velocity}")
        print(f"pose angular velocity: {msg.angular_velocity}")
        print("--------------")


def main(args=None):
    # ### rclpy 函式庫初始化
    rclpy.init(args=args)
    
    # ### 創建 TurtlesimSubscriber 物件 ts
    ts = TurtlesimSubscriber()
    
    # ### "旋轉" ts 節點，使其回調函數可以被調用
    # 這會讓程式保持運行，並持續監聽訂閱的主題，直到節點被關閉
    rclpy.spin(ts)
    
    # ### 在按下 ctrl-c 或其他方式關閉節點後執行
    # 銷毀節點
    ts.destroy_node()
    # 關閉 rclpy 客戶端函式庫
    rclpy.shutdown()

# 如果這個腳本是作為主程式執行，則調用 main 函數
if __name__ == '__main__':
    main()