import rclpy
from rclpy.node import Node
import sys, random

from turtlesim.srv import SetPen
from turtlesim.msg import Pose
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import ParameterValue


class TurtleClient(Node): # turtlesimSP: service and parameters
    def __init__(self):
        # 初始化父類別 Node，並將節點名稱設為 'turtle_service_parameter'
        super().__init__('turtle_service_parameter')
        
        # === 創建服務客戶端 (Service Client) ===
        # 創建一個客戶端，用於呼叫 /turtle1/set_pen 服務，服務類型為 SetPen
        self.cli_pen = self.create_client(SetPen, '/turtle1/set_pen')
        # 進入一個 while 迴圈，每秒檢查一次 /turtle1/set_pen 服務是否已啟動
        # 這是為了確保在我們發送請求前，服務端已經準備就緒
        while not self.cli_pen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen service not available, waiting again...')
        
        # 創建一個客戶端，用於呼叫 /turtle1/set_parameters 服務，服務類型為 SetParameters
        self.cli_param = self.create_client(SetParameters, '/turtle1/set_parameters')
        # 同樣地，等待 /turtle1/set_parameters 服務啟動
        while not self.cli_param.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_parameters service not available, waiting again...')
        
        # 創建一個 SetPen 服務的請求物件
        self.req_pen = SetPen.Request()

    # ### 定義一個方法來發送設定畫筆 (set_pen) 的請求
    def send_request(self, r, g, b):
        # 根據傳入的參數設定請求物件的內容
        self.req_pen.r = r
        self.req_pen.g = g
        self.req_pen.b = b
        self.req_pen.width = 5 # 將畫筆寬度固定設為 5
        self.req_pen.off = 0 # 0 表示畫筆放下 (開始繪畫)
        # 非同步地呼叫服務，並將請求物件 self.req_pen 發送出去
        # 這個呼叫會立即返回一個 future 物件，代表未來會收到的回應
        self.future = self.cli_pen.call_async(self.req_pen)

    # ### 定義一個方法來發送設定參數 (背景顏色) 的請求
    def set_parameters_request(self, r, g, b):
        # 創建一個 SetParameters 服務的請求物件
        self.cli_req = SetParameters.Request()
        
        # 設定要變更的參數列表
        # turtlesim 的背景顏色是由三個獨立的參數 background_r, background_g, background_b 控制的
        self.cli_req.parameters = [
            Parameter(name='background_r', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=r)),
            Parameter(name='background_g', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=g)),
            Parameter(name='background_b', value=ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=b))
        ]
        # 非同步地呼叫服務，並將請求物件 self.cli_req 發送出去
        self.future_param = self.cli_param.call_async(self.cli_req)

# ### 主函數
def main(args=None):
    # ### rclpy 函式庫初始化
    rclpy.init(args=args)
    
    # ### 創建 TurtleClient 物件 tcp
    tcp = TurtleClient()
    
    # ### 呼叫請求函數，傳入從命令列獲取的 RGB 值
    # sys.argv[0] 是腳本名稱, sys.argv[1] 是第一個參數，以此類推
    # 使用 int() 將字串參數轉為整數，並用 clip 函數確保值在 0-255 之間
    # 這部分程式碼在圖片中有誤，這裡修正為直接呼叫方法
    # 實際執行時，應先呼叫 `set_parameters_request`
    tcp.set_parameters_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))

    # ### 旋轉 (spin) 節點，直到服務回應完成
    # 這會阻塞程式，直到 self.future_param 這個 future 物件收到回應
    rclpy.spin_until_future_complete(tcp, tcp.future_param)
    # 獲取服務的回應結果 (雖然這裡沒有使用，但可以獲取)
    response_param = tcp.future_param.result()
    if response_param:
        tcp.get_logger().info('Set background color successfully')

    # 接著呼叫設定畫筆顏色的請求
    # 這裡使用隨機顏色，也可以改成從命令列讀取
    # random_color = [random.randint(0,255) for _ in range(3)]
    # tcp.send_request(*random_color) # * 用於解包列表
    tcp.send_request(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3]))
    
    # ### 再次旋轉節點，直到設定畫筆的服務回應完成
    rclpy.spin_until_future_complete(tcp, tcp.future)
    response_pen = tcp.future.result()
    if response_pen is not None: # set_pen 服務的回應是空的，所以檢查是否為 None
        tcp.get_logger().info('Set pen successfully')
    
    # ### 關閉節點
    tcp.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()