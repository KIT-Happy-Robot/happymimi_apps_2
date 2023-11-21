import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from std_msgs.msg import Float64
from nav2_msgs.action import NavigateToPose
from happy_apps_msgs.srv import NaviCoord, NaviCoordResponse



class NaviCoordServer:
    def __init__(self):
        super().__init__('navi_coord')  # ノードの名前を設定
        #サーバーを建てるよ。
        self.server = self.create_service(
            NaviCoord,
            'navi_coord_server',
            self.sendGoal 
        )

        self.action_client = self.create_client(
            NavigateToPose,
            'navigate_to_pose'
        )

        self.publisher = self.create_publisher(
            Float64,
            'servo/head',
        )

        # service通信のクライアントを作成
        self.clientOfserver = self.create_client(
            Empty,
            '/move_base/clear_costmaps',
        )

        # 初期ポジション　んーかわいい。
        self.location_name = None
        
        # ここに初期化コードを追加
        self.get_logger().info('MyNode has been initialized.')
    
    def sendGoal():
        print("sendGoal")

    def run(self):
        while rclpy.ok():
            # ここにノードのメインロジックを追加
            self.get_logger().info('Running...')
            rclpy.spin_once(self)

def main(args=None):
    rclpy.init(args=args)

    my_node = MyNode()

    try:
        my_node.run()
    except KeyboardInterrupt:
        pass
    finally:
        my_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()