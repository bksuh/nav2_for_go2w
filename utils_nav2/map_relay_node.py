import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

class MapRelayNode(Node):
    def __init__(self):
        super().__init__('map_relay_node')

        # SLAM의 map 토픽을 구독하기 위한 QoS (보통 기본값으로도 잘 동작)
        subscription_qos = QoSProfile(depth=10)

        # Nav2가 요구하는 map 토픽을 발행하기 위한 QoS
        publisher_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL # 핵심!
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map_relay_map',  # SLAM이 발행하는 원본 지도 토픽 이름
            self.map_callback,
            subscription_qos)
        
        self.publisher = self.create_publisher(
            OccupancyGrid,
            '/map',       # Nav2가 사용할 새로운 지도 토픽 이름
            publisher_qos)

    def map_callback(self, msg):
        self.get_logger().info('Relaying map...')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()