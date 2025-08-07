import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
class Player2(Node):

    def __init__(self):
        super().__init__('player2_node')
        qos_profile = QoSProfile(depth=10,
                                 durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.ping_publish = self.create_publisher(Int64, '/ping_pong', qos_profile)
        self.pong_subscribe = self.create_subscription(Int64, '/ping_pong', self.player1_callback, qos_profile)

        self.count = 0 
        self.get_logger().info('Player2 Ready!')

    def player1_callback(self, player1_msg):
        if player1_msg.data <= self.count: # discard message for player1
            return
        self.get_clock().sleep_for(Duration(seconds=1.0))
        self.count = player1_msg.data
        self.count +=1
        self.get_logger().info(f'Player 1 Pubslishing {self.count}')
        self.return_msgs = Int64()
        self.return_msgs.data = self.count
        self.ping_publish.publish(self.return_msgs)
        #------------------------------------------------
        #                    TODO:
        # 3.) Store received data to self.count 
        # 4.) Increment self.count
        # 5.) Log Player 1 Publishing <count>
        # 6.) Publish the self.count value!
        #------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    player2_node = Player2()
    rclpy.spin(player2_node)
    player2_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
