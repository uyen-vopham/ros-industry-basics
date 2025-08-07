import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
class Player1(Node):

    def __init__(self):
        super().__init__('player1_node')
        qos_profile = QoSProfile(depth=10,
                                 durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.ping_publisher = self.create_publisher(Int64, '/ping_pong', qos_profile)
        self.pong_susciber = self.create_subscription(Int64, '/ping_pong', self.player2_callback, qos_profile)

        self.count = 1 
        self.get_logger().info('Player1 Ready!')

    def player2_callback(self, player2_msg):
        if player2_msg.data <= self.count: # discard message for player2
            return
        self.get_clock().sleep_for(Duration(seconds=1.0)) # Periodic sleep to slowdown exchange 
        self.count = player2_msg.data
        self.count +=1
        self.get_logger().info(f'Player 1 Publishing {self.count}')
        self.return_msgs = Int64()
        self.return_msgs.data = self.count
        self.ping_publisher.publish(self.return_msgs)
        
    

def main(args=None):
    rclpy.init(args=args)
    player1_node = Player1()
    starting_msg = Int64()
    starting_msg.data = player1_node.count
    
    # 3.) Log Player 1 Publishing 
    #------------------------------------------------
    player1_node.get_logger().info('Player uyen Publishing %d' % starting_msg.data)
    
    #------------------------------------------------
    #                    TODO:
    # 4.) Publish to the ROS2 message to /ping_pong
    #------------------------------------------------
    player1_node.ping_publisher.publish(starting_msg)
    rclpy.spin(player1_node)
    player1_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
