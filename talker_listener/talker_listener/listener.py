import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Class Listener inherits the Node class from rclpy.node
class Listener(Node):
    def __init__(self):
        # Initialise a node using super class constructor
        super().__init__('listener_node')
        self.subscription = self.create_subscription(String, 'jde_task', self.listener_callback, 10)
        
    def listener_callback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)

    
def main(args=None):
    rclpy.init()
    subscriber = Listener()

    rclpy.spin(subscriber)

    subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()