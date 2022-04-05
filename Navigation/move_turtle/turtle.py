import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from std_msgs.msg import String

class Turtle_Node(Node):
    def __init__(self):
        super().__init__('turtle_node')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 25
        self.x_positions = [0.5, 0.65, 0.8]
        self.y_positions = [0.0, 0.2, 0.4]
        self.i = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = PoseStamped()
        if self.i < 3:
            msg.pose.position.x = self.x_positions[self.i]
            msg.pose.position.y = self.y_positions[self.i]
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.pose.position)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    turtle_node = Turtle_Node()

    rclpy.spin(turtle_node)

    turtle_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

