import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BSTalker(Node):
    """ This is the base station talker class. It is responsible for broadcasting
        messages from the base station to all agents. """

    def __init__(self):
        """ Initialize the BSTalker class. """
        super(BSTalker, self).__init__('base_station_talker')
        ## Create a publisher.
        self.publisher = self.create_publisher(String, 'base_station/heartbeat', 10)
        ## Create a timer.
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ## Create a counter.
        self.i = 0

    def timer_callback(self):
        """ Timer callback function. """
        msg = String()
        msg.data = str(self.i)
        self.publisher.publish(msg)
        self.get_logger().info('Base station publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    ## Create a BSTalker object.
    bs_talker = BSTalker()
    ## Spin the BSTalker object.
    rclpy.spin(bs_talker)
    ## Destroy the BSTalker object.
    bs_talker.destroy_node()
    ## Shutdown the ROS2 client.
    rclpy.shutdown()


if __name__ == '__main__':
    main()