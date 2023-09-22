import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AgentListener(Node):
    """ This is the agent listener class. It is responsible for listening
        messages from the base station. """

    def __init__(self):
        """ Initialize the AgentListener class. """
        super(AgentListener, self).__init__('agent_listener')
        ## Create a subscriber.
        self.subscription = self.create_subscription(
            String,
            'base_station/heartbeat',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        """ Listener callback function. """
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    ## Create an AgentListener object.
    agent_listener = AgentListener()
    ## Spin the AgentListener object.
    rclpy.spin(agent_listener)
    ## Destroy the AgentListener object.
    agent_listener.destroy_node()
    ## Shutdown the ROS2 client.
    rclpy.shutdown()


if __name__ == '__main__':
    main()