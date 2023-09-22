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
        ## Create a publisher.
        self.publisher = self.create_publisher(String, 'agent/status', 10)
        ## Create a timer.
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.status_identifier)
        ## Create a message.
        self.msg = ''
        
    def listener_callback(self, msg):
        """ Listener callback function. """
        self.get_logger().info('I heard: "%s"' % msg.data)
        ## Publish a message to the agent/status topic.
        self.msg = msg.data

    def status_identifier(self):
        """ Status identifier function. Check is the agent is connected
            to the base station. If yes then publish a message to the
            agent/status topic. """
        ## Check if the agent is connected to the base station.
        if len(self.msg) == 0:
            status = 'disconnected'
            ## Trigger agent state change.
            self.get_logger().info('Agent state changed to: "%s"' % "returning to base station")
        else:
            status = 'connected'
            ## Trigger agent state change.
            self.get_logger().info('Agent state changed to: "%s"' % "exploring")
        ## Publish a message to the agent/status topic.
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        ## Reset the message.
        self.msg = ''


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