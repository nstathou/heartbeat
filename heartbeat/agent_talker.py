import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class AgentTalker(Node):
    """ This is the agent talker class. It is responsible for broadcasting
        its state to the base station and all agents. """

    def __init__(self):
        """ Initialize the AgentTalker class. """
        super(AgentTalker, self).__init__('agent_talker')
        ## Create a publisher.
        self.publisher = self.create_publisher(String, 'agent/heartbeat', 10)
        ## Create a timer.
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ## Create a counter.
        self.i = 0

    def timer_callback(self):
        """ Timer callback function. """
        msg = String()
        msg.data = str(self.i)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1 


def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    ## Create an AgentTalker object.
    agent_talker = AgentTalker()
    ## Spin the AgentTalker object.
    rclpy.spin(agent_talker)
    ## Destroy the AgentTalker object.
    agent_talker.destroy_node()
    ## Shutdown the ROS2 client.
    rclpy.shutdown()


if __name__ == '__main__':
    main()