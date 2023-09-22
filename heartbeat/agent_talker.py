import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse

class AgentTalker(Node):
    """ This is the agent talker class. It is responsible for broadcasting
        its state to the base station and all agents. """

    def __init__(self, agent_name):
        """ Initialize the AgentTalker class. """
        super(AgentTalker, self).__init__(agent_name + '_talker')
        self.agent_name = agent_name
        ## Create a publisher.
        self.publisher = self.create_publisher(String, agent_name + '/heartbeat', 10)
        ## Create a timer.
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        ## Create a counter.
        self.i = 0

    def timer_callback(self):
        """ Timer callback function. """
        ## Publish a message to the agent/heartbeat topic.
        msg = String()
        msg.data = self.i
        self.publisher.publish(msg)
        self.get_logger().info(self.agent_name + ' publishing: ' + msg.data)
        self.i += 1 


def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    ## Create an argument parser.
    parser = argparse.ArgumentParser(description='AgentTalker Node')
    parser.add_argument('agent_name', type=str, help='Name of the agent')
    args = parser.parse_args()
    ## Create an AgentTalker object with the provided agent name.
    agent_talker = AgentTalker(args.agent_name)
    ## Spin the AgentTalker object.
    rclpy.spin(agent_talker)
    ## Destroy the AgentTalker object.
    agent_talker.destroy_node()
    ## Shutdown the ROS2 client.
    rclpy.shutdown()


if __name__ == '__main__':
    main()
