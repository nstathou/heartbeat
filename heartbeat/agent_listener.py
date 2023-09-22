import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import argparse

class AgentListener(Node):
    """ This is the agent listener class. It is responsible for listening
        messages from the base station. """

    def __init__(self, agent_name):
        """ Initialize the AgentListener class. """
        super(AgentListener, self).__init__(agent_name + '_listener')
        self.agent_name = agent_name
        ## Create a subscriber.
        self.subscription = self.create_subscription(
            String,
            'base_station/heartbeat',
            self.listener_callback,
            10)
        ## Prevent unused variable warning
        self.subscription  
        ## Create a publisher.
        self.publisher = self.create_publisher(String, agent_name + '/status', 10)
        ## Create a timer.
        timer_period = 1.0  ## seconds
        self.timer = self.create_timer(timer_period, self.status_identifier)
        ## Create a message.
        self.msg = ''

    def listener_callback(self, msg):
        """ Listener callback function. """
        self.get_logger().info(self.agent_name + ' heard: "%s"' % msg.data)
        self.msg = msg.data

    def status_identifier(self):
        """ Status identifier function. Check if the agent is connected
            to the base station. If yes, then publish a message to the
            agent/status topic. """
        if len(self.msg) == 0:
            status = 'disconnected'
            ## Example of changing state depending on the status
            self.get_logger().info(self.agent_name + ' state: "returning to base"')
        else:
            status = 'connected'
            ## Example of changing state depending on the status
            self.get_logger().info(self.agent_name + ' state: "exploring"')
        ## Publish a message to the agent/status topic.
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        ## Reset the message.
        self.msg = ''


def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    ## Create an AgentListener object with the provided agent name.
    parser = argparse.ArgumentParser(description='Agent Listener')
    parser.add_argument('agent_name', type=str, help='Name of the agent')
    args = parser.parse_args()
    ## Create an AgentListener object with the provided agent name.
    agent_listener = AgentListener(args.agent_name)
    ## Keep the node alive
    rclpy.spin(agent_listener)
    ## Destroy the node explicitly
    agent_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
