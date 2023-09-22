import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NUM_AGENTS = 2

class BSListener(Node):
    """ This is the base station listener class. It is responsible for listening
        messages from all agents. """
    
    def __init__(self):
        """ Initialize the BSListener class. """
        super(BSListener, self).__init__('base_station_listener')
        ## Create a subscriber for each agent.
        self.subscription = []
        for i in range(NUM_AGENTS):
            self.subscription.append(self.create_subscription(
                String,
                'agent' + str(i) + '/heartbeat',
                lambda msg, i=i: self.listener_callback(msg, i),  # Using a lambda function to capture the value of i
                10))
        ## Create a publisher.
        self.publisher = self.create_publisher(String, 'base_station/status', 10)
        ## Create a timer.
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.status_identifier)
        ## Create a list of messages.
        self.msgs = ['' for _ in range(NUM_AGENTS)]

    def listener_callback(self, msg, i):
        """ Listener callback function. """
        self.get_logger().info('Base station heard: "%s"' % msg.data + ' from agent' + str(i))
        ## Publish a message to the base_station/status topic.
        self.msgs[i] = msg.data

    def status_identifier(self):
        """ Status identifier function for the base station. For all agents that you receive 
            a message identify as connected, otherwise as disconnected. Publish a single string
            message to the base_station/status topic containing the status of all agents. """
        status = ''
        for i in range(NUM_AGENTS):
            if len(self.msgs[i]) == 0:
                status += 'agent' + str(i) + ': disconnected, '
            else:
                status += 'agent' + str(i) + ': connected, '
        ## Publish a message to the base_station/status topic.
        msg = String()
        msg.data = status
        self.publisher.publish(msg)
        ## Reset the message.
        self.msgs = ['' for _ in range(NUM_AGENTS)]

def main(args=None):
    """ Main function. """
    rclpy.init(args=args)
    ## Create a BSListener object.
    bs_listener = BSListener()
    ## Spin the BSListener object.
    rclpy.spin(bs_listener)
    ## Destroy the BSListener object.
    bs_listener.destroy_node()
    ## Shutdown the ROS2 client.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
