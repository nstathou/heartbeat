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

        self.msgs = ['' for _ in range(NUM_AGENTS)]

    def listener_callback(self, msg, i):
        """ Listener callback function. """
        self.get_logger().info('Agent ' + str(i) + ' heard: "%s"' % msg.data)
        ## Publish a message to the base_station/status topic.
        self.msgs[i] = msg.data

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
