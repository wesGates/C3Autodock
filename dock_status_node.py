from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from irobot_create_msgs.msg import DockStatus
from std_msgs.msg import String


class DockStatusMonitorNode(Node):
    def __init__(self, namespace):
        super().__init__('dock_sensor')

        # Publisher to send the dock status as a string message to other nodes
        # The topic '/check_dock_status' is used by other nodes in the system for synchronization and decision making.
        self.dock_status_publisher_ = self.create_publisher(String, f'/{namespace}/check_dock_status', 10)

        # Subscription to the '/dock_status' topic to listen for DockStatus messages
        # This allows the node to receive real-time updates on the robot's docking status.
        self.dock_status_subscriber_ = self.create_subscription(DockStatus, f'/{namespace}/dock_status', 
                                                                self.dock_status_listener, qos_profile_sensor_data)

        # Variable to store the latest dock status
        # It's initialized to None and will be updated with actual data from the '/dock_status' topic.
        self.current_dock_status = None


    def dock_status_listener(self, msg):
        # Callback function for processing messages received on the '/dock_status' topic
        # It updates 'self.current_dock_status' with the latest status.
        self.current_dock_status = msg.is_docked 
        # self.current_dock_status = str(msg.is_docked) # Uncomment if string representation is needed


    def publish_dock_status(self):
        # Publishes the current dock status as a string message
        # It converts the boolean 'current_dock_status' to a string before publishing.
        msg = String()
        msg.data = str(self.current_dock_status) # Convert the boolean to a string for consistency across topics
        self.dock_status_publisher_.publish(msg)


















