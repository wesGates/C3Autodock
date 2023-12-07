from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class OdomNode(Node):
    def __init__(self, namespace):
        super().__init__('odometry_tracker')

        # Publisher for PoseStamped messages
        self.pose_publisher_ = self.create_publisher(PoseStamped, f'/{namespace}/pose_stamped', 10)

        # Subscription to the odometry topic to receive Odometry messages
        self.odometry_subscriber_ = self.create_subscription(Odometry, f'/{namespace}/odom', 
                                                                self.odometry_listener, qos_profile_sensor_data)

        # Variable initialization
        self.current_odometry = None


    def odometry_listener(self, odometry_msg):
        # Update the current odometry
        self.current_odometry = odometry_msg

    def publish_odometry(self):
        # Check if we have received odometry data
        if self.current_odometry is not None:
            pose_stamped_msg = PoseStamped()
            pose_stamped_msg.header = self.current_odometry.header
            pose_stamped_msg.pose = self.current_odometry.pose.pose
            # print("published odometry (pose_stamped):", pose_stamped_msg)

            # Publish the PoseStamped message
            self.pose_publisher_.publish(pose_stamped_msg)
            self.get_logger().info('PoseStamped message published.')











# class OdomNode(Node):
#     def __init__(self, namespace):
#         super().__init__('odometry_tracker')

#         # Publisher to send PoseStamped messages to other nodes, which is a subset of the /odom topic and is used for the NavigateToPosition() action
#         # This is used to broadcast the robot's pose based on odometry data to other parts of the system.
#         self.pose_publisher_ = self.create_publisher(PoseStamped, f'/{namespace}/pose_stamped', 10)

#         # Subscription to the 'odom' topic to receive Odometry messages
#         # This enables the node to listen for real-time odometry data from the robot's movement, which is tracked by a combination of several sensors.
#         self.odometry_subscriber_ = self.create_subscription(Odometry, f'/{namespace}/odom', 
#                                                              self.odometry_listener, qos_profile_sensor_data)

#         # Variable to store the latest odometry data received from the subscription
#         self.current_odometry = None

#     def odometry_listener(self, odometry_msg):
#         # Listener function for the 'odom' subscription
#         # Updates the current odometry data with the raw information from the received message.
#         self.current_odometry = odometry_msg

#     def publish_odometry(self):
#         # Publishes the current odometry data as a PoseStamped message
#         # This method checks if odometry data is available and then publishes it.
#         if self.current_odometry is not None: # Just a check to see if current_odometry has been updated yet. Unnecessary now
#             # Creating a PoseStamped message from the current odometry data
#             pose_stamped_msg = PoseStamped() # Establish a PoseStamped type message object
#             pose_stamped_msg.header = self.current_odometry.header  # Copying only the header from the odometry data
#             pose_stamped_msg.pose = self.current_odometry.pose.pose  # Setting the pose information

#             # Publishing the PoseStamped message on the specified /pose_stamped topic
#             self.pose_publisher_.publish(pose_stamped_msg)
#             # Logging information for debugging and confirmation
#             self.get_logger().info('PoseStamped message published.')



















