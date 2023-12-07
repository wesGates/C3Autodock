# ROS Imports
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Create3 Packages
from irobot_create_msgs.msg import  IrOpcode

# Python Packages
from math import pi
from std_msgs.msg import String # Some topics have specific datatypes

class IrMonitorNode(Node):
	def __init__(self, namespace):
		super().__init__('ir_sensor')
		
		# Publishers
		self.opcode_publisher_ = self.create_publisher(String, 'ir_opcode_number', 10)

		# Subscriptions
		self.opcode_subscriber = self.create_subscription(IrOpcode, f'/{namespace}/ir_opcode', 
														  self.ir_opcode_listener, qos_profile_sensor_data)
		
		# Variable Initialization for storing published information
		self.current_opcode = None


	def ir_opcode_listener(self, msg):
		""" 
		Constantly updates the opcode message from the /ir_opcode topic.
		This includes a bunch of info that will be filtered out in the publisher.
		"""
		self.current_opcode = msg.opcode # Containes all the opcode information (a lot of stuff we dont need.



	def publish_ir_opcode(self):
		""" This method publishes the opcode value as a UInt8 message to all subscribers """
		msg = String()
		msg.data = str(self.current_opcode)

		self.opcode_publisher_.publish(msg)
		return self.current_opcode




# class IrMonitorNode(Node):
#     def __init__(self, namespace):
#         super().__init__('ir_sensor')
        
#         # Publisher for sending IR opcode information as a string message to other nodes.
#         # The topic 'ir_opcode_number' is intended for use in other parts of the system that require this IR information.
# 		# NOTE: This isn't actually how it's accessed in the main script. The 'return' function in publish_ir_opcode is instead used. Would love to hear why this is wrong.
#         self.opcode_publisher_ = self.create_publisher(String, 'ir_opcode_number', 10)

#         # Subscription to the 'ir_opcode' topic to receive IrOpcode messages.
#         # This enables the node to listen for real-time updates of IR opcodes from the robot's sensors, freeing up threads and avoiding excess signal noise.
#         self.opcode_subscriber = self.create_subscription(IrOpcode, f'/{namespace}/ir_opcode', 
#                                                           self.ir_opcode_listener, qos_profile_sensor_data)
        
#         # Variable initialized to store the latest IR opcode received from the subscription.
#         self.current_opcode = None

#     def ir_opcode_listener(self, msg):
#         """ 
#         Listener function for the 'ir_opcode' subscription.
#         Processes and updates the current opcode based on the received message.
#         """
#         # Extracts and stores the opcode from the received message.
#         self.current_opcode = msg.opcode 

#     def publish_ir_opcode(self):
#         """ 
#         Publishes the current IR opcode as a string message, but is actually 
#         called to directly provide the current opcode reading as an integer. Kris 
#         helped me with this for two hours before I just tried "return". Would
#         love to hear why it is wrong.
#         """
#         # Create a new string message and assign the current opcode
#         # Converting the opcode to a string for standard message formatting.
#         msg = String()
#         msg.data = str(self.current_opcode)

#         # Publish the opcode information to the 'ir_opcode_number' topic.
#         self.opcode_publisher_.publish(msg)
#         # Return the current opcode for use in the function call within the docking() method in roomba_node.
#         return self.current_opcode
	 
