#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import IPython
roll = pitch = yaw = 0.0

class PID:
	def __init__(self):
		self._K_P = 0
		self._K_D = 0
		self._K_I = 0
		self._I_acc = 0
		self._bias = 0
		salf._gain_saturation = 0
		self._old_input = 0

	def update_value(error):
		pass
		
class PID_controller:
	def __init__(self):
		self._pitch = 0
		self._roll  = 0
		self._yaw   = 0
		pid = PID() 
		rospy.init_node('quaternion_to_euler')
		self.pub1 = rospy.Publisher('/mybot/LF_position_controller/command', Float64, queue_size=10)
		self.pub2 = rospy.Publisher('/mybot/RF_position_controller/command', Float64, queue_size=10)
		self.pub3 = rospy.Publisher('/mybot/LR_position_controller/command', Float64, queue_size=10)
		self.pub4 = rospy.Publisher('/mybot/RR_position_controller/command', Float64, queue_size=10)
		sub = rospy.Subscriber ('/imu', Imu, self.get_rotation)

	def get_rotation (self, msg):
		orientation_q = msg.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self._roll, self._pitch, self._yaw) = euler_from_quaternion (orientation_list)
		print 'P:', self._pitch, '  R:', self._roll, '  Y:', self._yaw
		if (self._pitch> 0.01):
			self.motor_command(-1)
		elif (self._pitch < -0.01):
			self.motor_command(1)

	def control_loop(input):

	def motor_command (self, value):
		self.pub1.publish(value)
		self.pub2.publish(value)
		self.pub3.publish(value)
		self.pub4.publish(value)		

if __name__ == "__main__":
	controller = PID_controller()
	rospy.spin()

