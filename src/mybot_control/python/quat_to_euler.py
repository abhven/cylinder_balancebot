#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64
import numpy as np

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
		self._gain_saturation = 0
		self._old_input = 0
		self._gain = 0

	def update_pid(self, value, setpoint = 0):
		error = setpoint - value
		deriv = self._old_input - value
		self._I_acc += error
		self._old_input = value

		self._gain = self._K_P * error
		self._gain += self._K_I * self._I_acc
		self._gain += self._K_D * deriv

		if abs(self._gain) > self._gain_saturation:
			self._gain *= (self._gain_saturation / abs(self._gain)) 

		return self._gain

	def set_pid_val(self, K_P = None, K_D = None, K_I = None, bias = None, gain_saturation = None , I_acc = None):
		if not K_P == None:
			self._K_P = K_P
		
		if not K_D == None:
			self._K_D = K_D
		
		if not K_I == None:
			self._K_I = K_I
		
		if not bias == None:
			self._bias = bias

		if not I_acc == None:
			self._I_acc = I_acc

		if not gain_saturation == None:
			self._gain_saturation = gain_saturation

#balancing: KP= 1.4 KI 0.1 KD 0.01
		
class PID_controller:
	def __init__(self):
		self._pitch = 0
		self._roll  = 0
		self._yaw   = 0
		self._odo   = 0
		self._vel   = 0
		self._balance_set_pt  = 0
		self._position_set_pt = 100
		self.balance_pid  = PID()
		self.velocity_pid = PID()
		self.position_pid = PID() 
		self.balance_pid.set_pid_val(K_P = 40, K_D = 1000, K_I = 0, bias = 0, gain_saturation = 100 , I_acc = 0)
		self.velocity_pid.set_pid_val(K_P = 2, K_D = 10, K_I = 0.0000, bias = 0, gain_saturation = 0.1 , I_acc = 0)
		self.position_pid.set_pid_val(K_P = 20, K_D = 100, K_I = 0.0000, bias = 0, gain_saturation = 10 , I_acc = 0)
		rospy.init_node('quaternion_to_euler')
		self.pub2 = rospy.Publisher('/mybot/RF_position_controller/command', Float64, queue_size=10)
		self.pub3 = rospy.Publisher('/mybot/LR_position_controller/command', Float64, queue_size=10)
		self.pub4 = rospy.Publisher('/mybot/RR_position_controller/command', Float64, queue_size=10)
		self.pub1 = rospy.Publisher('/mybot/LF_position_controller/command', Float64, queue_size=10)
		self.pub = rospy.Publisher('/mybot/odo', Float64, queue_size=10)
		sub = rospy.Subscriber ('/imu', Imu, self.get_rotation)
		sub_odo = rospy.Subscriber ('/mybot/joint_states', JointState, self.update_odom)


	def update_odom(self, msg):
		# self._vel = np.mean(msg.velocity)
		# self._odo = np.mean(msg.position)
		self._vel = msg.velocity[0]
		self._odo = msg.position[0]
		self.pub.publish(self._odo)
		self.position_controller(self._position_set_pt)

	def get_rotation (self, msg):
		orientation_q = msg.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self._roll, self._pitch, self._yaw) = euler_from_quaternion (orientation_list)
		# print 'P:', self._pitch, '  R:', self._roll, '  Y:', self._yaw
		# self.bang_bang_controller(self._pitch)
		self.balance_controller(self._balance_set_pt)
		

	def bang_bang_controller(self, angle):
		if (angle> 0.01):
			self.motor_command(-1)
		elif (angle < -0.01):
			self.motor_command(1)
		else:
			self.motor_command(0)

	def balance_controller(self, setpoint = 0):
		value = self.balance_pid.update_pid(self._pitch, setpoint)
		self.motor_command(value)

	def velocity_controller(self, setpoint = 0):
		value = self.velocity_pid.update_pid(self._vel, setpoint)
		self._balance_set_pt = -value
		# self.balance_controller(setpoint,  = -value)

	def position_controller(self, setpoint = 0):
		value = self.position_pid.update_pid(self._odo, setpoint)
		self.velocity_controller(setpoint = value)

	def motor_command (self, value):
		self.pub1.publish(value)
		self.pub2.publish(value)
		self.pub3.publish(value)
		self.pub4.publish(value)

if __name__ == "__main__":
	controller = PID_controller()
	rospy.spin()

