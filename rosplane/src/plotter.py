#!/usr/bin/env python

import rospy
import numpy as np
import time
import collections as cl
from rosplane_msgs.msg import Controller_Commands
from rosplane_msgs.msg import State
from rosplane_msgs.msg import Controller_Internals
from rosflight_msgs.msg import Status
from rosflight_msgs.msg import Command

import matplotlib.pyplot as plt
import matplotlib.animation as animation


class plotter():

	def __init__(self):

		# RC boolean True if safety pilot has control
		self.RC = True

		# Length of the queues we're using
		self.max_length = 500

		# Font size for the RC/Autopilot indicator
		self.fontsize = 30

		# Commanded states
		self.Va_c_q = cl.deque(maxlen=self.max_length)
		self.h_c_q = cl.deque(maxlen=self.max_length)
		self.chi_c_q = cl.deque(maxlen=self.max_length)
		self.cmd_t_q = cl.deque(maxlen=self.max_length)
		self.cmd2_t_q = cl.deque(maxlen=self.max_length)

		self.theta_c_q = cl.deque(maxlen=self.max_length)
		self.phi_c_q = cl.deque(maxlen=self.max_length)

		# Time associated with each commanded state
		self.state_t_q = cl.deque(maxlen=self.max_length)

		# States
		self.h_q = cl.deque(maxlen=self.max_length)
		self.Va_q = cl.deque(maxlen=self.max_length)
		self.alpha_q = cl.deque(maxlen=self.max_length)
		self.beta_q = cl.deque(maxlen=self.max_length)
		self.phi_q = cl.deque(maxlen=self.max_length)
		self.theta_q = cl.deque(maxlen=self.max_length)
		self.psi_q = cl.deque(maxlen=self.max_length)
		self.chi_q = cl.deque(maxlen=self.max_length)
		self.p_q = cl.deque(maxlen=self.max_length)
		self.q_q = cl.deque(maxlen=self.max_length)
		self.r_q = cl.deque(maxlen=self.max_length)
		self.Vg_q = cl.deque(maxlen=self.max_length)

		# Raw actuator controller_commands
		self.e_q = cl.deque(maxlen=self.max_length)
		self.a_q = cl.deque(maxlen=self.max_length)
		self.r_q = cl.deque(maxlen=self.max_length)
		self.t_q = cl.deque(maxlen=self.max_length)

		# Time associated with each commanded state
		self.actuator_t_q = cl.deque(maxlen=self.max_length)

		# Update the plot at 10Hz
		self.plot_update_interval = 10

		# Define our figure
		self.fig = plt.figure()

		#Add all of the plots
		self.ax1 = self.fig.add_subplot(3,3,1)
		self.ax2 = self.fig.add_subplot(3,3,2)
		self.ax3 = self.fig.add_subplot(3,3,3)
		self.ax4 = self.fig.add_subplot(3,3,4)
		self.ax5 = self.fig.add_subplot(3,3,5)
		self.ax6 = self.fig.add_subplot(3,3,6)
		self.ax7 = self.fig.add_subplot(3,3,7)
		self.ax8 = self.fig.add_subplot(3,3,8)
		self.ax9 = self.fig.add_subplot(3,3,9)

		# Define our animation and the animation function to call
		self.ani = animation.FuncAnimation(self.fig, self.plot_data, interval=self.plot_update_interval)

		# Subscribe to commanded states, states, and RC status
		self.cmd_sub_ = rospy.Subscriber('controller_commands', Controller_Commands, self.cmd_callback, queue_size=10)
		self.cmd2_sub_ = rospy.Subscriber('controller_inners', Controller_Internals, self.cmd2_callback, queue_size=10);
		self.state_sub_ = rospy.Subscriber('state', State, self.state_callback, queue_size=10)
		self.RC_sub_ = rospy.Subscriber('status', Status, self.RC_callback, queue_size=10)
		self.actuator_sub_ = rospy.Subscriber('command', Command, self.actuator_callback, queue_size=10)

		# Show the animation
		plt.show()

		while not rospy.is_shutdown():
			# wait for new messages and call the callback when they arrive
			rospy.spin()


	def plot_data(self, i):
		# Indicate RC override by changing the title
		if self.RC:
			self.fig.suptitle('RC', fontsize=self.fontsize)
		else:
			self.fig.suptitle('Autopilot', fontsize=self.fontsize, color='r')

		self.ax1.clear()
		self.ax1.plot(list(self.cmd_t_q), list(self.Va_c_q), 'r')
		self.ax1.plot(list(self.state_t_q), list(self.Va_q), 'b')
		self.ax1.set_xlim(self.state_t_q[0], self.state_t_q[-1])


		self.ax2.clear()
		self.ax2.plot(list(self.cmd_t_q), list(self.h_c_q), 'r')
		self.ax2.plot(list(self.state_t_q), list(self.h_q), 'b')
		self.ax2.set_xlim(self.state_t_q[0], self.state_t_q[-1])

		self.ax3.clear()
		self.ax3.plot(list(self.cmd_t_q), list(self.chi_c_q), 'r')
		self.ax3.plot(list(self.state_t_q), list(self.chi_q), 'b')
		self.ax3.set_xlim(self.state_t_q[0], self.state_t_q[-1])

		self.ax4.clear()
		self.ax4.plot(list(self.cmd2_t_q), list(self.phi_c_q), 'r')
		self.ax4.plot(list(self.state_t_q), list(self.phi_q), 'b')
		self.ax1.set_xlim(self.state_t_q[0], self.state_t_q[-1])

		self.ax5.clear()
		self.ax5.plot(list(self.cmd2_t_q), list(self.theta_c_q), 'r')
		self.ax5.plot(list(self.state_t_q), list(self.theta_q), 'b')
		self.ax1.set_xlim(self.state_t_q[0], self.state_t_q[-1])

		self.ax6.clear()
		self.ax6.plot(list(self.state_t_q), list(self.psi_q), 'b')
		self.ax1.set_xlim(self.state_t_q[0], self.state_t_q[-1])

		# Rotational Velocity States
		# self.ax7.clear()
		# self.ax7.plot(list(self.state_t_q), list(self.p_q), 'b')
		# self.ax7.set_title('p')
		#
		# self.ax8.clear()
		# self.ax8.plot(list(self.state_t_q), list(self.q_q), 'b')
		# self.ax8.set_title('q')
		#
		# self.ax9.clear()
		# self.ax9.plot(list(self.state_t_q), list(self.r_q), 'b')
		# self.ax9.set_title('r')

		# Raw Actuator Data
		self.ax7.clear()
		self.ax7.plot(list(self.actuator_t_q), list(self.e_q), 'b')
		self.ax1.set_xlim(self.actuator_t_q[0], self.actuator_t_q[-1])

		self.ax8.clear()
		self.ax8.plot(list(self.actuator_t_q), list(self.a_q), 'b')
		self.ax1.set_xlim(self.actuator_t_q[0], self.actuator_t_q[-1])

		self.ax9.clear()
		self.ax9.plot(list(self.actuator_t_q), list(self.t_q), 'b')
		self.ax1.set_xlim(self.actuator_t_q[0], self.actuator_t_q[-1])

		self.ax1.set_ylim(-5, 30.0)
		self.ax2.set_ylim(-5, 200)
		self.ax3.set_ylim(-np.pi, np.pi)
		self.ax4.set_ylim(-np.pi/2, np.pi/2)
		self.ax5.set_ylim(-np.pi/2, np.pi/2)
		self.ax6.set_ylim(-np.pi, np.pi)
		self.ax7.set_ylim(-1.0, 1.0)
		self.ax8.set_ylim(-1.0, 1.0)
		self.ax9.set_ylim(-0.1, 1.0)

		self.ax1.set_title('Va')
		self.ax2.set_title('h')
		self.ax3.set_title('chi')
		self.ax4.set_title('phi')
		self.ax5.set_title('theta')
		self.ax6.set_title('psi')
		self.ax7.set_title('delta_e')
		self.ax8.set_title('delta_a')
		self.ax9.set_title('delta_t')


	def cmd_callback(self, msg):
		# Collect all of the commanded messages
		self.Va_c_q.append(msg.Va_c)
		self.h_c_q.append(msg.h_c)
		self.chi_c_q.append(msg.chi_c)

		# Record the ROS time
		self.cmd_t_q.append(rospy.get_time())
	def cmd2_callback(self, msg):
		# Collect all of the commanded messages
		self.theta_c_q.append(msg.theta_c)
		self.phi_c_q.append(msg.phi_c)

		# Record the ROS time
		self.cmd2_t_q.append(rospy.get_time())

	def state_callback(self, msg):
		# Collect the states
		self.h_q.append(-msg.position[2])
		self.Va_q.append(msg.Va)
		self.alpha_q.append(msg.alpha)
		self.beta_q.append(msg.beta)
		self.phi_q.append(msg.phi)
		self.theta_q.append(msg.theta)
		self.psi_q.append(msg.psi)
		self.p_q.append(msg.p)
		self.q_q.append(msg.q)
		self.r_q.append(msg.r)
		self.Vg_q.append(msg.Vg)

		# We don't want chi to be outside of the +- 2pi range
		myChi = msg.chi
		while abs(myChi) > np.pi:
			myChi -= np.sign(myChi)*2*np.pi

		self.chi_q.append(myChi)

		# Record the ROS time
		self.state_t_q.append(rospy.get_time())

	def RC_callback(self, msg):
		# Record RC override status
		self.RC = msg.rc_override

	def actuator_callback(self, msg):
		self.a_q.append(msg.x)
		self.e_q.append(msg.y)
		self.r_q.append(msg.z)
		self.t_q.append(msg.F)

		# Record the ROS time
		self.actuator_t_q.append(rospy.get_time())

if __name__ == '__main__':
	rospy.init_node('plotter')
	myPlotter = plotter()
