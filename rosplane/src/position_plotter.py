#!/usr/bin/env python
import rospy
from rosplane_msgs.msg import State
from rosplane_msgs.msg import Waypoint
from rosflight_msgs.msg import Status
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class gps_writer:
    def __init__(self):
        self.fig = plt.figure()
        self.gps_sub_ = rospy.Subscriber("/state" , State , self.stateCallback, queue_size=1)
        self.rc_sub_  = rospy.Subscriber("/status", Status, self.rcCallback  , queue_size=1)
        self.wp_sub_  = rospy.Subscriber("/waypoint_path", Waypoint, self.waypointCallback, queue_size=1)
        self.anim = animation.FuncAnimation(self.fig, self.plot_path, interval=1.0)

        # WAYPOINT DATA
        self.wps = [[], []]
        self.last_index = 0
        self.num_wps = 0
        self.plotted_wps = 0
        self.R = 10.0
        self.next_wp = 0
        self.Es = []
        self.Ns = []
        self.wpn = []
        self.wpe = []
        self.theta = np.linspace(0,2.0*np.pi,200)
        self.RC = True
        self.initial = True
        plt.scatter([0],[0])
        plt.axis('equal')
        plt.draw()
        plt.pause(0.0000001)
        # style.use('fivethirtyeight')
        ROS_INFO("POSITION PLOTTER INITIALIZED")

        while not rospy.is_shutdown():
            rospy.spin()

    def plot_path(self, i):
        if self.initial:
            # n = 200.0
            # e = 200.0
            # plt.fill(e + self.R*np.cos(self.theta),n + self.R*np.sin(self.theta),"r")
            # self.wps[0].append(n)
            # self.wps[1].append(e)
            # plt.text(e, n, str(1), fontsize=12)
            #
            # n = 200.0
            # e = -200.0
            # plt.fill(e + self.R*np.cos(self.theta),n + self.R*np.sin(self.theta),"r")
            # self.wps[0].append(n)
            # self.wps[1].append(e)
            # plt.text(e, n, str(2), fontsize=12)
            #
            # n = 40.0
            # e = -200.0
            # plt.fill(e + self.R*np.cos(self.theta),n + self.R*np.sin(self.theta),"r")
            # self.wps[0].append(n)
            # self.wps[1].append(e)
            # plt.text(e, n, str(3), fontsize=12)
            #
            # n = 40.0
            # e = 200.0
            # plt.fill(e + self.R*np.cos(self.theta),n + self.R*np.sin(self.theta),"r")
            # self.wps[0].append(n)
            # self.wps[1].append(e)
            # plt.text(e, n, str(4), fontsize=12)
            #
            # n = 200.0
            # e = 0.0
            # plt.fill(e + self.R*np.cos(self.theta),n + self.R*np.sin(self.theta),"r")
            # self.wps[0].append(n)
            # self.wps[1].append(e)
            # plt.text(e, n, str(5), fontsize=12)
            self.initial = False
        if self.num_wps > self.plotted_wps:
            for i in range(self.plotted_wps,self.num_wps):
                plt.fill(self.wps[1][i] + self.R*np.cos(self.theta), self.wps[0][i] + self.R*np.sin(self.theta),"r")
        if len(self.Ns) > 0:
            var = len(self.Es) - 1
            if self.RC:
                plt.plot(self.Es[self.last_index:],self.Ns[self.last_index:],'b',linewidth=4)
            else:
                plt.plot(self.Es[self.last_index:],self.Ns[self.last_index:],'r',linewidth=4)
            self.last_index = var
            plt.draw()
            plt.pause(0.000000001)
            # Change the colors of the waypoints if you get to it.
            if (len(self.wps) > self.next_wp and len( self.wps[self.next_wp]) == 2):
                if (np.sqrt((distE - self.wps[self.next_wp][1])**2.0 + (distN - self.wps[self.next_wp][0])**2.0) < self.R):
                    plt.fill(self.wps[self.next_wp][1] + self.R*np.cos(self.theta),self.wps[self.next_wp][0]+ self.R*np.sin(self.theta),"b")
                    self.next_wp = self.next_wp + 1

    def stateCallback(self, msg):
        self.Ns.append(msg.position[0])
        self.Es.append(msg.position[1])

    def rcCallback(self, msg):
    	self.RC = msg.rc_override

    def waypointCallback(self, msg):
        self.num_wps = self.num_wps + 1
        self.wps[0].append(msg.w[0])
        self.wps[1].append(msg.w[1])

if __name__ == '__main__':
    rospy.init_node('gps_writer_py', anonymous=True)
    gp = gps_writer()
