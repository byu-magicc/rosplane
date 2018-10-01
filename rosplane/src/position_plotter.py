#!/usr/bin/env python
import rospy
from rosplane_msgs.msg import State
from rosplane_msgs.msg import Waypoint
from rosplane_msgs.msg import Current_Path
from rosflight_msgs.msg import Status
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class gps_writer:
    def __init__(self):
        self.recieved_path = False;
        self.path_type = 0 # 0 is orbit, 1 is line
        self.r_p = []
        self.q_p = []
        self.c_p = []
        self.r_p.append(0)
        self.r_p.append(0)
        self.r_p.append(0)
        self.q_p.append(0)
        self.q_p.append(0)
        self.q_p.append(0)
        self.c_p.append(0)
        self.c_p.append(0)
        self.c_p.append(0)
        self.new_path = False
        self.old_type = -1
        self.old_r_n = 99999999.0
        self.old_rho = -1.0
        self.old_c_n = 999999999.0
        self.rho_p  = 1
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
        self.fig = plt.figure()
        self.gps_sub_ = rospy.Subscriber("state" , State , self.stateCallback, queue_size=1)
        self.rc_sub_  = rospy.Subscriber("status", Status, self.rcCallback  , queue_size=1)
        # self.wp_sub_  = rospy.Subscriber("waypoint_path", Waypoint, self.waypointCallback, queue_size=1)
        self.path_sub_  = rospy.Subscriber("current_path", Current_Path, self.currentPathCallback, queue_size=1)
        self.anim = animation.FuncAnimation(self.fig, self.plot_path, interval=1.0)

        # WAYPOINT DATA
        plt.scatter([0],[0])
        plt.axis('equal')
        plt.draw()
        plt.pause(0.0000001)
        # style.use('fivethirtyeight')
        # ROS_INFO("POSITION PLOTTER INITIALIZED")

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
        if self.recieved_path and self.new_path == True:
            self.new_path = False
            if self.path_type == 0:
                # plot an orbit
                plt.plot(self.c_p[1] + self.rho_p*np.sin(self.theta), self.c_p[0] + self.rho_p*np.cos(self.theta),"k")
            if self.path_type == 1:
                length = 800.0;
                pe_n = self.r_p[0] + self.q_p[0]*length
                pe_e = self.r_p[1] + self.q_p[1]*length
                plt.plot([self.r_p[1], pe_e], [self.r_p[0], pe_n],"k")
                # plot a line

    def stateCallback(self, msg):
        self.Ns.append(msg.position[0])
        self.Es.append(msg.position[1])

    def rcCallback(self, msg):
    	self.RC = msg.rc_override

    # def waypointCallback(self, msg):
    #     self.num_wps = self.num_wps + 1
    #     self.wps[0].append(msg.w[0])
    #     self.wps[1].append(msg.w[1])
    def currentPathCallback(self, msg):
        self.recieved_path = True;
        if (self.old_type != msg.path_type or self.old_r_n != msg.r[0]  or self.old_rho != msg.rho or self.old_c_n != msg.c[0]):
            self.new_path = True
            self.old_type = msg.path_type
            self.old_r_n = msg.r[0]
            self.old_rho = msg.rho
            self.old_c_n = msg.c[0]
            self.path_type = msg.path_type
            self.r_p[0] = msg.r[0]
            self.r_p[1] = msg.r[1]
            self.r_p[2] = msg.r[2]
            self.q_p[0] = msg.q[0]
            self.q_p[1] = msg.q[1]
            self.q_p[2] = msg.q[2]
            self.c_p[0] = msg.c[0]
            self.c_p[1] = msg.c[1]
            self.c_p[2] = msg.c[2]
            self.rho_p  = msg.rho

if __name__ == '__main__':
    rospy.init_node('gps_writer_py', anonymous=True)
    gp = gps_writer()
