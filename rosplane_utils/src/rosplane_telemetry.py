#!/usr/bin/env python

################################################################################
#
# Copyright (c) 2017 BYU MAGICC Lab.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

import rospy
import argparse
from rosplane_msgs.msg import State, State_Lite


class ROSplane_Telemetry(object):

    def __init__(self, rate):
        """
        Compresses state and publishes at a lower rate for human consumption
        over a low data rate link
        """
        rospy.Subscriber("state", State, self.mav_state_callback)
        self.state_lite_pub = rospy.Publisher('telem/state_lite', State_Lite, queue_size=1)
        self.rate = rate
        self.state_lite_msg = State_Lite()
        self.ready = False


    def mav_state_callback(self, msg):
        # get the current mav state
        self.state_lite_msg.header = msg.header
        self.state_lite_msg.position = msg.position
        self.state_lite_msg.Va = msg.Va
        self.state_lite_msg.phi = msg.phi
        self.state_lite_msg.theta = msg.theta
        self.state_lite_msg.chi = msg.chi
        self.state_lite_msg.initial_lat = msg.initial_lat
        self.state_lite_msg.initial_lon = msg.initial_lon
        self.state_lite_msg.initial_alt = msg.initial_alt
        self.ready = True


    def clean_shutdown(self):
        print("\nExiting pointer...")
        return True

    def run(self):
        rate = rospy.Rate(self.rate) # default 20hz
        while not rospy.is_shutdown():
            if self.ready:
                self.state_lite_pub.publish(self.state_lite_msg)
            rate.sleep()

def main():
    """Telemetry node to compress and publish state at a lower rate
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    parser.add_argument('--rate', type=float, default=20.0)
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rosplane_telemetry", log_level=rospy.DEBUG)

    telemetry = ROSplane_Telemetry(args.rate)
    rospy.on_shutdown(telemetry.clean_shutdown)
    telemetry.run()

    print("Done.")

if __name__ == '__main__':
    main()