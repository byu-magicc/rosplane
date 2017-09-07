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
from rosplane_msgs.msg import Controller_Commands
from math import pi

def tunning_command_publisher():
    pub = rospy.Publisher('controller_commands', Controller_Commands, queue_size=10)
    rospy.init_node('roll_tunning')
    rate = rospy.Rate(0.5) # 0.5hz
    roll_angle = rospy.get_param('~roll_angle', 40*pi/180)
    airspeed = rospy.get_param('~airspeed', 12)
    toggle = 1.
    while not rospy.is_shutdown():
        toggle *= -1
        msg = Controller_Commands()
        msg.Va_c = airspeed
        msg.aux_valid = True
        msg.aux_state = msg.ROLL_PITCH_VA_TUNING
        msg.aux = [0, toggle*roll_angle, 0, 0]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        tunning_command_publisher()
    except rospy.ROSInterruptException:
        pass
