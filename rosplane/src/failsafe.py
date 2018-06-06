#!/usr/bin/env python

import subprocess, rospy, time
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from rosflight_msgs.msg import Status

ROUTER = '10.10.10.2'
CMD = ['fping', '-r0', ROUTER] # use fping to ping the router ip address. -r0 retries 0 times to make it fail faster


# This will test for interop and RC failures
# Interop Connection Test: ping the router and use the system time to keep track of downtime
# - Is called in while loop at a rate of 10 Hz
# RC Connection Test: listens for failsafe status in state topic.
# - Is automatically called each time the state topic updates

class failsafe():
    def __init__(self):
        self.wasRCAlive = True
        self.calledRTH = False
        self.calledTerminate = False
        self.RCDiedTime = 0

        self.wasInteropAlive = True
        self.interopDiedTime = 0

        # Subscribe to state to see the RC status inputs
        self.RC_sub = rospy.Subscriber("state", Status, self.statusCallback)
    
    def testInterop(self):
        """Tests the interop system for downtime"""
        if not self.isInteropAlive(): # If interop is not alive
            if self.wasInteropAlive: # if inteorp was alive last check
                self.interopDiedTime = time.time() # get the time it went down
                self.wasInteropAlive = False
            else:
                elapsedTime = self.interopDiedTime - time.time()
                self.testElapsedTime(elapsedTime)
        else: # Not in failsafe
            self.wasInteropAlive = True
        

    def statusCallback(self, msg):
        """Status message callback. Tests for RC downtime. \n\nIs automatically called when the state topic is updated"""
        if msg.failsafe: # If in Failsafe mode
                if self.wasRCAlive: # If RC was alive last check
                    self.RCDiedTime = msg.header.stamp # get the time it went down
                    self.wasRCAlive = False
                else:
                    elapsedTime = self.RCDiedTime - msg.header.stamp
                    self.testElapsedTime(elapsedTime)
        else: # Not in failsafe
            self.wasRCAlive = True


    def testElapsedTime(self, elapsedTime):
        """Tests a given elapsed time to see if a failsafe should be called
        
        Keyword arguments:
        elapsedTime -- The time that has elapsed, in seconds
        """
        if elapsedTime > 30 and not self.calledRTH: # If in failsafe for more than 30s and we haven't already told it to RTH
            self.calledRTH = True
            self.returnToHome()
        if elapsedTime > 180 and not self.calledTerminate: # If in failsafe for 3 mins and haven't already told it to terminate
            self.calledTerminate = True
            self.terminateFlight()
    

    def resetFailsafe(self):
        """Resets previously called RTH (but not terminate) failsafes if RC and Interop are now alive"""
        if self.wasRCAlive and self.wasInteropAlive: # if both were alive last check
            if self.calledRTH: # if RTH had been called
                try:
                    rospy.ServiceProxy('resume_path', Trigger)
                except rospy.ServiceException as e:
                    print("resume_path service called failed: ", e)
            if self.calledTerminate: # if terminate had been called
                pass # ONLY CALL IF WE KNOW IT'S STILL IN THE AIR. Leaving commented until we know how.
                # try:
                #     rospy.ServiceProxy('save_flight', Trigger)
                # except rospy.ServiceException as e:
                #     print("save_flight service called failed: ", e)
            
            # Reset all of the default values
            self.calledRTH = False
            self.calledTerminate = False
    

    def isInteropAlive(self):
        """One time check to see if the interop system is alive."""
        try:
            subprocess.check_output(CMD)
        except subprocess.CalledProcessError:
            return False
        return True


    def returnToHome(self):
        """Calls the /return_to_home rosservice"""
        try:
            rospy.ServiceProxy('return_to_home', Trigger)
        except rospy.ServiceException as e:
            print("Return to home service called failed: ", e)


    def terminateFlight(self):
        """Calls the /terminate_flight rosservice"""
        try:
            rospy.ServiceProxy('terminate_flight', Trigger)
        except rospy.ServiceException as e:
            print("terminate_flight ervice called failed: ", e)

if __name__ == '__main__':
    rospy.init_node('Status_Watcher')

    failsafeObj = failsafe()

    rospy.wait_for_service('return_to_home')
    rospy.wait_for_service('terminate_flight')

    r = rospy.Rate(10) # 10Hz

    while not rospy.is_shutdown():
        failsafeObj.testInterop()
        r.sleep()