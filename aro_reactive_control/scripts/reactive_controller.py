#!/usr/bin/env python3
"""
Simple reactive controller for turtlebot robot.
"""

import rospy
import numpy as np  # you probably gonna need this
from geometry_msgs.msg import Twist, Vector3
from aro_msgs.msg import SectorDistances
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from std_srvs.srv import Trigger, SetBool, TriggerResponse, TriggerRequest
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import numpy as np

# TODO HW 01 and HW 02: add necessary imports

class ReactiveController():

    def __init__(self):
        rospy.loginfo('Initializing node')
        rospy.init_node('reactive_controller')
        self.initialized = False
        self.active = False
        self.activation_time = 0

        # TODO HW 01: register listener for laser scan message
        self.scan_subscriber = rospy.Subscriber("/scan", LaserScan, queue_size=10, callback=self.scan_cb)

        # TODO HW 02:publisher for "/cmd_vel" topic (use arg "latch=True")
        self.twist_publisher = rospy.Publisher("/cmd_vel", Twist, latch=True)

        # TODO HW 01: register publisher for "/reactive_control/sector_dists" topic
        self.reactive_publisher = rospy.Publisher("/reactive_control/sector_dists", SectorDistances)
        # publishing minimum distances as message of type aro_msgs/SectorDistances

        # TODO HW 02: create proxy for the mission evaluation service and wait till the start service is up
        rospy.wait_for_service('/reactive_control/evaluate_mission')
        self.evaluate_missin_service = rospy.ServiceProxy('/reactive_control/evaluate_mission', Trigger)
        # rospy.wait_for_service('/reactive_control/activate')
        # TODO HW 02: create service server for mission start
        rospy.Service('/reactive_control/activate', SetBool, self.activate_cb)

        # TODO: you are probably going to need to add some variables

        # TODO HW 02: add timer for mission end checking
        self.time_subscriber = rospy.Subscriber("/clock", Clock, queue_size=10, callback=self.timer_cb)
        self.initialized = True

        rospy.loginfo('Reactive controller initialized. Waiting for data.')

    def timer_cb(self, event): 
        """
        Callback function for timer.

        :param event (rospy TimerEvent): event handled by the callback 
        """
       
        if not self.initialized:
            return

        if self.active:
            if self.activation_time == 0:
                self.activation_time = event.clock.secs
            # if event.clock.secs - self.activation_time > 40:
            #     resp = self.evaluate_missin_service()
            #     self.active = False
        # TODO HW 02: Check that the active time had elapsed and send the mission evaluation request


    def activate_cb(self, req: SetBoolRequest) -> SetBoolResponse: 
        """
        Activation service callback.

        :param req: obtained ROS service request 

        :return: ROS service response 
        """

        rospy.loginfo_once('Activation callback entered')

        # TODO HW 02: Implement callback for activation service
        # Do not forget that this is a service callback, so you have to return a SetBoolResponse object with .success
        # set to True.
        self.active = req.data
        ret = SetBoolResponse()
        ret.success = True
        return ret

    def scan_cb(self, msg):
        """
        Scan callback.

        :param msg: obtained message with data from 2D scanner of type ??? 
        """
        rospy.loginfo_once('Scan callback entered')

        if not self.initialized: 
            return

        # TODO HW 01: Implement callback for 2D scan, process and publish required data
        # rospy.loginfo(len(msg.ranges))
        # rospy.loginfo(min(msg.ranges))
        right = min(msg.ranges[-90:-30])
        front = min(msg.ranges[-30:] + msg.ranges[:30])
        left = min(msg.ranges[30:90])
        response = SectorDistances(distance_front=front, distance_left=left, distance_right=right)
        self.reactive_publisher.publish(response)
        # TODO HW 02: Add robot control based on received scan
        if self.active:
            vel = 0.2
            vel = min((front - 0.1)*0.8, 0.8)
            ang = (1.2 + abs(vel))
            if abs(ang) < 0.1:
                ang= 0.1
            if front > 0.6:
                self.apply_control_inputs(vel, 0)
            elif left > right:
                self.apply_control_inputs(vel, ang)
            else:
                self.apply_control_inputs(vel, -ang)
        else:
            self.apply_control_inputs(0, 0)


    def apply_control_inputs(self, velocity: float, angular_rate: float):
        """
        Applies control inputs.

        :param velocity: required forward velocity of the robot 
        :param angular_rate: required angular rate of the robot 
        """
        # TODO HW 02: publish required control inputs 
        response = Twist()
        response.angular = Vector3(angular_rate, angular_rate, angular_rate)
        response.linear = Vector3(velocity, 0, 0)
        self.twist_publisher.publish(response)

if __name__ == '__main__':
    rc = ReactiveController()
    rospy.spin()
