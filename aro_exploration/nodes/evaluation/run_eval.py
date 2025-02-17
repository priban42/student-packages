#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Pose
import lxml.etree as ET
import numpy as np
import rospkg
import os
from uuid import uuid4
import roslaunch
import tf
import csv
from datetime import datetime
import re

def clamp(minimum, x, maximum):
    return max(minimum, min(x, maximum))

class Evaluathor():

    RUN_SINGLE = "single"
    RUN_MANUAL = "manual"
    RUN_AUTO = "auto"

    MARKER_TOPIC = "/relative_marker_pose"
    GT_ODOM_TOPIC = "/ground_truth_odom"

    def __init__(self):
        rospack = rospkg.RosPack()
        self.aro_sim_pkg = rospack.get_path("aro_sim")  # aro_sim package path
        self.aro_exp_pkg = rospack.get_path("aro_exploration")  # aro_sim package path
        self.outFolder = os.path.expanduser("~/aro_evaluation")
        if not os.path.isdir(self.outFolder):
            os.mkdir(self.outFolder)

        self.requestedMap = rospy.get_param("~map_name", "aro_eval_1")  # name of the requested world
        self.requestedMarker = rospy.get_param("~marker_config", 1)
        self.spawnMode = rospy.get_param("~spawn_mode", "fixed")  # random starting position
        self.runMode = rospy.get_param("~run_mode", self.RUN_MANUAL)  # if run o multiple maps, how are the maps being switched
        self.timeLimit = rospy.get_param("~time_limit", 120)  # go to next map after X seconds if run mode is auto
        self.localization_visualize = rospy.get_param("~localization_visualize", False)
        self.rviz = rospy.get_param("~rviz", True)
        self.gui = rospy.get_param("~gui", False)

        self.sim_launch = None  # variable to hold the simulator launcher
        self.mapIndex = -1  # for running on multiple maps from a list (i.e. requestedMap is a list)
        self.mapFrame, self.odomFrame = "map", "odom"
        self.initPoses = None
        self.publishedMarkerPose = None
        self.startTime = None
        self.stopSimVar = False
        self.distanceDriven = 0.0

        self.stamp = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")

        # compute data fields
        self.dataFields = ["map", "marker", "score"]        
        self.data = []

        self.markerListener = rospy.Subscriber(self.MARKER_TOPIC, PoseStamped, self.markerUpdate_cb, queue_size=1)
        self.gt_odom_sub = rospy.Subscriber(self.GT_ODOM_TOPIC, Odometry, self.process_gt_odom, queue_size=1)
        self.gt_odom = None

        self.stopSimService = rospy.Service('stop_simulation', SetBool, self.stopSimulation_cb)

        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.timer_cb)

        self.marker_score_pub = rospy.Publisher('/marker_score', Float32, queue_size=1)
        self.marker_error_pub = rospy.Publisher('/marker_error', Float32, queue_size=1)

        # rospy.loginfo("Setting up evaluation:\n\t{} map mode\n\tmap(s): {}\n\t{} run mode\n\t{} spawn mode".format(
        #     "multi" if self.multiEval else "single",
        #     self.requestedMap,
        #     self.runMode,
        #     self.spawnMode
        # ))

    def __formatTime(self, secs):
        """Splits the time in seconds into hours, minutes, and seconds

        Arguments:
            secs {int} -- time in seconds

        Returns:
            tuple
        """
        h = int(secs / 3600)
        r = secs - h * 3600
        m = int(r / 60)
        r -= m * 60
        return h, m, int(r)

    def __generateRandomSpawn(self):
        """ Generates random x & y position for the robot.
        """
        return np.random.randn(2) / 2

    def markerUpdate_cb(self, msg):
        self.publishedMarkerPose = msg.pose

    def process_gt_odom(self, msg):
        if self.gt_odom is not None:
            prev_pose = np.array([self.gt_odom.position.x, self.gt_odom.position.y])
            new_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
            dist = np.linalg.norm(new_pose - prev_pose)
            self.distanceDriven += dist
        else:
            self.startingPose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        self.gt_odom = msg.pose.pose
        if self.startTime is None:
            self.startTime = rospy.Time.now()

    def stopSimulation_cb(self, req):
        self.stopSimVar = req.data
        return SetBoolResponse(True, "Stopping simulation.")

    def timer_cb(self, tim):
        try:
            # compare the received map with the GT
            self.compareData()
        except Exception as e:
            rospy.logerr(e)

    def compareData(self):

        dist = 999
        self.robotScore = 0.0
        if self.publishedMarkerPose is not None:
            dist = np.linalg.norm([self.publishedMarkerPose.position.x-self.gt_marker[0], self.publishedMarkerPose.position.y-self.gt_marker[1]])
            self.markerScore = clamp(0.0, 1.0 - clamp(0.0, 2*(dist-0.25),1.0),1.0)
            self.marker_score_pub.publish(self.markerScore)
            self.marker_error_pub.publish(dist*100.0)
            rospy.loginfo("Marker distance from reference is {}, giving score: {}.".format(dist, np.round(self.markerScore,2)))
        else:
            self.markerScore = 0.0
        
        if self.gt_odom is not None:
            robotDistFromStart = np.linalg.norm([self.gt_odom.position.x - self.startingPose[0], self.gt_odom.position.y - self.startingPose[1]])
            if robotDistFromStart <= 0.25:
                self.robotScore = 1
            else:
                self.robotScore = clamp(0.0,1.0 - clamp(0.0,2*(robotDistFromStart-0.25),1.0),1.0)
            rospy.loginfo("Robot distance from start is {}, giving score: {}.".format(robotDistFromStart, np.round(self.robotScore,2)))
        else: 
            robotDistFromStart = 999

        if self.markerScore > 0.0:
            self.finalScore = np.round(self.markerScore + self.robotScore,2)
        else:
            self.finalScore = 0.0

        rospy.loginfo("Combined score: {}.".format(self.finalScore))

        # compute time
        if self.startTime is None:
            currentTime = rospy.Time.from_sec(0).secs
        else:
            currentTime = (rospy.Time.now() - self.startTime).secs

        if currentTime >= self.timeLimit or self.stopSimVar:
                
            rospy.loginfo("Evaluation ending.")
            
            self.markerScore = np.round(self.markerScore,2)
            self.robotScore = np.round(self.robotScore,2)
            if self.markerScore > 0:
                self.finalScore = np.round(self.markerScore + self.robotScore,2)
            else:
                self.finalScore = 0
            
            rospy.loginfo("Time limit reached or localization finished, stopping.")

            rospy.loginfo("Final robot distance from start is {}, giving score: {}.".format(robotDistFromStart, self.robotScore))
            rospy.loginfo("Final marker distance from reference is {}, giving score: {}.".format(dist, self.markerScore))
            rospy.loginfo("Final combined score: {}.".format(self.finalScore))
            
            self.stopSim()
            rospy.signal_shutdown("End of evaluation.")
            return


    def __loadMarker(self, mapName):
        self.mapName = mapName
        # load map GT
        markerFile = os.path.join(self.aro_exp_pkg, 'data', 'evaluation', 'marker_gt',  "{}".format(self.mapName), "{}.txt".format(self.requestedMarker))

        if not os.path.exists(markerFile):
            e_msg = "Ground truth marker file {} for the world {} was not found!".format(self.requestedMarker, self.mapName)
            rospy.logfatal(e_msg)
            raise IOError(e_msg)

        # launchFile = os.path.join(self.aro_sim_pkg, "launch", "turtlebot3.launch")
        # tree = ET.parse(launchFile)
        # root = tree.getroot()

        self.gt_marker = np.loadtxt(markerFile)

    def stopSim(self):
        self.sim_launch.shutdown()

    def restart(self):
        if self.sim_launch is not None:
            self.stopSim()

        self.stopSimVar = False
        self.startTime = None

        self.__loadMarker(self.requestedMap)
        self.startingPose = [0,0]

        spawn_command = []

        # Launch the simulator
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_command = [self.aro_exp_pkg,
                          "aro_exploration_sim.launch",
                          "world:={}".format(self.mapName),
                          "marker_config:={}".format(self.requestedMarker),
                          "ground_truth:=true",
                          "mr_use_gt:=false",
                          "rviz:=" + ("true" if self.rviz else "false"),
                          "gui:=" + ("true" if self.gui else "false"),
                          "localization_visualize:=" + ("true" if self.localization_visualize else "false"),
                          "run_mode:=eval"
                          ]
        launch_command += spawn_command

        sim_launch_file = os.path.join(self.aro_exp_pkg, "launch","exploration","aro_exploration_sim.launch")
        sim_launch_args = launch_command[2:]
        launch_files = [(sim_launch_file, sim_launch_args)]
        self.sim_launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        rospy.loginfo(self.sim_launch.roslaunch_files)
        # self.sim_launch.force_log = True
        self.sim_launch.start()
        rospy.loginfo("ARO SIM launched.")

        self.startTime = None

    def saveStatistics(self):
        # save results
        resultFilePath = os.path.join(self.outFolder, "results_{}.csv".format(self.stamp))
        rospy.loginfo("Saving results to : {}".format(resultFilePath))

        currentTime = (rospy.Time.now() - self.startTime).secs if self.startTime is not None else 0

        with open(resultFilePath, "w") as f:
            f.write("map, marker, score, return_score, marker_score, marker_found, distance_driven, elapsed_time \n")
            f.write("{}, {}, {}, {}, {}, {}, {:.1f}, {} \n".format(
                self.mapName, self.requestedMarker, self.finalScore, self.robotScore, self.markerScore,
                self.publishedMarkerPose is not None, self.distanceDriven, currentTime))

    def run(self):
        self.restart()
        try:
            rospy.spin()  # spin
        finally:
            self.saveStatistics()
            self.sim_launch.shutdown()  # stop the simulator


if __name__ == "__main__":
    rospy.init_node("evaluathor")

    evaluathor = Evaluathor()
    evaluathor.run()
