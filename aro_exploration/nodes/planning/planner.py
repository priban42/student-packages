#!/usr/bin/env python3

from __future__ import absolute_import, division, print_function
import rospy
import numpy as np
from queue import PriorityQueue
import itertools
from typing import List
from scipy.ndimage import grey_dilation
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import Pose2D, Pose, PoseStamped, Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from aro_msgs.srv import PlanPath, PlanPathRequest, PlanPathResponse
import tf2_ros
from aro_exploration.utils import map_to_grid_coordinates, grid_to_map_coordinates, get_circular_dilation_footprint


"""
Here are imports that you will most likely need. However, you may wish to remove or add your own import.
"""

class PathPlanner:
    def __init__(self):
        # Initialize the node
        rospy.init_node("path_planner")

        self.robot_grid_position = None
        self.grid = None
        self.grid_info = None
        self.origin_pos = None
        self.grid_resolution = None
        # Helper variable to determine if grid was received at least once
        self.grid_ready = False

        self.map_frame = rospy.get_param("~map_frame", "icp_map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        self.robot_diameter = float(rospy.get_param("~robot_diameter", 0.6))
        self.occupancy_threshold = int(rospy.get_param("~occupancy_threshold", 25))

        # You may wish to listen to the transformations of the robot
        self.tf_buffer = tf2_ros.Buffer()
        # Use the tfBuffer to obtain transformation as needed
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publishers for visualization
        self.path_vis_pub = rospy.Publisher('path', Path, queue_size=1)
        self.start_and_goal_vis_pub = rospy.Publisher('start_and_goal', MarkerArray, queue_size=1)

        # The services will be set up when the first occupancy grid message comes
        self.plan_publish_service = None
        self.plan_service = None

        # Subscribe to grid
        self.grid_subscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)

        rospy.loginfo('Path planner initialized.')

    def plan_path_and_publish(self, request: PlanPathRequest):
        response = self.plan_path(request)
        self.publish_path(response.path)
        return response
    @staticmethod
    def hash_np(arr, gap=10000):
        return np.sum((arr + 500)*(gap**np.arange(arr.shape[0])))

    def plan_path(self, request: PlanPathRequest) -> PlanPathResponse:
        """ Plan and return path from the requrested start position to the requested goal """

        # Get the position of the goal (real-world)
        start_position = map_to_grid_coordinates(np.array([request.start.x, request.start.y]), self.grid_info)
        goal_position = map_to_grid_coordinates(np.array([request.goal.x, request.goal.y]), self.grid_info)

        # Visualize start and goal
        self.publish_start_and_goal_vis(request.start, request.goal)

        # check that start and goal positions are inside the grid. 
        if start_position[0] < 0 or start_position[1] < 0 or start_position[0] >= self.grid.shape[1] or start_position[1] >= self.grid.shape[0]:
            rospy.logwarn(
                "WARNING: start grid position is outside the grid. [cell_x,cell_y]=[{:f},{:f}], grid shape [w={:f},h={:f}]. "
                "Returning an empty trajectory.".format(
                    start_position[0], start_position[1], self.grid.shape[1], self.grid.shape[0]))
            response = PlanPathResponse([])
            return response

        if goal_position[0] < 0 or goal_position[1] < 0 or goal_position[0] >= self.grid.shape[1] or goal_position[1] >= self.grid.shape[0]:
            rospy.logwarn(
                "WARNING: goal grid position is outside the grid. [cell_x,cell_y]=[{:f},{:f}], grid shape [w={:f},h={:f}]. "
                "Returning an empty trajectory.".format(
                    goal_position[0], goal_position[1], self.grid.shape[1], self.grid.shape[0]))
            response = PlanPathResponse([])
            return response

        path = []
        dilation_footprint = get_circular_dilation_footprint(self.robot_diameter, self.grid_resolution)
        # dilation_footprint_wide = get_circular_dilation_footprint(self.robot_diameter, self.grid_resolution)
        dilation_footprint_wide = dilation_footprint
        # print(f"{dilation_footprint.shape=}")
        # print(f"{dilation_footprint_wide.shape=}")
        # print(f"{dilation_footprint_wide.shape=}")
        # print(f"{goal_position[1] - dilation_footprint_wide.shape[1] // 2}:{goal_position[1] + (dilation_footprint_wide.shape[1] + 1) // 2}")
        # print(f"{goal_position[0] - dilation_footprint_wide.shape[0] // 2}:{goal_position[0] + (dilation_footprint_wide.shape[0] + 1) // 2}")
        # print(f"{(1-dilation_footprint_wide.astype(int))=}")
        # print(f"{dilation_footprint_wide.shape=}")

        dilated_grid = np.copy(self.grid)
        dilated_grid[dilated_grid != 0] = -1
        dilated_grid = -grey_dilation(-dilated_grid, footprint=dilation_footprint)
        # print(f"{dilated_grid.shape=}")
        # print(f"{(dilated_grid[goal_position[1] - dilation_footprint_wide.shape[1] // 2:goal_position[1] + (dilation_footprint_wide.shape[1] + 1) // 2,goal_position[0] - dilation_footprint_wide.shape[0] // 2:goal_position[0] + (dilation_footprint_wide.shape[0] + 1) // 2]).shape=}")
        # print(f"{(dilated_grid[start_position[1] - dilation_footprint_wide.shape[1] // 2:start_position[1] + (dilation_footprint_wide.shape[1] + 1) // 2,start_position[0] - dilation_footprint_wide.shape[0] // 2:start_position[0] + (dilation_footprint_wide.shape[0] + 1) // 2]).shape=}")

        dilated_grid[
        goal_position[1] - dilation_footprint_wide.shape[1] // 2:goal_position[1] + (dilation_footprint_wide.shape[1] + 1) // 2,
        goal_position[0] - dilation_footprint_wide.shape[0] // 2:goal_position[0] + (dilation_footprint_wide.shape[0] + 1) // 2] *=0#(1-dilation_footprint_wide.astype(int))
        dilated_grid[
        start_position[1] - dilation_footprint_wide.shape[1] // 2:start_position[1] + (dilation_footprint_wide.shape[1] + 1) // 2,
        start_position[0] - dilation_footprint_wide.shape[0] // 2:start_position[0] + (dilation_footprint_wide.shape[0] + 1) // 2] *=0#(1-dilation_footprint_wide.astype(int))
        current_t = start_position
        # closed = set()
        closed = np.zeros((1000, 1000))
        parents = {}
        pq = PriorityQueue()
        counter = itertools.count()
        pq.put((np.linalg.norm(start_position-goal_position), -1, start_position))
        # closed.add(self.hash_np(start_position))
        closed[start_position[0], start_position[1]] = 1
        parents[self.hash_np(start_position)] = None
        print(f"planner: planning started")
        for i in range(100000):
            # print(f"bp planner loop start {pq.qsize()=}")
            if i>0 and i%500 == 0:
                print(f"planner iteration: {i=}")
            if pq.qsize() == 0:
                print(f"{pq.qsize()=}")
                return PlanPathResponse([])
            cost, _, current_t = pq.get()
            # print(f"bp planner a: {cost=}, {current_t=}")
            # print(cost, current_t)
            if np.alltrue(current_t == goal_position):
                # print("bp planner b")
                break
            # print("bp planner 1:")
            for x in [-1, 0, 1]:
                # print("bp planner c:")
                for y in [-1, 0, 1]:
                    # print("bp planner d:")
                    if not ( x == 0 and y == 0):
                        # print(f"bp planner 2:")
                        t_diff = np.array([x, y])
                        # print(f"bp planner 3: {t_diff=}")
                        next_t = current_t+t_diff
                        # print(f"bp planner 4: {next_t}")
                        # if self.hash_np(next_t) not in closed:
                        if closed[next_t[0], next_t[1]] == 0:
                            # print("bp planner 5:")
                            if (dilated_grid[next_t[1], next_t[0]] == 0 and
                                    dilated_grid[next_t[1], current_t[0]] == 0 and
                                    dilated_grid[current_t[1], next_t[0]] == 0):
                                # print("bp planner 6:")
                                G = cost + np.linalg.norm(t_diff) - np.linalg.norm(goal_position - current_t)
                                H = np.linalg.norm(goal_position - next_t)
                                pq.put((G+H, next(counter), next_t))
                                parents[self.hash_np(next_t)] = current_t
                                # print("bp planner 7:")
                            # print("bp planner 8:")
                        # closed.add(self.hash_np(next_t))
                        # print("bp planner 9:")
                        closed[next_t[0], next_t[1]] = 1
                        # print("bp planner 10:")
                    # print("bp planner 11:")
                # print("bp planner 12:")
            # print("bp planner 13:")
        # print("bp planner 14")
        if not np.alltrue(current_t == goal_position):
            print("planner: not np.alltrue(current_t == goal_position), returning PlanPathResponse([])")
            return PlanPathResponse([])
        print("planner: exploring iterations done")

        # TODO: Copy the occupancy grid into some temporary variable and inflate the obstacles. Use the kernel from get_circular_dilation_footprint()

        # TODO: Make sure you take into account unknown grid tiles as non-traversable and also inflate those.

        # TODO: Compute the path using A* and a euclidean distance heuristic. You can move diagonally, but check the safety of the 4 cells 

        # TODO: Grid tiles could and should be explored repeatedly with updated cost.

        # TODO: Hint: utilize look-up grids for bools and cost values, storing visited points in list is SLOW. Also consider using an ordered queue (e.g. PriorityQueue)

        # TODO: Submitting badly optimized code can lead to failed evaluations during semestral work simulation.
        i = 0
        while True:
            path.append(current_t)
            current_t = parents[self.hash_np(current_t)]
            if current_t is None:
                break
            if i > 1000:
                print("planner: path too long")
                path = [None]
                break
            i += 1
        print("planner: path generated")
        path = path[::-1]
        # Convert the path (list of grid cell coordinates) into a service response with a list of /icp_map frame poses 
        real_path = [Pose2D(pos[0], pos[1], 0) for pos in
                     [grid_to_map_coordinates(waypoint, self.grid_info) for waypoint in path]]
        response = PlanPathResponse(real_path)
        return response

    def extract_grid(self, msg):
        width, height, self.grid_resolution = msg.info.width, msg.info.height, msg.info.resolution
        self.origin_pos = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.grid_info = msg.info
        self.grid = np.reshape(msg.data, (height, width))

    def grid_cb(self, msg):
        self.extract_grid(msg)
        if not self.grid_ready:
            # Create services
            self.plan_publish_service = rospy.Service('plan_path_publish', PlanPath, self.plan_path_and_publish)
            self.plan_service = rospy.Service('plan_path', PlanPath, self.plan_path)
            self.grid_ready = True

    def publish_path(self, path_2d: List[Pose2D]):
        msg = Path()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = rospy.get_rostime()
        for waypoint in path_2d:
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = 0
            msg.poses.append(pose)

        rospy.loginfo("Publishing plan.")
        self.path_vis_pub.publish(msg)

    def publish_start_and_goal_vis(self, start: Pose2D, goal: Pose2D):
        msg = MarkerArray()
        m_start = Marker()
        m_start.header.frame_id = self.map_frame
        m_start.id = 1
        m_start.type = 2
        m_start.action = 0
        m_start.pose = Pose()
        m_start.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        m_start.pose.position = Point(start.x, start.y, 0.0)
        # m_start.points.append(Point(start.x, start.y, 0.0))
        m_start.color.r = 1.0
        m_start.color.g = 0.0
        m_start.color.b = 0.0
        m_start.color.a = 0.8
        m_start.scale.x = 0.1
        m_start.scale.y = 0.1
        m_start.scale.z = 0.001
        msg.markers.append(m_start)

        # goal marker
        m_goal = Marker()
        m_goal.header.frame_id = self.map_frame
        m_goal.id = 2
        m_goal.type = 2
        m_goal.action = 0
        m_goal.pose = Pose()
        m_goal.pose.orientation = Quaternion(0.0, 0.0, 0.0, 1.0)
        m_goal.pose.position = Point(goal.x, goal.y, 0.0) 
        # m_start.points.append(Point(start.x, start.y, 0.0))
        m_goal.color.r = 0.0
        m_goal.color.g = 1.0
        m_goal.color.b = 0.0
        m_goal.color.a = 0.8
        m_goal.scale.x = 0.1
        m_goal.scale.y = 0.1
        m_goal.scale.z = 0.001
        msg.markers.append(m_goal)
        rospy.loginfo("Publishing start and goal markers.")
        self.start_and_goal_vis_pub.publish(msg)


if __name__ == "__main__":
    pp = PathPlanner()

    rospy.spin()
