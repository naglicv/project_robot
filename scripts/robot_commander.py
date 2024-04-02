#! /usr/bin/env python3
# Mofidied from Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from enum import Enum
import time

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Quaternion, PoseStamped, PoseWithCovarianceStamped, PointStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import Spin, NavigateToPose
from turtle_tf2_py.turtle_tf2_broadcaster import quaternion_from_euler
from visualization_msgs.msg import Marker

from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration as rclpyDuration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from rclpy.service import Service

#from geometry_msgs.srv import SetPoint

import math
#import pyttsx3
import tf_transformations



class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3

amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

qos_profile = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

class RobotCommander(Node):

    def __init__(self, node_name='robot_commander', namespace=''):
        super().__init__(node_name=node_name, namespace=namespace)
        
        self.pose_frame_id = 'map'
        
        # Flags and helper variables
        self.goal_handle = None
        self.result_future = None
        self.feedback = None
        self.status = None
        self.initial_pose_received = False
        self.is_docked = None

        # ROS2 subscribers
        self.create_subscription(DockStatus,
                                 'dock_status',
                                 self._dockCallback,
                                 qos_profile_sensor_data)
        
        self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                              'amcl_pose',
                                                              self._amclPoseCallback,
                                                              amcl_pose_qos)
        
        self.people_marker_sub = self.create_subscription(Marker,
                                                          'people_marker',
                                                          self._peopleMarkerCallback,
                                                          QoSReliabilityPolicy.BEST_EFFORT)
        
        self.people_marker_sub = self.create_subscription(Marker,
                                                          'breadcrumbs',
                                                          self.breadcrumbs_callback,
                                                          QoSReliabilityPolicy.BEST_EFFORT)
        
        # ROS2 publishers
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        self.face_pub = self.create_publisher(PointStamped,
                                                      'face',
                                                      qos_profile)
        
    
        
        
        # ROS2 Action clients
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.spin_client = ActionClient(self, Spin, 'spin')
        self.undock_action_client = ActionClient(self, Undock, 'undock')
        self.dock_action_client = ActionClient(self, Dock, 'dock')
        #self.mark_point_client = self.create_client(SetPoint, 'mark_point')
        #self.transform_point_client = self.create_client(SetBool, 'transform_point')

        self.latest_people_marker_pose = None
        self.latest_people_marker_robot_pose = None
        self.current_pose = None
        self.detected_faces = []
        self.approached_faces_num = 0
        self.new_face_detected = False
        self.start_detecting = False

        self.get_logger().info(f"Robot commander has been initialized!")

    # def face_callback(self, msg):
    #     detected_face = msg.point
    #     #self.new_face_detected = True
    #     #self.detected_faces.append(detected_face)
    #     new = True
    #     for face in self.detected_faces:
    #         if abs(detected_face.x - face.x) < 1.0 and abs(detected_face.y - face.y) < 2.5:
    #             self.get_logger().info(f"Face already marked")
    #             new = False
    #             break
    #     self.new_face_detected = new
    #     if new:
    #         self.detected_faces.append(detected_face)

    def breadcrumbs_callback(self, msg):
        #self.latest_people_marker_pose = msg.pose.position
        #self.latest_people_marker_robot_pose = self.current_pose
        #self.new_face_detected = True
        #self.detected_faces.append(msg.pose.position)
        self.info("~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        #self.approached_faces_num = 0
        #self.start_detecting = True

    def _peopleMarkerCallback(self, msg):
        """Handle new messages from 'people_marker'."""
        if self.start_detecting:
            curr_pose = self.current_pose
            self.info('Received people marker pose')
            # Store the latest pose for use in the movement loop
            #self.latest_people_marker_pose = msg.pose.position
            detected_face = msg
            detected_face_world = detected_face
            x = detected_face_world.pose.position.x + curr_pose.pose.position.x
            y = detected_face_world.pose.position.y + curr_pose.pose.position.y
            
            #self.new_face_detected = True
            #self.detected_faces.append(detected_face)
            new = True
            for face in self.detected_faces:
                if abs(x - face.pose.position.x) < 1.0 and abs(y - face.pose.position.y) < 1.0:
                    self.get_logger().info(f"Face already marked")
                    self.info(f"Face coordinates: {detected_face.pose.position.x}, {detected_face.pose.position.y}")
                    self.info(f"Robot coordinates: {curr_pose.pose.position.x}, {curr_pose.pose.position.y}")
                    self.info(f"--> sum coordinates: {x}, {y}")

                    new = False
                    break
            if new:
                self.start_detecting = False
                self.latest_people_marker_robot_pose = curr_pose
                self.info(f"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< New face detected")
                self.info(f"Face coordinates: {detected_face.pose.position.x}, {detected_face.pose.position.y}")
                self.info(f"Robot coordinates: {curr_pose.pose.position.x}, {curr_pose.pose.position.y}")
                self.info(f"--> sum coordinates: {x}, {y}")
                self.new_face_detected = True
                point_msg = PointStamped()
                point_msg.header.frame_id = 'base_link'
                point_msg.header.stamp = self.get_clock().now().to_msg()
                point_msg.point = detected_face.pose.position # Extract the position from the PoseStamped message

                # Publish the PointStamped message
                self.info("")
                self.info("PPPPPPPPUUUUUUUUUUUUUUUUUUUUUBBBBBBBBBBBBBBBBBBBLLLLLLLLLLLLLLLLLLLLLLIIIIIIIIIIIIIIIIIIIISSSSSSSSSSSSSSHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
                self.info("")

                self.face_pub.publish(point_msg)
                detected_face_world.pose.position.x = x
                detected_face_world.pose.position.y = y
                self.detected_faces.append(detected_face_world)
                #self.info(f"Face coordinates: {detected_face.x}, {detected_face.y}")
                
                
                #self.new_face_detected = True
                #self.detected_faces.append(detected_face)
                #self.latest_people_marker_pose = detected_face
                #self.latest_people_marker_robot_pose = self.current_pose
            
            self.latest_people_marker_pose = None

        
    def destroyNode(self):
        self.nav_to_pose_client.destroy()
        super().destroy_node()     

    def goToPose(self, pose, behavior_tree=''):
        """Send a `NavToPose` action request."""
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = behavior_tree

        self.info('Navigating to goal: ' + str(pose.pose.position.x) + ' ' +
                  str(pose.pose.position.y) + '...')
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg,
                                                                   self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Goal to ' + str(pose.pose.position.x) + ' ' +
                       str(pose.pose.position.y) + ' was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def spin(self, spin_dist, time_allowance=10):
        self.debug("Waiting for 'Spin' action server")
        while not self.spin_client.wait_for_server(timeout_sec=1.0):
            self.info("'Spin' action server not available, waiting...")
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = spin_dist
        goal_msg.time_allowance = Duration(sec=time_allowance)

        self.info(f'Spinning to angle {goal_msg.target_yaw}....')
        send_goal_future = self.spin_client.send_goal_async(goal_msg, self._feedbackCallback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.error('Spin request was rejected!')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True
    
    def undock(self):
        """Perform Undock action."""
        self.info('Undocking...')
        self.undock_send_goal()

        while not self.isUndockComplete():
            time.sleep(0.1)

    def undock_send_goal(self):
        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()

    def isUndockComplete(self):
        """
        Get status of Undock action.

        :return: ``True`` if undocked, ``False`` otherwise.
        """
        if self.undock_result_future is None or not self.undock_result_future:
            return True

        rclpy.spin_until_future_complete(self, self.undock_result_future, timeout_sec=0.1)

        if self.undock_result_future.result():
            self.undock_status = self.undock_result_future.result().status
            if self.undock_status != GoalStatus.STATUS_SUCCEEDED:
                self.info(f'Goal with failed with status code: {self.status}')
                return True
        else:
            return False

        self.info('Undock succeeded')
        return True

    def cancelTask(self):
        """Cancel pending task request of any type."""
        self.info('Canceling current task.')
        if self.result_future:
            future = self.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, future)
        return

    def isTaskComplete(self):
        """Check if the task request of any type is complete yet."""
        if not self.result_future:
            # task was cancelled or completed
            return True
        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.10)
        if self.result_future.result():
            self.status = self.result_future.result().status
            if self.status != GoalStatus.STATUS_SUCCEEDED:
                self.debug(f'Task with failed with status code: {self.status}')
                return True
        else:
            # Timed out, still processing, not complete yet
            return False

        self.debug('Task succeeded!')
        return True

    def getFeedback(self):
        """Get the pending action feedback message."""
        return self.feedback

    def getResult(self):
        """Get the pending action result message."""
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        elif self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        elif self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        else:
            return TaskResult.UNKNOWN

    def waitUntilNav2Active(self, navigator='bt_navigator', localizer='amcl'):
        """Block until the full navigation system is up and running."""
        self._waitForNodeToActivate(localizer)
        if not self.initial_pose_received:
            time.sleep(1)
        self._waitForNodeToActivate(navigator)
        self.info('Nav2 is ready for use!')
        return

    def _waitForNodeToActivate(self, node_name):
        # Waits for the node within the tester namespace to become active
        self.debug(f'Waiting for {node_name} to become active..')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info(f'{node_service} service not available, waiting...')

        req = GetState.Request()
        state = 'unknown'
        while state != 'active':
            self.debug(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.debug(f'Result of get_state: {state}')
            time.sleep(2)
        return
    
    def YawToQuaternion(self, angle_z):
        quat_tf = quaternion_from_euler(0, 0, angle_z)

        # Convert a list to geometry_msgs.msg.Quaternion
        quat_msg = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        return quat_msg

    def _amclPoseCallback(self, msg):
        self.debug('Received amcl pose')
        self.initial_pose_received = True
        self.current_pose = msg.pose
        return

    def _feedbackCallback(self, msg):
        self.debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
    def _dockCallback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def setInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = self.pose_frame_id
        msg.header.stamp = 0
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return
    
    def debug(self, msg):
        self.get_logger().debug(msg)
        return
    
    def check_approach(self, point):
        self.get_logger().info(f"IM LOOKING FOR FACES")
        #Check if there is a new 'people_marker' pose to go to first
        self.start_detecting = False
        if self.new_face_detected:
            
            curr_pose = self.latest_people_marker_robot_pose
        
            coord_face = PoseStamped()
            coord_face.header.frame_id = 'base_link'
            coord_face.header.stamp = self.get_clock().now().to_msg()
            coord_face = self.detected_faces[self.approached_faces_num]

            # x = curr_pose.pose.position.x + coord_face_relative_to_r.x
            # y = curr_pose.pose.position.y + coord_face_relative_to_r.y
            # z = coord_face_relative_to_r.z + curr_pose.pose.orientation.z
            x = coord_face.pose.position.x
            y = coord_face.pose.position.y
            z = math.atan2(y, x)
            self.get_logger().info(f"----------------------------> {z}")


            coord_face.pose.orientation = self.YawToQuaternion(z)  # ****
            
            
            #coord_face = self.current_pose + self.latest_people_marker_pose
            # for face in self.detected_faces:
            #     if abs(x - face.x) < 1.0 and abs(y - face.y) < 2.5:
            #         self.get_logger().info(f"Face already marked")
            #         #new = False
            #         return False


            # Assuming 'coord_face' is a PoseStamped message
            # Create a PointStamped message
            # point_msg = PointStamped()
            # point_msg.header = coord_face.header # Copy the header from the PoseStamped message
            # point_msg.point = coord_face.pose.position # Extract the position from the PoseStamped message

            # # Publish the PointStamped message
            # self.info("")
            # self.info("PPPPPPPPUUUUUUUUUUUUUUUUUUUUUBBBBBBBBBBBBBBBBBBBLLLLLLLLLLLLLLLLLLLLLLIIIIIIIIIIIIIIIIIIIISSSSSSSSSSSSSSHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
            # self.info("")

            # self.face_pub.publish(point_msg)
            # time.sleep(2)
            # if not self.new_face_detected:
            #     self.start_detecting = True
            #     return False

            #if new:
            # marked_poses.append(coord_face)
            self.get_logger().info(f"Number of detected faces so far: {len(self.detected_faces)}")
            for l in range(len(self.detected_faces)):
                self.get_logger().info(f"face {l}: x: {self.detected_faces[l].pose.position.x}, y: {self.detected_faces[l].pose.position.y}, z: {self.detected_faces[l].pose.orientation.z}")

            # MOVE TOWARDS THE FACE
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()

            goal_pose.pose.position.x = (curr_pose.pose.position.x - x)/2
            goal_pose.pose.position.y = (curr_pose.pose.position.y - y)/2

            # **Change 2: Set goal_pose orientation using yaw_angle**
            #yaw_angle = math.atan2(y1, x1)
            goal_pose.pose.orientation = self.YawToQuaternion(z)

            self.goToPose(goal_pose)
            while not self.isTaskComplete():
                self.info("Moving towards the face...")
                time.sleep(1)

            # ROTATE TO FACE THE FACE
            # Calculate the yaw angle to face the detected face
            #yaw_angle = math.atan2(y, x)
            # Use the spin function to rotate the robot to face the detected face
            self.spin(-z)
            while not self.isTaskComplete():
                self.info("Rotating towards the face...")
                time.sleep(1)
            # say hi
            #engine = pyttsx3.init()
            #engine.say("Hi there cutie")
            #engine.runAndWait()    
            self.get_logger().info(f"Hello there!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            time.sleep(5)

            # MOVE BACK TO THE POINT
            goal_pose.pose.position.x = point[0]
            goal_pose.pose.position.y = point[1]

            ## fix maybe
            goal_pose.pose.orientation = self.YawToQuaternion(point[2])

            self.goToPose(goal_pose)
            while not self.isTaskComplete():
                self.info("Moving back to the point...")
                time.sleep(1)
            time.sleep(1)
            
            #goal_pose.pose.orientation = self.YawToQuaternion(0.5)         
            # Reset the latest people marker pose to ensure it's only used once
            self.latest_people_marker_pose = None
            self.new_face_detected = False
            self.approached_faces_num += 1
            return True
        else:
            return False

def main(args=None):
    
    rclpy.init(args=args)
    rc = RobotCommander()

    # Wait until Nav2 and Localizer are available
    rc.waitUntilNav2Active()

    # Check if the robot is docked, only continue when a message is recieved
    while rc.is_docked is None:
        rclpy.spin_once(rc, timeout_sec=0.5)

    # If it is docked, undock it first
    if rc.is_docked:
        rc.undock()
    
    
    # Finally send it a goal to reach
    points = [[-1.16,-0.4,0.0],[-1.76,1.51,0.0],[-1.58,4.32,-0.584],[-1.58,3.44,-0.165],[0.25,3.32,-0.568],[1.9,3.04,0.57],[2.22,1.87,-0.989],[0.39,1.87,-0.207],[0.63,-0.76,0.458],[1.5,-0.4,-0.069],[3.27,-1.4,0.961],[2.23,-1.78,-1],[1.14,-1.8,-1.0],[-0.16,-1.33,0.832]]

    # for point in points:
    #     goal_pose = PoseStamped()
    #     goal_pose.header.frame_id = 'map'
    #     goal_pose.header.stamp = rc.get_clock().now().to_msg()

    #     goal_pose.pose.position.x = point[0]
    #     goal_pose.pose.position.y = point[1]
    #     goal_pose.pose.orientation = rc.YawToQuaternion(point[2])

    #     rc.goToPose(goal_pose)

    #     while not rc.isTaskComplete():
    #         rc.info("Waiting for the task to complete...")
    #         time.sleep(1)

    #     #goal_pose.pose.orientation = rc.YawToQuaternion(point[2]+3)
    #     #rc.goToPose(goal_pose)

    #     #while not rc.isTaskComplete():
    #     #    rc.info("Waiting for the task to complete...")
    #     #    time.sleep(1)

    #     #goal_pose.pose.orientation = rc.YawToQuaternion(point[2]+5.9)
    #     #rc.goToPose(goal_pose)

    #     #while not rc.isTaskComplete():
    #     #    rc.info("Waiting for the task to complete...")
    #     #    time.sleep(1)

    #     # Calculate the spin distance
    #     spin_dist = 0.5 * math.pi

    #    # Call the spin method four times to spin a full circle
    #     for _ in range(4):
    #         rc.spin(spin_dist)
    #         while not rc.isTaskComplete():
    #             rc.info("Waiting for the task to complete...")
    #             time.sleep(1)

    #         time.sleep(2)  #
           
    # rc.destroyNode()

    marked_poses = []
    i = 0
    while len(points) > i:
        #Check if there is a new 'people_marker' pose to go to first
        # if rc.latest_people_marker_pose:
        #     rc.get_logger().info("AAAAAAAAAAAAAH")
        #     #
        #     # coord_face = PoseStamped()
        #     # coord_face.header.frame_id = 'map'
        #     # coord_face.header.stamp = rc.get_clock().now().to_msg()

        #     # x = rc.current_pose.pose.position.x + rc.latest_people_marker_pose.x
        #     # y = rc.current_pose.pose.position.y + rc.latest_people_marker_pose.y
        #     # z = rc.latest_people_marker_pose.z
        #     # coord_face.pose.position.x = x
        #     # coord_face.pose.position.y = y
        #     # coord_face.pose.orientation = rc.YawToQuaternion(z)
        #     # #coord_face = rc.current_pose + rc.latest_people_marker_pose
        #     # new = True
        #     # for face in marked_poses:
        #     #     if abs(x - face.pose.position.x) < 0.3 and abs(y - face.pose.position.y) < 0.3:
        #     #         rc.get_logger().info(f"Face already marked")
        #     #         new = False
        #     #         break
        #     # if new:
        #     #     marked_poses.append(coord_face)
        #     #     curr_pose = rc.current_pose
        #     #     goal_pose = PoseStamped()
        #     #     goal_pose.header.frame_id = 'map'
        #     #     goal_pose.header.stamp = rc.get_clock().now().to_msg()


        #     #     #make it better maybe
        #     #     goal_pose.pose.orientation = rc.YawToQuaternion(-z)

        #     #     goal_pose.pose.position.x = rc.current_pose.pose.position.x + rc.latest_people_marker_pose.x/2
        #     #     goal_pose.pose.position.y = rc.current_pose.pose.position.y + rc.latest_people_marker_pose.y/2

        #     #     rc.goToPose(goal_pose)
        #     #     while not rc.isTaskComplete():
        #     #         rc.info("Waiting for the task to complete...")
        #     #         time.sleep(1)

        #     #     # say hi
        #     #     rc.get_logger().info(f"Hello there!")
        #     #     time.sleep(1)

        #     #     goal_pose.pose.position.x = curr_pose.pose.position.x
        #     #     goal_pose.pose.position.y = curr_pose.pose.position.y
        #     #     ## fix maybe
        #     #     goal_pose.pose.orientation = rc.YawToQuaternion(z)

        #     #     rc.goToPose(goal_pose)
        #     #     while not rc.isTaskComplete():
        #     #         rc.info("Waiting for the task to complete...")
        #     #         time.sleep(1)
        #     #     time.sleep(1)
                
        #     #     #goal_pose.pose.orientation = rc.YawToQuaternion(0.5)         
        #     #     # Reset the latest people marker pose to ensure it's only used once
        #     rc.latest_people_marker_pose = None
        #     #curr_pose = rc.current_pose
            
        
        point = points[i]
        # If no new 'people_marker' pose, proceed with the next point in the list
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = rc.get_clock().now().to_msg()
        goal_pose.pose.position.x = point[0]
        goal_pose.pose.position.y = point[1]
        goal_pose.pose.orientation = rc.YawToQuaternion(point[2])



        rc.goToPose(goal_pose)
        

        while not rc.isTaskComplete():
            rc.info("Waiting for the task to complete...")
            time.sleep(1)

        rc.latest_people_marker_pose = None
        spin_dist = 0.05 * math.pi
        n = 0
        while n < 40:
            rc.spin(spin_dist)
            #rc.start_detecting = False
            rc.start_detecting = True
            while not rc.isTaskComplete():
                
                rc.info("Spinning in circle...")
                #rc.get_logger().info(f"curr pose x: {rc.current_pose.pose.position.x} y: {rc.current_pose.pose.position.y} z: {rc.current_pose.pose.orientation.z}")
                
                # rc.check_approach(marked_poses, rc.current_pose)
                time.sleep(1)
                

            
            #time.sleep(3)
            approached_face = rc.check_approach(point)
            if approached_face: 
                #rc.cancelTask()
                n = -1
                break
            n += 1
            #rc.get_logger().info(f"curr pose x: {rc.current_pose.pose.position.x} y: {rc.current_pose.pose.position.y} z: {rc.current_pose.pose.orientation.z}")
            #if rc.check_approach(marked_poses, point): 
            #    n = 0

        
        i+=1

        # if rc.latest_people_marker_pose and rc.latest_people_marker_pose not in marked_poses:
        #     rc.get_logger().info(f"I got coordinates for the face!")
        #     goal_pose = PoseStamped()
        #     goal_pose.header.frame_id = 'map'
        #     goal_pose.header.stamp = rc.get_clock().now().to_msg()
            

        #     marked_poses.append(rc.latest_people_marker_pose)




        #     #rc.get_logger().info(f"current pose: x={rc.current_pose} Possition of face: {rc.latest_people_marker_pose.x} {rc.latest_people_marker_pose.y} {rc.latest_people_marker_pose.z}")
        #     # goal_pose.pose.position.x = 0.0
        #     # goal_pose.pose.position.y = 0.0
        #     goal_pose.pose.orientation = rc.YawToQuaternion(-rc.latest_people_marker_pose.z)

        #     goal_pose.pose.position.x = rc.current_pose.pose.position.x + rc.latest_people_marker_pose.x/2
        #     goal_pose.pose.position.y = rc.current_pose.pose.position.y + rc.latest_people_marker_pose.y/2

        #     rc.goToPose(goal_pose)
        #     #goal_pose.pose.orientation = rc.YawToQuaternion(0.5)         
        #     # Reset the latest people marker pose to ensure it's only used once
        #     rc.latest_people_marker_pose = None
        # else:
        #     point = points[i]
        #     # If no new 'people_marker' pose, proceed with the next point in the list
        #     goal_pose = PoseStamped()
        #     goal_pose.header.frame_id = 'map'
        #     goal_pose.header.stamp = rc.get_clock().now().to_msg()
        #     goal_pose.pose.position.x = point[0]
        #     goal_pose.pose.position.y = point[1]
        #     goal_pose.pose.orientation = rc.YawToQuaternion(point[2])


        #     while not rc.isTaskComplete():
        #         rc.info("Waiting for the task to complete...")
        #         time.sleep(1)

        #     rc.goToPose(goal_pose)
        #     spin_dist = 0.5 * math.pi

        #     for _ in range(4):
        #         rc.spin(spin_dist)
        #         while not rc.isTaskComplete():
        #             rc.info("Waiting for the task to complete...")
        #             time.sleep(1)

            
        #     i+=1
  

    
    # rc.destroyNode()
    # And a simple example
if __name__=="__main__":
    main()