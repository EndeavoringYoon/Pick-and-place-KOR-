# Codes

# path_planning.py

### cam.py와 같이 사용하는 버전

```python
#!/usr/bin/env python

from __future__ import print_function
from dis import dis
import re
from shutil import move

import tf
import sys
sys.path.append("../js_ws/src")
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import numpy as np
import math
import ast
import cv2
from cam import camera_node
from scripts.predictor import SEG_CLASS_NAMES
import open3d as o3d

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import Float64
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL
#getting group name of robot
robot_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class path_planning(object):
    """path_planning"""

    def __init__(self):
        super(path_planning, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        gripper_name = "gripper"
        gripper_group = moveit_commander.MoveGroupCommander(gripper_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        # pub_cart = rospy.Publisher("publishing_trajectory_for_dynamixel", moveit_msgs.msg.RobotTrajectory, queue_size=10)
        display_trajectory_publisher = rospy.Publisher(
            "display_planned_path_for_dynamixel",
            Float64,
            queue_size=20,
        )
        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Setting box position and size manually(We'll not use it because we're going to use CV)
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = robot_group.get_planning_frame() #world
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = 0.25
        box_pose.pose.position.z = 0.15
        quat_box=tf.transformations.quaternion_from_euler(0,0,tau/8)
        box_pose.pose.orientation=geometry_msgs.msg.Quaternion(*quat_box)
        box_size=(0.05,0.05,0.3)

        # Deciding end position:
        end_position = geometry_msgs.msg.Pose()
        quat=tf.transformations.quaternion_from_euler(0,0,-tau/8)
        end_position.orientation=geometry_msgs.msg.Quaternion(*quat)
        end_position.position.x = 0.25
        end_position.position.y = -0.25
        end_position.position.z = box_pose.pose.position.z+0.05

        #Defining ground
        ground1_name="ground1"
        ground1_pose=geometry_msgs.msg.PoseStamped()
        ground1_pose.header.frame_id = robot_group.get_planning_frame()
        ground1_pose.pose.position.x = 0.065
        ground1_pose.pose.position.y = 0.0
        ground1_pose.pose.position.z = 0.0175
        quat_ground1=tf.transformations.quaternion_from_euler(0,0,0)
        ground1_pose.pose.orientation=geometry_msgs.msg.Quaternion(*quat_ground1)
        ground1_size=(0.07, 0.2, 0.035)
        scene.add_box(ground1_name, ground1_pose,size=ground1_size)

        ground2_name="ground2"
        ground2_pose=geometry_msgs.msg.PoseStamped()
        ground2_pose.header.frame_id = robot_group.get_planning_frame()
        ground2_pose.pose.position.x = 0.2
        ground2_pose.pose.position.y = 0.0
        ground2_pose.pose.position.z = -0.0025
        quat_ground2=tf.transformations.quaternion_from_euler(0,0,0)
        ground2_pose.pose.orientation=geometry_msgs.msg.Quaternion(*quat_ground2)
        ground2_size=(0.4, 0.4, 0.005)
        scene.add_box(ground2_name, ground2_pose,size=ground2_size)

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.box_pose=box_pose
        self.box_size=box_size
        self.end_position=end_position
        # self.pub_cart=pub_cart

    def get_box_pose(self,get_from_cv):
        received_box=geometry_msgs.msg.PoseStamped()
        received_box.header.frame_id = robot_group.get_planning_frame()
        received_box.pose.position.x=get_from_cv['recommended_cp'][0]/1000
        received_box.pose.position.y=get_from_cv['recommended_cp'][1]/1000
        received_box.pose.position.z=get_from_cv['recommended_cp'][2]/1000
        quat_box=tf.transformations.quaternion_from_euler(0,0,math.atan(get_from_cv['recommended_cp'][1]/get_from_cv['recommended_cp'][0]))
        received_box.pose.orientation=geometry_msgs.msg.Quaternion(*quat_box)
        self.box_pose=received_box
        self.end_position.position.z=received_box.pose.position.z
        return received_box
    
    def get_box_size(self, get_from_cv):
        received_box_size=(0.05,get_from_cv['width']/1000,get_from_cv['recommended_cp'][2]*2/1000)
        self.box_size=received_box_size
        return received_box_size

    def get_end_position(self):
        end_position_now = self.end_position
        return end_position_now

    def gripper(self, state="open"):
        gripper_group=self.gripper_group
        gripper_goal=gripper_group.get_current_joint_values()
        box_size=self.box_size
        # print(box_size)
        if state=="open":
            gripper_goal[0] = 0.015
            gripper_goal[1] = 0.015
            val_for_dis=0.077
        elif state =="close":
            gripper_goal[0] = -0.010
            gripper_goal[1] = -0.010
            val_for_dis=0.023

        elif state == "grip_close":
            gripper_goal[0] = box_size[1]/2-0.010+0.004
            gripper_goal[1] = box_size[1]/2-0.010+0.004
            val_for_dis = box_size[1]

        elif state == "test":
            gripper_goal[0] = 0.015
            gripper_goal[1] = 0.015
            val_for_dis=0.023
        else:
            print("inproper state: the default state is open")
            gripper_goal[0] = 0.015
            gripper_goal[1] = 0.015
            val_for_dis=0.077
        self.display_trajectory(val_for_dis)

        # gripper_group.go(gripper_goal, wait=True)
    
    def go_to_joint_state(self):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = tau/8
        joint_goal[1] = -1.05
        joint_goal[2] = 0.35
        joint_goal[3] = 0.7

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self,pose_goal):

        move_group = self.move_group
        print("name of end effector:",moveit_commander.move_group.MoveGroupCommander.get_end_effector_link) ## end effector name

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        move_group.get_current_pose()
        # pose_goal = geometry_msgs.msg.Pose()
        # p=tf.transformations.quaternion_from_euler(0,0,0)
        # pose_goal.orientation=geometry_msgs.msg.Quaternion(*p)
        # pose_goal.orientation.z=tau/8
        # pose_goal.orientation.w=1
        # pose_goal.position.x = 0.0
        # pose_goal.position.y = 0.0
        # pose_goal.position.z = 0.3
        print(pose_goal)
        move_group.set_pose_target(pose=pose_goal)
      
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        move_group.go(pose_goal,wait=True) #예제에서는 pose_goal이 안넣어져 있는데 넣었더니 작동함
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

    # def plan_cartesian_path_2(self, pose, scale=0.1):
    #     self.display_trajectory(101)
    #     # Copy class variables to local variables to make the web tutorials more clear.
    #     # In practice, you should use the class variables directly unless you have a good
    #     # reason not to.
    #     move_group = self.move_group
    #     scale_here=scale
        
    #     waypoints = []
    #     wpose=move_group.get_current_pose().pose
    #     waypoints.append(copy.deepcopy(wpose))
    #     wpose = pose  #Should be kind of move_group.get_current_pose().pose
    #     waypoints.append(copy.deepcopy(wpose))
    #     print("wpose:", wpose,"\n")

    #     print("waypoints:",waypoints)

    #     fraction=0
    #     accuracy=0.90
    #     sleep_time=0.1

    #     while fraction<accuracy:
    #         (plan, fraction) = move_group.compute_cartesian_path(
    #             waypoints, scale_here, 0.0  # waypoints to follow  # eef_step # jump_threshold
    #         )
    #         time.sleep(sleep_time)
    #         scale_here-=0.001
    #         if scale_here<0.001:
    #             scale_here=0.1
    #             accuracy-=0.05
    #         move_group.clear_pose_targets()

        
    #     time.sleep(1)
    #     print("preparing complete plan")
    #     if scale_here !=0.1:
    #         scale_here+=0.001
    #     (plan, fraction) = move_group.compute_cartesian_path(waypoints, scale_here, 0.0)

    #     print(plan)
    #     # Note: We are just planning, not asking move_group to actually move the robot yet:

    #     return plan, fraction

    def plan_cartesian_path(self, pose, scale=0.1):
        self.display_trajectory(101)
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        scale_here=scale
        
        waypoints = []
        wpose=move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        wpose = pose  #Should be kind of move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        print("waypoints:",waypoints)

        fraction_max=0
        accuracy=0.90
        sleep_time=0.05
        max_scale=0
        while scale_here>0.003:
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, scale_here, 0.0  # waypoints to follow  # eef_step # jump_threshold
            )
            
            
            if fraction>fraction_max:
               max_scale=copy.deepcopy(scale_here)
               fraction_max=copy.deepcopy(fraction)
            if fraction>0.9:
                break
            time.sleep(sleep_time)
            scale_here-=0.004
            

        move_group.clear_pose_targets()
        time.sleep(0.5)
        print("preparing complete plan")
        print(max_scale,fraction_max)
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, max_scale, 0.0)

        # print(plan)
        # Note: We are just planning, not asking move_group to actually move the robot yet:

        return plan, fraction
    
    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        if plan ==100:
            print("Sending trajectory for arm")
        elif plan < 10:
            print("Sending values for gripper to move:",plan)
        else:
            print("initializing:")
        display_trajectory_publisher.publish(plan)
        
        

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        
        self.display_trajectory(100)
        move_group.execute(plan, wait=True)
        
        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def round_to_2(x):
        return int(math.floor(x*100))/100

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        box_pose=self.box_pose
        box_size=self.box_size

        print(box_pose)
        print(box_size)
        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        
        box_name = "box"
        scene.add_box(box_name, box_pose,size=box_size)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    def calculate_distance(self, dist):
        box_pose = self.box_pose
        goal_dist=copy.deepcopy(box_pose)
        goal_dist.pose.position.x-=dist*math.cos(box_pose.pose.orientation.z)
        goal_dist.pose.position.y-=dist*math.sin(box_pose.pose.orientation.z)
        
        return goal_dist.pose

def main():
    
    rospy.init_node('path_planning', anonymous=False)
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    
    try:
        scene = moveit_commander.PlanningSceneInterface()
        eef_link = moveit_commander.MoveGroupCommander('arm').get_end_effector_link()
        scene.remove_attached_object(eef_link, name='box')
        scene.remove_world_object(eef_link, name='box')
        tutorial.remove_box()
    except:
            pass
    tutorial = path_planning()
    print(tutorial.move_group.get_current_pose().pose)
    input(
        "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
    
    robot_group.clear_pose_targets()
    time.sleep(1)
    robot_group.go([-1.3,-1.3,0.0,0.0], wait=True)
    tutorial.display_trajectory(100)
    time.sleep(1)
    input("Ready to take picture")
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    
    cam = camera_node() 

    try:
        idx = input("put class number you want.")
        idx = int(idx)

        assert idx > 0 and idx <= 3, "No segmentation provided for background."

        print(SEG_CLASS_NAMES[idx])

    except(ValueError):
        print("try again, I only get integer.")

    pcd_dict = cam.segmentation_to_pointcloud(searching_class = idx, class_n = 4)
    # rgb_img, depth_img, image_mask = cam.visualize()
    print(pcd_dict)

    # box=tutorial.get_box_pose()
    # print("box",box)
    box_pose = tutorial.get_box_pose(pcd_dict).pose
    box_size = tutorial.get_box_size(pcd_dict)
    input("for test:print enter")
    tutorial.gripper("test")
    input("============initalizing position============================")
    robot_group.clear_pose_targets()
    initial_pose=[0,0,0,0]
    robot_group.go(initial_pose, wait=True)
    tutorial.display_trajectory(100)
    robot_group.stop()

    print("==================initializing position: done")
    fk=tutorial.move_group.get_current_pose().pose
    input("============ Press `Enter` to add a box to the planning scene ...")
    tutorial.add_box()
        
    # Ready to grip

    input("============ Press `Enter` to plan and display a Cartesian path ...")
    prepare_to_pick= tutorial.calculate_distance(0.02)
    (cartesian_plan, fraction_pre) = tutorial.plan_cartesian_path(prepare_to_pick)

    # input("============ Press `Enter` to execute a saved path ...")
    tutorial.execute_plan(cartesian_plan)

    input("============ Open gripper ===============")
    tutorial.gripper("open")

    input("================== Trying to Grip ===============")
    robot_group.clear_pose_targets()
    where_to_grip=tutorial.calculate_distance(-0.03)
    (cartesian_plan_to_grip, fraction_grip) = tutorial.plan_cartesian_path(where_to_grip)
    tutorial.execute_plan(cartesian_plan_to_grip)

        
    input("================ Close Gripper ============")
    tutorial.gripper("grip_close")

    # Grip(assume grip: just attach used)
    input("============ Press `Enter` to attach a box to the Panda robot ...")
    tutorial.attach_box()
        
    #Move to Target Place
    input(
            "============ Press `Enter` to plan a path with an attached collision object ..."
        )
    robot_group.clear_pose_targets()
    going_up=tutorial.calculate_distance(0.04)
    going_up.position.z+=0.05
    (cartesian_plan_to_prevent_collision,fraction__collision) = tutorial.plan_cartesian_path(going_up)
    tutorial.execute_plan(cartesian_plan_to_prevent_collision)

    
    input("==========Press 'Enter' to move to final position")
    end_pose=tutorial.get_end_position()
    (cartesian_plan_to_end, fraction_end) = tutorial.plan_cartesian_path(end_pose)
    tutorial.execute_plan(cartesian_plan_to_end)

    # input("============= second planning ============")
    # (second_plan_to_end, fraction_second_end) = tutorial.plan_cartesian_path(end_pose)
    # tutorial.execute_plan(second_plan_to_end)

    input("============ Press `Enter` to finish pick & place ...")
    tutorial.gripper("open")
    tutorial.detach_box()
    time.sleep(0.5)
    tutorial.remove_box()
    time.sleep(0.5)
    print("============ Python tutorial demo complete!")

if __name__ == "__main__":
    main()
```

### realsense 없이 임의 위치로 설정하여 이동하는 버전

```python
#!/usr/bin/env python

from __future__ import print_function
from dis import dis
from shutil import move

import tf
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import time
import numpy as np
import argparse
import math
import ast

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import Float64
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL
#getting group name of robot
robot_group = moveit_commander.MoveGroupCommander("arm")
gripper_group = moveit_commander.MoveGroupCommander("gripper")

def all_close(goal, actual, tolerance):
    
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True

class path_planning(object):
    """path_planning"""

    def __init__(self):
        super(path_planning, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node("i_hate_gazebo", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        gripper_name = "gripper"
        gripper_group = moveit_commander.MoveGroupCommander(gripper_name)
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        # pub_cart = rospy.Publisher("publishing_trajectory_for_dynamixel", moveit_msgs.msg.RobotTrajectory, queue_size=10)
        display_trajectory_publisher = rospy.Publisher(
            "display_planned_path_for_dynamixel",
            Float64,
            queue_size=20,
        )
        display_trajectory_once_more_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Setting box position and size
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = robot_group.get_planning_frame() #world
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = 0.25
        box_pose.pose.position.z = 0.15
        quat_box=tf.transformations.quaternion_from_euler(0,0,tau/8)
        box_pose.pose.orientation=geometry_msgs.msg.Quaternion(*quat_box)
        box_size=(0.05,0.05,0.3)

        # Deciding end position:
        end_position = geometry_msgs.msg.Pose()
        quat=tf.transformations.quaternion_from_euler(0,0,0)
        end_position.orientation=geometry_msgs.msg.Quaternion(*quat)
        end_position.position.x = 0.3
        end_position.position.y = 0.0
        end_position.position.z = box_pose.pose.position.z+0.05

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.gripper_group = gripper_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.display_trajectory_once_more_publisher = display_trajectory_once_more_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names
        self.box_pose=box_pose
        self.box_size=box_size
        self.end_position=end_position
        # self.pub_cart=pub_cart

    def get_box_pose(self):
        box_pose_now = self.box_pose
        return box_pose_now
    
    def get_box_size(self):
        box_size_now = self.box_size
        return box_size_now

    def get_end_position(self):
        end_position_now = self.end_position
        return end_position_now

    def gripper(self, state="open"):
        gripper_group=self.gripper_group
        gripper_goal=gripper_group.get_current_joint_values()
        box_size=self.box_size
        # print(box_size)
        if state=="open":
            gripper_goal[0] = 0.015
            gripper_goal[1] = 0.015
            val_for_dis=0.077
        elif state =="close":
            gripper_goal[0] = -0.010
            gripper_goal[1] = -0.010
            val_for_dis=0.023
        elif state == "grip_open":
            gripper_goal[0] = box_size[1]/2-0.010+0.002
            gripper_goal[1] = box_size[1]/2-0.010+0.002
            val_for_dis=0.077
            print("gripper_goal:",gripper_goal)
        elif state == "grip_close":
            gripper_goal[0] = box_size[1]/2-0.010
            gripper_goal[1] = box_size[1]/2-0.010
            val_for_dis = box_size[1]

        elif state == "test":
            gripper_goal[0] = 0.015
            gripper_goal[1] = 0.015
            val_for_dis=0.023
        else:
            print("inproper state: the default state is open")
            gripper_goal[0] = 0.015
            gripper_goal[1] = 0.015
            val_for_dis=0.077
        self.display_trajectory(val_for_dis)

        # gripper_group.go(gripper_goal, wait=True)
    

    def go_to_joint_state(self):

        move_group = self.move_group
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = tau/8
        joint_goal[1] = -1.05
        joint_goal[2] = 0.35
        joint_goal[3] = 0.7

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, position):

        move_group = self.move_group
        print("name of end effector:",moveit_commander.move_group.MoveGroupCommander.get_end_effector_link) ## end effector 이름 가져오기

        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:
        end_position = self.end_position
      
        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        #move_group.go(pose_goal,wait=True) #예제에서는 pose_goal이 안넣어져 있는데 넣었더니 작동함
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        #return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, pose, scale=0.1):
        self.display_trajectory(101)
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group
        scale_here=scale
        
        waypoints = []
        wpose=move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        wpose = pose  #Should be kind of move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))
        print("wpose:", wpose,"\n")

        # wpose.position.z = 0.3  # First move up (z)
        # #wpose.position.y -= scale * 0.3  # and sideways (y) #이거 작동 안함(orientation 문제인듯)
        # wpose.position.x += scale * 0.3  # Second move forward/backwards in (x)
        # print("wpose:", wpose, "\n")
        # waypoints.append(copy.deepcopy(wpose))

        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y += scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))
        print("waypoints:",waypoints)

        fraction=0
        accuracy=0.90
        sleep_time=0.1

        while fraction<accuracy:
            (plan, fraction) = move_group.compute_cartesian_path(
                waypoints, scale_here, 0.0  # waypoints to follow  # eef_step
            )  # jump_threshold
            time.sleep(sleep_time)
            scale_here-=0.001
            if scale_here<0.001:
                scale_here=0.1
                accuracy-=0.05
            move_group.clear_pose_targets()

        
        time.sleep(1)
        print("preparing complete plan")
        if scale_here !=0.1:
            scale_here+=0.001
        (plan, fraction) = move_group.compute_cartesian_path(waypoints, scale_here, 0.0)

        print(plan)
        # Note: We are just planning, not asking move_group to actually move the robot yet:

        return plan, fraction

        ## END_SUB_TUTORIAL
    
    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        if plan ==100:
            print("i send arm to move: code number", plan)
        elif plan < 10:
            print("i send gripper to move:",plan)
        display_trajectory_publisher.publish(plan)
        
        

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        # pub_cart = self.pub_cart
        
        self.display_trajectory(100)
        # pub_cart.publish(plan)
        move_group.execute(plan, wait=True)
        
        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node was just created (https://github.com/ros/ros_comm/issues/176),
        ## or dies before actually publishing the scene update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed.
        ## To avoid waiting for scene updates like this at all, initialize the
        ## planning scene interface with  ``synchronous = True``.
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        box_pose=self.box_pose
        box_size=self.box_size
        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        
        box_name = "box"
        scene.add_box(box_name, box_pose,size=box_size)

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "gripper"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )

    def calculate_distance(self, dist):
        box_pose = self.get_box_pose()
        goal_dist=copy.deepcopy(box_pose)
        goal_dist.pose.position.x-=dist*math.cos(box_pose.pose.orientation.z)
        goal_dist.pose.position.y-=dist*math.sin(box_pose.pose.orientation.z)
        
        return goal_dist.pose

def main():
    try:
        rospy.init_node('ihategazebo', anonymous=False)
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        tutorial = ihategazebo()
        print(tutorial.move_group.get_current_pose().pose)
        input(
            "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        )
        
        # box=tutorial.get_box_pose()
        # print("box",box)
        box_pose = tutorial.get_box_pose().pose
        box_size = tutorial.get_box_size()
        input("for test:print enter")
        tutorial.gripper("test")
        input("============initalizing position============================")
        robot_group.clear_pose_targets()
        initial_pose=[0,0,0,0]
        robot_group.go(initial_pose, wait=True)
        tutorial.display_trajectory(100)
        robot_group.stop()
        # input("==============Press 'Enter' to test gripper=====================")
        # tutorial.gripper("open")
        # time.sleep(3)
        # tutorial.gripper("close")
        
        print("==================initializing position: done")

        input("============ Press `Enter` to add a box to the planning scene ...")
        tutorial.add_box()
        
        # Ready to grip

        input("============ Press `Enter` to plan and display a Cartesian path ...")
        prepare_to_pick= tutorial.calculate_distance(0.2)
        (cartesian_plan, fraction_pre) = tutorial.plan_cartesian_path(prepare_to_pick)

        input("============ Press `Enter` to execute a saved path ...")
        tutorial.execute_plan(cartesian_plan)

        input("============ Open gripper ===============")
        tutorial.gripper("open")

        input("================== Trying to Grip ===============")
        robot_group.clear_pose_targets()
        where_to_grip=tutorial.calculate_distance(0.02)
        (cartesian_plan_to_grip, fraction_grip) = tutorial.plan_cartesian_path(where_to_grip)
        tutorial.execute_plan(cartesian_plan_to_grip)

        
        input("================ Close Gripper ============")
        tutorial.gripper("grip_close")

        # Grip(assume grip: just use attach)
        input("============ Press `Enter` to attach a box to the Panda robot ...")
        tutorial.attach_box()
        
        #Move to Target Place
        input(
            "============ Press `Enter` to plan a path with an attached collision object ..."
        )

        robot_group.clear_pose_targets()
        end_pose=tutorial.get_end_position()
        # print(end_pose)
        (cartesian_plan_to_end, fraction_end) = tutorial.plan_cartesian_path(end_pose)
        tutorial.execute_plan(cartesian_plan_to_end)

        input("============= second planning ============")
        (second_plan_to_end, fraction_second_end) = tutorial.plan_cartesian_path(end_pose)
        tutorial.execute_plan(second_plan_to_end)

        '''
        put_it_down=tutorial.get_end_position()
        put_it_down.position.z-=0.05
        print("put_it_down:",put_it_down)
        # cartesian_to_detach, fraction_to_detach = tutorial.plan_cartesian_path(put_it_down)
        # tutorial.execute_plan(cartesian_to_detach)
        time.sleep(1)
        robot_group.go(put_it_down, wait=True)
        '''

        input("============ Press `Enter` to detach the box from the Panda robot ...")
        tutorial.detach_box()

        input(
            "============ Press `Enter` to remove the box from the planning scene ..."
        )
        tutorial.remove_box()
        print("============ Python tutorial demo complete!")

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == "__main__":
    main()
```

# sending_trajectory.py

```python
#!/usr/bin/env python

from cmath import pi
import rospy
import geometry_msgs
from trajectory_msgs.msg import *
from moveit_msgs.msg import *
import time
import copy
from std_msgs.msg import Float64
from dynamixel_workbench_msgs.srv import DynamixelCommand
import math

trueorfalse=Float64(101.0)
# traj=moveit_msgs.msg.DisplayTrajectory()
def gripper_goal(data): #data는 물체의 가로폭에 해당하는 값을 받아와야 함
    print(data)
    val=(data*1000+33.0)/2.0
    print(val-1395/val)
    theta=round(math.asin((val-(1395/val))/30)/pi*2048)
    print(theta)
    theta=2048-theta
    print(theta)
    return theta

def publish_to_dynamixel(data):
    pub=rospy.Publisher('/dynamixel_workbench/joint_trajectory',JointTrajectory,queue_size=10)
    r=rospy.Rate(10)
    pub.publish(data)

def callback(data):
    global trueorfalse
    # global traj
    traj=data
    # print(type(traj.trajectory[-1].joint_trajectory.points))
    # print(num_of_epoch)
    if traj.trajectory[-1].joint_trajectory.joint_names[0]=='joint1' and trueorfalse==Float64(100):
        print('it is arm')
        traj_real=traj.trajectory[-1].joint_trajectory
        print(traj_real)
        publish_to_dynamixel(traj_real)
        print("end send arm")
        trueorfalse=Float64(101.0)
    else:
        pass

def state_subscriber():
    lol2=rospy.Subscriber("/move_group/display_planned_path",DisplayTrajectory, callback)

def send_trajectory():
    global trueorfalse
    pub2=rospy.ServiceProxy("/dynamixel_workbench/dynamixel_command", DynamixelCommand)
    while not rospy.is_shutdown():
        print("receiving:")
        # while not rospy.is_shutdown():

        trueorfalse=rospy.wait_for_message("display_planned_path_for_dynamixel", Float64)
        print(trueorfalse)
        if float(trueorfalse.data)==100.0:
            print("getting arm trajectory")
            state_subscriber()
        elif float(trueorfalse.data)<10.0:
            print("moving gripper")
            resp = pub2("",15,"Goal_Position",gripper_goal(trueorfalse.data))
            trueorfalse=Float64(101.0)
        else:
            pass
    

if __name__ == '__main__' :
    rospy.init_node("joint_trajectory")
    trueorfalse=Float64(101.0)
    try:
        # while not rospy.is_shutdown():
        send_trajectory()
        # traj_test = rospy.wait_for_message("/move_group/display_planned_path",DisplayTrajectory)
        # print(traj_test)
    except rospy.ROSInterruptException:
        pass
```

# sending_jointstates.py

```python
#!/usr/bin/env python

import rospy
import geometry_msgs
from sending_trajectory import gripper_goal
from trajectory_msgs import *
from moveit_msgs.msg import *
from sensor_msgs.msg import JointState
import time
import copy
import math

def publish_to_joint_states(data):
    pub=rospy.Publisher('joint_states',JointState,queue_size=100)
    r=rospy.Rate(10)
    pub.publish(data)

def position_cal(data): #data는 물체의 가로폭에 해당하는 값을 받아와야 함
    position=(15*math.sin(data)+math.sqrt(40**2-(15*math.cos(data))**2))/1000-0.0371
    print(position)
    return position

def yoon(data):
    position=data/(-100)
    print(position)
    return position

def callback(data):
    ihateu=[]
    ihateu=copy.deepcopy(data)
    gripper_pose=yoon(data.position[0])
    ihateu.name.insert(1,'gripper_sub')
    ihateu.position=(gripper_pose,gripper_pose,data.position[1],data.position[2],data.position[3],data.position[4])
    ihateu.velocity=(data.velocity[0],data.velocity[0],data.velocity[1],data.velocity[2],data.velocity[3],data.velocity[4])
    ihateu.effort=(data.effort[0],data.effort[0],data.effort[1],data.effort[2],data.effort[3],data.effort[4])
    # print(ihateu)
    
    publish_to_joint_states(ihateu)

def state_subscriber():
    # rospy.init_node('state_subscriber',anonymous=True)
    rospy.Subscriber("/dynamixel_workbench/joint_states", JointState,callback)

    rospy.spin()

    
if __name__ == '__main__' :
    rospy.init_node("joint_states")
    try:
        # while not rospy.is_shutdown():
        state_subscriber()
        print('done')
        # traj_test = rospy.wait_for_message("/move_group/display_planned_path",DisplayTrajectory)
        # print(traj_test)
    except rospy.ROSInterruptException:
        pass
```