#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
attach_l_shape_no_user_interaction.py

This script attaches a properly oriented L-shaped antenna to the Panda arm's gripper flange.
The L-shape is composed of two connected boxes:
    - Box1: Horizontal part (shorter side)
    - Box2: Vertical part (longer side)
The antenna is oriented such that the vertical part points upward, parallel to the robot's head.
The script runs automatically without requiring user interaction.
"""

import sys
import rospy
import moveit_commander
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Quaternion
import numpy as np
from tf.transformations import quaternion_from_euler

def create_properly_oriented_l_shape(end_effector_link):
    """
    Creates an L-shaped collision object composed of two connected boxes with the desired orientation.

    :param end_effector_link: The name of the robot's end effector link.
    :return: CollisionObject representing the properly oriented L-shaped antenna.
    """
    # Initialize the CollisionObject
    l_shape = CollisionObject()
    l_shape.id = "proper_l_shape_antenna"
    l_shape.header.frame_id = end_effector_link

    # Define rotation: Vertical part points upward
    roll = 0
    pitch = 0
    yaw = 150    # Rotation as per user specification
    q = quaternion_from_euler(roll, pitch, yaw)

    # Define Box1: Horizontal part of the L (Shorter Side)
    box1 = SolidPrimitive()
    box1.type = SolidPrimitive.BOX
    box1.dimensions = [0.078, 0.05, 0.01]  # [length (x), width (y), height (z was 0.02)] for corner reflector [0.078, 0.05, 0.0705]

    box1_pose = Pose()
    box1_pose.orientation = Quaternion(*q)  # Apply rotation
    box1_pose.position.x = 0.0045  # for corner reflector 0.0045
    box1_pose.position.y = -0.0045 # for corner reflector -0.0045
    box1_pose.position.z = 0.005 # for corner reflector 0.035

    # Define Box2: Vertical part of the L (Longer Side)
    box2 = SolidPrimitive()
    box2.type = SolidPrimitive.BOX
    box2.dimensions = [0.08, 0.065, 0.1]  # [length (x), width (y), height (z)] in meters

    box2_pose = Pose()
    box2_pose.orientation = Quaternion(*q)  # Apply the same rotation
    box2_pose.position.x = 0.06  # Positioned exactly at the end of Box1
    box2_pose.position.y = -0.06
    box2_pose.position.z = -0.04  # Half of box2's height to stand upright

    # Assign the primitives and their poses to the CollisionObject
    l_shape.primitives = [box1, box2]
    l_shape.primitive_poses = [box1_pose, box2_pose]
    l_shape.operation = CollisionObject.ADD

    rospy.loginfo("Created properly oriented L-shaped collision object with two connected boxes.")
    return l_shape

def main():
    # Initialize MoveIt! Commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("attach_proper_l_shape", anonymous=True)

    # Instantiate RobotCommander and PlanningSceneInterface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate MoveGroupCommander for the panda_arm
    move_group = moveit_commander.MoveGroupCommander("panda_arm")

    # Allow some time for the PlanningSceneInterface to initialize
    rospy.sleep(2)

    # Retrieve the end effector link name dynamically
    end_effector_link = move_group.get_end_effector_link()
    rospy.loginfo(f"End effector link identified as: {end_effector_link}")

    # Create the properly oriented L-shaped collision object
    l_shape = create_properly_oriented_l_shape(end_effector_link)

    # Add the L-shaped object to the planning scene
    rospy.loginfo("Adding properly oriented L-shaped antenna to the planning scene...")
    scene.add_object(l_shape)
    rospy.sleep(2)  # Wait for the object to be added

    # Attach the L-shaped object to the gripper flange
    rospy.loginfo("Attaching properly oriented L-shaped antenna to the gripper flange...")
    #touch_links = robot.get_link_names(group="panda_arm")
    scene.attach_box(end_effector_link, l_shape.id, touch_links=["panda_link7","panda_link8"])
    rospy.sleep(1)

    # Verify attachment
    attached_objects = scene.get_attached_objects([l_shape.id])
    if l_shape.id in attached_objects:
        rospy.loginfo("Properly oriented L-shaped antenna successfully attached to the gripper flange.")
    else:
        rospy.logwarn("Failed to attach properly oriented L-shaped antenna to the gripper flange.")

    rospy.loginfo("L-shaped antenna attachment complete. Exiting script.")

    # Shutdown MoveIt! commander
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
