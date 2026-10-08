#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
# fix eigenpy location
sys.path.append('/opt/ros/noetic/lib/python3.8/site-packages')
import rospy
import moveit_commander
import numpy as np
import geometry_msgs.msg
import eigenpy
from math import pi, degrees
import copy
import time


def get_A_T_G(phi, theta):
    """
    Computes the transformation matrix based on azimuthal and polar angles.
    """
    radius = 0.17  # Radius of the hemisphere (15 cm)
    x_position = radius * np.sin(theta) * np.cos(phi)
    y_position = radius * np.sin(theta) * np.sin(phi)
    z_position = radius * np.cos(theta)

    # Calculate direction to the origin for orientation
    direction_to_center = np.array([-x_position, -y_position, -z_position])
    norm = np.linalg.norm(direction_to_center)
    if norm == 0:
        direction_to_center = np.array([0.0, 0.0, -1.0])
    else:
        direction_to_center /= norm  # Normalize

    # Calculate rotation matrix
    z_axis = direction_to_center
    up = np.array([0.0, 0.0, 1.0])  # World Z-axis as reference
    if np.allclose(z_axis, up) or np.allclose(z_axis, -up):
        up = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(up, z_axis)
    if np.linalg.norm(x_axis) == 0:
        x_axis = np.array([1.0, 0.0, 0.0])  # Default X-axis if cross product is zero
    else:
        x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    rotation_matrix = np.column_stack((x_axis, y_axis, z_axis))

    # Build the transformation matrix
    gripper_frame = np.eye(4)
    gripper_frame[:3, :3] = rotation_matrix
    gripper_frame[:3, 3] = np.array([x_position, y_position, z_position])

    return gripper_frame


def get_wpose(angle, O_T_FG, move_group):
    """
    Computes the target pose with an additional rotation around the Z-axis.

    :param angle: Rotation angle in radians (e.g., 0 or pi).
    :param O_T_FG: Transformation matrix for the target position.
    :param move_group: MoveGroupCommander instance.
    :return: geometry_msgs.msg.Pose with updated orientation.
    """
    wpose = move_group.get_current_pose().pose
    mod = np.eye(4)
    mod[0:3, 0:3] = eigenpy.AngleAxis(angle, np.array([0., 0., 1.])).toRotationMatrix()
    T = np.matmul(O_T_FG, mod)
    wpose.position.x = T[0, 3]
    wpose.position.y = T[1, 3]
    wpose.position.z = T[2, 3]
    q = eigenpy.Quaternion(T[:3, :3])
    wpose.orientation.x = q.x
    wpose.orientation.y = q.y
    wpose.orientation.z = q.z
    wpose.orientation.w = q.w
    return wpose


def compute_plan(wpose, move_group):
    """
    Plans a motion to the given pose.

    :param wpose: Target pose.
    :param move_group: MoveGroupCommander instance.
    :return: Tuple of (plan, delta), where delta is the joint movement distance.
    """
    move_group.set_pose_target(wpose)
    move_group.set_max_velocity_scaling_factor(0.05)
    move_group.set_max_acceleration_scaling_factor(0.05)
    
    # Call the plan method and handle different return types
    plan_output = move_group.plan()
    
    # Determine if plan_output is a tuple, RobotTrajectory, or bool
    if isinstance(plan_output, tuple):
        # Newer versions might return (success, plan, planning_time, error_code)
        success = plan_output[0]
        if not success:
            return None, np.inf
        plan = plan_output[1]
    elif isinstance(plan_output, bool):
        # If a boolean is returned, retrieve the last plan
        if not plan_output:
            return None, np.inf
        plan = move_group.get_last_plan()
    else:
        # Assume plan_output is a RobotTrajectory
        plan = plan_output
    
    # Check if the plan is valid
    if not plan or not plan.joint_trajectory.points:
        return None, np.inf  # Invalid plan
    
    # Calculate joint movement delta
    q_first = plan.joint_trajectory.points[0].positions
    q_last = plan.joint_trajectory.points[-1].positions
    delta = np.linalg.norm(np.array(q_first) - np.array(q_last))
    
    return plan, delta


def main():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("panda_hemisphere_scan_with_rotation", anonymous=True)

    # Instantiate RobotCommander and PlanningSceneInterface
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate MoveGroupCommander for the panda arm
    move_group = moveit_commander.MoveGroupCommander("panda_arm")

    # Allow replanning to increase the odds of a solution
    move_group.allow_replanning(True)

    # Set planning parameters
    move_group.set_planning_time(2)  # Increased planning time
    move_group.set_num_planning_attempts(100)  # Increased number of attempts
    #move_group.set_max_velocity_scaling_factor(0.05)
    #move_group.set_max_acceleration_scaling_factor(0.05)

    # Move to the initial joint configuration (from your team lead's code)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.15
    joint_goal[1] = -0.16
    joint_goal[2] = 0.22
    joint_goal[3] = -2.41
    joint_goal[4] = 0.10
    joint_goal[5] = 2.25
    joint_goal[6] = 0.6
    move_group.go(joint_goal, wait=True)
    move_group.stop()

    # Wait for the robot to settle
    rospy.sleep(1.0)

    # Set up the kinematics
    # Define the transformation between the center of the sphere and the origin
    O_T_A = np.eye(4)
    current_pose = move_group.get_current_pose().pose
    O_T_A[0, 3] = current_pose.position.x
    O_T_A[1, 3] = current_pose.position.y
    O_T_A[2, 3] = 0.125  # Center point as in your original code

    # Capture the current gripper rotation
    o_gripper = current_pose.orientation
    q_gripper = eigenpy.Quaternion(o_gripper.w, o_gripper.x, o_gripper.y, o_gripper.z)
    O_R_FG = q_gripper.matrix()

    # Compute the gripper rotation at phi=0., theta=1e-3
    A_T_G_initial = get_A_T_G(0.0, 1e-3)
    O_R_G = A_T_G_initial[:3, :3]

    # Compute the difference between both frames
    G_T_FG = np.eye(4)
    G_T_FG[:3, :3] = np.matmul(np.linalg.inv(O_R_G), O_R_FG)

    # Set planner to RRTConnect (since LBKPIECE was causing issues)
    move_group.set_planner_id("RRTConnect")

    # Adjust goal tolerances
    move_group.set_goal_position_tolerance(0.005)  # 5 mm
    move_group.set_goal_orientation_tolerance(0.02)  # ~1.1 degrees
    move_group.attach_object("proper_l_shape_antenna",move_group.get_end_effector_link())

    # Define Phi and Theta Ranges with 10-degree increments
    phi_values = np.radians(np.arange(-180, 180,10 ))  # From -180° to 180° in 10° increments
    #phi_values = np.radians([-60.])  # From -180° to 180° in 10° increments

    # Define Theta Values from 0° to -70° in 10-degree increments
    theta_values = np.radians(np.arange(0, -71, -10))  # From 0° to -70° in -10° steps

    # Initialize direction for alternating phi ranges (optional for zig-zag pattern)
    direction = -1.0

    # Main loop to perform the hemispherical movement with integrated 180-degree rotation logic
    failed = False
    for phi in phi_values:
        # Toggle direction between -1 and 1 for each phi (optional)
        direction = 1.0 if direction == -1.0 else -1.0
        rospy.loginfo(f"Direction: {direction}")

        # Depending on direction, set the order of phi if needed
        # In this case, phi is already being iterated, so direction might influence other behaviors
        # Currently, direction is only printed; modify as needed for advanced patterns

        # Inner loop: Polar angle (theta)
        for theta in theta_values:
            theta_deg = degrees(theta)
            phi_deg = degrees(phi)
            rospy.loginfo(f"  Moving to phi = {phi_deg:.1f} deg, theta = {theta_deg:.1f} deg")

            # Compute the transformation matrix for current phi and theta
            A_T_G = get_A_T_G(phi, theta)
            O_T_FG = np.matmul(O_T_A, np.matmul(A_T_G, G_T_FG))

            # Generate two poses with 0° and 180° rotation
            wpose0 = get_wpose(0.0, O_T_FG, move_group)
            wposepi = get_wpose(pi, O_T_FG, move_group)

            # Compute plans for both poses
            rospy.loginfo("Planning")
            plan0, delta0 = compute_plan(wpose0, move_group)
            planpi, deltapi = compute_plan(wposepi, move_group)
            rospy.loginfo("Finished planning")

            # Select the plan with minimal joint movement
            if delta0 < deltapi:
                selected_plan = plan0
            else:
                selected_plan = planpi

            # Execute the selected plan if valid
            if selected_plan:
                res = move_group.execute(selected_plan, wait=True)
                if not res:
                    rospy.logwarn("Execution failed, aborting")
                    failed = True
                    break
                move_group.stop()
                move_group.clear_pose_targets()
            else:
                rospy.logwarn("    No valid plan found for this pose. Skipping.")
                continue

            # Wait before moving to the next pose (10-second delay after each theta increment)
            time.sleep(20.0)  # Add 10-second delay

        if failed:
            break

    # Clear path constraints after movement (if any)
    move_group.clear_path_constraints()

    # Shutdown moveit_commander
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
