#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

def add_flat_table_and_rectangle():
    # Initialize MoveIt Commander and ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('add_flat_table_and_rectangle_node', anonymous=True)

    # Initialize the PlanningSceneInterface
    scene = moveit_commander.PlanningSceneInterface()
    rospy.sleep(1)  # Allow time for the interface to initialize

    # -----------------------
    # Add Table
    # -----------------------
    table = CollisionObject()
    table.id = "flat_table"
    table.header.frame_id = "panda_link0"  # Reference frame (robot's base)

    # Define the table's shape and dimensions
    table_primitive = SolidPrimitive()
    table_primitive.type = SolidPrimitive.BOX
    table_primitive.dimensions = [1.0, 1.0, 0.05]  # [x, y, z] in meters

    # Define the table's pose
    table_pose = PoseStamped()
    table_pose.header.frame_id = "panda_link0"
    table_pose.pose.position.x = 0.6  # **Change Table X Position Here/orig=0.3**
    table_pose.pose.position.y = 0.2  # **Change Table Y Position Here**
    table_pose.pose.position.z = -0.025  # **Change Table Z Position Here/origanl was -0.025**
    table_pose.pose.orientation.w = 1.0  # No rotation

    # Assign the shape and pose to the CollisionObject
    table.primitives = [table_primitive]
    table.primitive_poses = [table_pose.pose]
    table.operation = CollisionObject.ADD

    # Add the table to the planning scene
    scene.add_object(table)
    rospy.loginfo("Flat table added to the planning scene.")
    rospy.sleep(1)  # Allow time for the object to be added

    # -----------------------
    # Add Rectangle on Table
    # -----------------------
    rectangle = CollisionObject()
    rectangle.id = "table_rectangle"
    rectangle.header.frame_id = "panda_link0"  # Same reference frame as the table

    # Define the rectangle's shape and dimensions
    rectangle_primitive = SolidPrimitive()
    rectangle_primitive.type = SolidPrimitive.BOX
    rectangle_primitive.dimensions = [0.06, 0.04, 0.13]  # [x, y, z] in meters (z-axis)0.3+ 10 =  cm for offset of 0.1 in OTA

    # Define the rectangle's pose
    rectangle_pose = PoseStamped()
    rectangle_pose.header.frame_id = "panda_link0"
    # Position the rectangle on the table's surface
    rectangle_pose.pose.position.x = 0.48  # **Change Rectangle X Position Here** previous setting was 0.48
    rectangle_pose.pose.position.y = 0.17  # **Change Rectangle Y Position Here**previous setting was 0.17
    rectangle_pose.pose.position.z = -0.025 + 0.05 + 0.005  # On top of the table
    rectangle_pose.pose.orientation.w = 1.0  # No rotation

    # Assign the shape and pose to the CollisionObject
    rectangle.primitives = [rectangle_primitive]
    rectangle.primitive_poses = [rectangle_pose.pose]
    rectangle.operation = CollisionObject.ADD

    # Add the rectangle to the planning scene
    scene.add_object(rectangle)
    rospy.loginfo("Rectangle added to the table.")
    rospy.sleep(1)  # Allow time for the object to be added

    # -----------------------
    # Add Visualization Markers for Coloring
    # -----------------------
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(1)  # Allow time for the publisher to connect

    # ----- Marker for Table (Brown) -----
    table_marker = Marker()
    table_marker.header.frame_id = "panda_link0"
    table_marker.header.stamp = rospy.Time.now()
    table_marker.ns = "objects"
    table_marker.id = 0
    table_marker.type = Marker.CUBE
    table_marker.action = Marker.ADD
    table_marker.pose = table_pose.pose
    table_marker.scale.x = table_primitive.dimensions[0]
    table_marker.scale.y = table_primitive.dimensions[1]
    table_marker.scale.z = table_primitive.dimensions[2]
    table_marker.color.r = 0.6  # Brown color
    table_marker.color.g = 0.3
    table_marker.color.b = 0.0
    table_marker.color.a = 1.0  # Fully opaque

    marker_publisher.publish(table_marker)
    rospy.loginfo("Visualization marker for table published.")
    rospy.sleep(1)  # Allow time for the marker to appear

    # ----- Marker for Rectangle (Black) -----
    rectangle_marker = Marker()
    rectangle_marker.header.frame_id = "panda_link0"
    rectangle_marker.header.stamp = rospy.Time.now()
    rectangle_marker.ns = "objects"
    rectangle_marker.id = 1
    rectangle_marker.type = Marker.CUBE
    rectangle_marker.action = Marker.ADD
    rectangle_marker.pose = rectangle_pose.pose
    rectangle_marker.scale.x = rectangle_primitive.dimensions[0]
    rectangle_marker.scale.y = rectangle_primitive.dimensions[1]
    rectangle_marker.scale.z = rectangle_primitive.dimensions[2]
    rectangle_marker.color.r = 0.9  # Black color
    rectangle_marker.color.g = 0.0
    rectangle_marker.color.b = 0.2
    rectangle_marker.color.a = 1.0  # Fully opaque

    marker_publisher.publish(rectangle_marker)
    rospy.loginfo("Visualization marker for rectangle published.")
    rospy.sleep(1)  # Allow time for the marker to appear

if __name__ == '__main__':
    try:
        add_flat_table_and_rectangle()
    except rospy.ROSInterruptException:
        pass
