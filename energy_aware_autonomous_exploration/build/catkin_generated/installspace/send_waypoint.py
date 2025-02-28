#!/usr/bin/env python3

import rospy
import numpy as np
from agiros_msgs.msg import Reference, Command, QuadState, Setpoint
# from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Transform, Twist, PoseStamped, Point, Quaternion, Vector3, Pose
from std_msgs.msg import Header





def send_waypoint():
    rospy.init_node('waypoint_publisher', anonymous=True)
    
    # Publisher to the Agilicious go_to_pose topic
    waypoint_pub = rospy.Publisher('/kingfisher/agiros_pilot/go_to_pose', PoseStamped, queue_size=10)
    
    rospy.sleep(2)  # Give time for publisher to register

    # Create a PoseStamped message
    waypoint = PoseStamped()
    waypoint.header.stamp = rospy.Time.now()
    waypoint.header.frame_id = "world"  # Change frame_id if needed
    waypoint.pose.position.x = 0.0    # Set your desired X position
    waypoint.pose.position.y = 0.0    # Set your desired Y position
    waypoint.pose.position.z = 1.0    # Set your desired altitude (Z)

    # Orientation (Quaternion)
    waypoint.pose.orientation.x = 0.0
    waypoint.pose.orientation.y = 0.0
    waypoint.pose.orientation.z = 0.0
    waypoint.pose.orientation.w = 1.0  # No rotation

    rospy.loginfo("Publishing waypoint to Agilicious framework...")
    waypoint_pub.publish(waypoint)
    rospy.loginfo(f"Waypoint sent: {waypoint.pose.position}")

    rospy.sleep(1)  # Allow some time for the message to be processed



def send_waypoints(waypoints):
    rospy.init_node('waypoint_publisher', anonymous=True)
    waypoint_pub = rospy.Publisher('/kingfisher/agiros_pilot/go_to_pose', PoseStamped, queue_size=10)
    
    rospy.sleep(2)  # Allow publisher to initialize

    for i, (x, y, z) in enumerate(waypoints):
        waypoint = PoseStamped()
        waypoint.header.stamp = rospy.Time.now()
        waypoint.header.frame_id = "map"

        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.pose.position.z = z
        waypoint.pose.orientation.w = 1.0  # No rotation

        rospy.loginfo(f"Sending waypoint {i+1}: ({x}, {y}, {z})")
        waypoint_pub.publish(waypoint)
        
        rospy.sleep(5)  # Wait before sending the next waypoint






# def create_trajectory():
#     rospy.init_node('send_trajectory', anonymous=True)
#     pub = rospy.Publisher('/kingfisher/agiros_pilot/trajectory', Reference, queue_size=10)
    
#     # Wait for publisher to establish connection
#     rospy.sleep(1.0)

#     # Create the Reference message
#     trajectory_msg = Reference()
#     trajectory_msg.header.stamp = rospy.Time.now()

#     # Define trajectory waypoints
#     waypoints = []

#     # --- WAYPOINT 1: Initial Hover Position ---
#     state_1 = QuadState()
#     state_1.header.stamp = rospy.Time.now()
#     state_1.t = 0.0
#     state_1.pose.position.x = 0.0
#     state_1.pose.position.y = 0.0
#     state_1.pose.position.z = 1.5  # Hover at 1.5m
#     state_1.velocity.linear = Vector3(0.0, 0.0, 0.0)
#     state_1.acceleration.linear = Vector3(0.0, 0.0, 0.0)
    
#     command_1 = Command()
#     command_1.t = 0.0
#     command_1.collective_thrust = 9.81  # Hover thrust

#     setpoint_1 = Setpoint()
#     setpoint_1.state = state_1
#     setpoint_1.command = command_1
#     waypoints.append(setpoint_1)

#     # --- WAYPOINT 2: Move Forward 2m ---
#     state_2 = QuadState()
#     state_2.header.stamp = rospy.Time.now()
#     state_2.t = 2.0
#     state_2.pose.position.x = 2.0  # Move 2m forward
#     state_2.pose.position.y = 0.0
#     state_2.pose.position.z = 1.5
#     state_2.velocity.linear = Vector3(1.0, 0.0, 0.0)  # Move forward
#     state_2.acceleration.linear = Vector3(0.0, 0.0, 0.0)

#     command_2 = Command()
#     command_2.t = 2.0
#     command_2.collective_thrust = 9.81  # Maintain altitude

#     setpoint_2 = Setpoint()
#     setpoint_2.state = state_2
#     setpoint_2.command = command_2
#     waypoints.append(setpoint_2)

#     # --- WAYPOINT 3: Turn Left (or Right) ---
#     state_3 = QuadState()
#     state_3.header.stamp = rospy.Time.now()
#     state_3.t = 4.0
#     state_3.pose.position.x = 2.5  # Move slightly forward
#     state_3.pose.position.y = 1.0  # Turn left (positive y), for right use -1.0
#     state_3.pose.position.z = 1.5
#     state_3.velocity.linear = Vector3(0.5, 0.5, 0.0)  # Diagonal movement
#     state_3.acceleration.linear = Vector3(0.0, 0.0, 0.0)

#     command_3 = Command()
#     command_3.t = 4.0
#     command_3.collective_thrust = 9.81  # Maintain altitude

#     setpoint_3 = Setpoint()
#     setpoint_3.state = state_3
#     setpoint_3.command = command_3
#     waypoints.append(setpoint_3)

#     # Add waypoints to trajectory
#     trajectory_msg.points = waypoints

#     # Publish trajectory
#     rospy.loginfo("Publishing trajectory to Agilicious...")
#     pub.publish(trajectory_msg)

#     rospy.sleep(1.0)  # Give time for ROS to process before shutting down







# Function to normalize a quaternion
def normalize_quaternion(q):
    norm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
    return Quaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm)

# Function to create a trajectory with smooth turns
def create_trajectory():
    rospy.init_node("trajectory_publisher", anonymous=True)
    pub = rospy.Publisher("/kingfisher/agiros_pilot/trajectory", Reference, queue_size=10)
    rospy.sleep(2)  # Ensure publisher is registered

    trajectory_msg = Reference()

    # Define waypoints: (x, y, z, yaw)
    # waypoints = [
    #     (0.0, 0.0, 1.0, 0.0),   # Start
    #     (2.0, 0.0, 1.0, 0.0),   # Forward 2m
    #     (2.5, 0.5, 1.0, 0.1),   # Slight turn
    #     (3.0, 1.0, 1.0, 0.2),   # Continue turn
    #     (3.5, 1.5, 1.0, 0.3)    # Complete turn
    # ]
    waypoints = [
        (0.0, 0.0, 1.0, 0.0),   
        (1.0, 0.0, 1.0, 0.0), 
        (2.0, 0.0, 1.0, 0.0),
        (3.0, 0.0, 1.0, 0.0)
    ]


    for i, (x, y, z, yaw) in enumerate(waypoints):
        setpoint = Setpoint()
        setpoint.state = QuadState()
        setpoint.command = Command()

        # Set timestamp
        setpoint.state.header.stamp = rospy.Time.now()
        setpoint.command.header.stamp = rospy.Time.now()

        # Assign position
        setpoint.state.pose.position.x = x
        setpoint.state.pose.position.y = y
        setpoint.state.pose.position.z = z

        # Convert yaw to quaternion
        q_w = np.cos(yaw / 2)
        q_z = np.sin(yaw / 2)
        setpoint.state.pose.orientation = normalize_quaternion(Quaternion(0.0, 0.0, q_z, q_w))

        # Set velocity
        setpoint.state.velocity.linear = Vector3(0.1, 0.0, 0.0)  # Move smoothly
        setpoint.state.velocity.angular = Vector3(0.0, 0.0, 0.0)

        # Set acceleration (optional)
        # setpoint.state.acceleration.linear = Vector3(0.1, 0.0, 0.0)
        # setpoint.state.acceleration.angular = Vector3(0.0, 0.0, 0.0)

        # Set jerk & snap (optional)
        # setpoint.state.jerk = Vector3(0.05, 0.05, 0.0)
        # setpoint.state.snap = Vector3(0.0, 0.0, 0.0)

        # Set motor commands (dummy values)
        # setpoint.command.collective_thrust = 9.81  # Gravity compensation
        # setpoint.command.bodyrates = Vector3(0.0, 0.0, 0.0)  # Slight yaw rotation
        # setpoint.command.thrusts = [1.0, 1.0, 1.0, 1.0]

        # Append to trajectory
        trajectory_msg.points.append(setpoint)

    rospy.loginfo("Publishing trajectory...")
    pub.publish(trajectory_msg)
    rospy.loginfo("Trajectory sent successfully!")






# Send a single waypoint
# if __name__ == '__main__':
#     try:
#         send_waypoint()
#     except rospy.ROSInterruptException:
#         pass


# Send multiple waypoints
if __name__ == '__main__':
    waypoints = [
        (1.0, 0.0, 1.0),
        (2.0, 0.0, 1.0),
        (3.0, -0.5, 1.0),
        (4.0, -0.5, 1.0)
    ]
    try:
        send_waypoints(waypoints)
    except rospy.ROSInterruptException:
        pass


# Send a trajectory
# if __name__ == "__main__":
#     try:
#         create_trajectory()
#     except rospy.ROSInterruptException:
#         pass