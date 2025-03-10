#!/usr/bin/env python3

from collections import deque
import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import csv

# Parameters
k = 0.1  # Look forward gain
Lfc = 4.0  # [m] Look-ahead distance
Kp = 1.0  # Speed proportional gain

dt = 0.1  # [s] Time tick
WB = 2.1  # [m] Wheel base of vehicle
max_cmd_vel = 2.9  # Maximum value for /cmd_vel
target_speed = 2.5  # [m/s] Target speed

# Global variables to store odometry data
odom_x = 0.0
odom_y = 0.0
odom_yaw = 0.0
odom_velocity = 0.0

# Global buffers for storing odometry data for median filter
buffer_size = 5
odom_x_buffer = deque(maxlen=buffer_size)
odom_y_buffer = deque(maxlen=buffer_size)
odom_yaw_buffer = deque(maxlen=buffer_size)

# Global lists for storing trajectory and reference path
trajectory_x = []
trajectory_y = []

# Global variable to store the last target index
last_target_index = 0

# Global variable to track total distance traveled
total_distance = 0.0

# Generate reference path
reference_path_x = []
reference_path_y = []
row_count = 0
max_rows = 165  # Limit to the first 132 rows

with open('/home/omen/Downloads/output_file.csv', 'r') as csv_file:
    reader = csv.reader(csv_file)
    next(reader)  # Skip header row
    for row in reader:
        if row_count >= max_rows:
            break
        reference_path_x.append(float(row[0]))
        reference_path_y.append(float(row[1]))
        row_count += 1

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(math.radians(self.yaw)))
        self.rear_y = self.y - ((WB / 2) * math.sin(math.radians(self.yaw)))

    def update(self, a, delta):
        self.x += self.v * math.cos(math.radians(self.yaw)) * dt
        self.y += self.v * math.sin(math.radians(self.yaw)) * dt
        self.yaw += (self.v / WB) * math.degrees(math.atan(math.tan(math.radians(delta)))) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(math.radians(self.yaw)))
        self.rear_y = self.y - ((WB / 2) * math.sin(math.radians(self.yaw)))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

def proportional_control(target, current):
    a = Kp * (target - current)
    scaled_a = max(0.0, min(0.3, a / max_cmd_vel))
    return scaled_a

def pure_pursuit_steer_control(state):
    global last_target_index
    Lf = 4.0  # Look-ahead distance
    target_index = last_target_index

    for i in range(last_target_index, len(reference_path_x)):
        dx = reference_path_x[i] - state.x
        dy = reference_path_y[i] - state.y
        distance = math.hypot(dx, dy)
        
        if distance > Lf:
            target_index = i
            break

    # Reset target index if it reaches the end
    if target_index == len(reference_path_x) - 1:
        target_index = 0
    
    # Use weighted average to smooth the transition between lookahead points
    current_target_x = reference_path_x[target_index]
    current_target_y = reference_path_y[target_index]
    last_target_x = reference_path_x[last_target_index]
    last_target_y = reference_path_y[last_target_index]
    
    target_x = 0.5 * current_target_x + 0.5 * last_target_x
    target_y = 0.5 * current_target_y + 0.5 * last_target_y

    # Update last target index
    last_target_index = target_index

    alpha = math.atan2(target_y - state.y, target_x - state.x) - math.radians(state.yaw)
    delta = math.degrees(math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0))

    delta = max(-28, min(28, delta))
    
    return delta, Lf, target_x, target_y

def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    plt.arrow(x, y, length * math.cos(math.radians(yaw)), length * math.sin(math.radians(yaw)),
              fc=fc, ec=ec, head_width=width, head_length=width)
    plt.plot(x, y, 'ok')

def plot_lookahead_point(target_x, target_y):
    plt.plot(target_x, target_y, 'or', label='Lookahead Point')

def odom_callback(msg):
    global odom_x, odom_y, odom_yaw, odom_velocity, previous_state, odom_x_buffer, odom_y_buffer, odom_yaw_buffer
    
    position = msg.pose.pose.position
    orientation = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    yaw = math.degrees(yaw)
    
    # Add new data to buffers
    odom_x_buffer.append(position.x)
    odom_y_buffer.append(position.y)
    odom_yaw_buffer.append(yaw)

    # Compute median values
    odom_x = np.median(odom_x_buffer)
    odom_y = np.median(odom_y_buffer)
    odom_yaw = np.median(odom_yaw_buffer)

    state = State(x=odom_x, y=odom_y, yaw=odom_yaw, v=msg.twist.twist.linear.x)

    odom_velocity = msg.twist.twist.linear.x
    previous_state = state


def plot_all_waypoints():
    plt.plot(reference_path_x, reference_path_y, 'k.', label='Waypoints')


def main():
    global previous_state, trajectory_x, trajectory_y, total_distance

    rospy.init_node('pure_pursuit_plot')
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/aft_mapped_adjusted', Odometry, odom_callback)
    steering_pub = rospy.Publisher('/in_Car_steering_in_degree', Float32, queue_size=1)
    velocity_pub = rospy.Publisher('/in_Car_velocity_in_KM/H', Float32, queue_size=1)

    previous_state = State(x=0.0, y=0.0, yaw=0.0, v=0.0)

    plt.ion()

    prev_x, prev_y = odom_x, odom_y

    while not rospy.is_shutdown():
        if previous_state is not None:
            ai = proportional_control(target_speed, previous_state.v) * max_cmd_vel
            di, Lf, target_x, target_y = pure_pursuit_steer_control(previous_state)
               
            previous_state.update(ai, di)
            
            # Calculate the distance from the previous position to the current position
            distance_increment = math.hypot(odom_x - prev_x, odom_y - prev_y)
            total_distance += distance_increment
            prev_x, prev_y = odom_x, odom_y
            
            if (total_distance < 90):

                steering_pub.publish(Float32(-di))
                velocity_pub.publish(Float32(1.8))

            else:
                steering_pub.publish(Float32(0.0))
                velocity_pub.publish(Float32(0.0))
               

            print(f"Odometry - X Position: {odom_x:.2f} m")
            print(f"Odometry - Y Position: {odom_y:.2f} m")
            print(f"Odometry - Yaw Angle: {odom_yaw:.2f} degrees")
            print(f"Odometry - Velocity: {odom_velocity:.2f} m/s")
            print(f"Time: {rospy.get_time():.2f} s")
            print(f"Velocity: {previous_state.v:.2f} m/s")
            print(f"Steering Angle (delta): {di:.2f} degrees")
            print(f"Yaw Angle: {previous_state.yaw:.2f} degrees")
            print(f"Look-ahead Distance: {Lf:.2f} m")
            print(f"Total Distance Traveled: {total_distance:.2f} m")
            print("-----------------------------")

            trajectory_x.append(odom_x)
            trajectory_y.append(odom_y)

            plt.cla()
            plt.plot(reference_path_x, reference_path_y, '-g', label='Reference Path')
            plt.plot(trajectory_x, trajectory_y, '-b', label='Trajectory')
            plt.plot(odom_x, odom_y, 'ok', label='Current Position')
            plot_all_waypoints()
            plot_arrow(odom_x, odom_y, odom_yaw)
            plot_lookahead_point(target_x, target_y)
            plt.axis("equal")
            plt.grid(True)
            plt.title(f"Speed[km/h]: {previous_state.v * 3.6:.1f}")
            plt.legend()
            plt.draw()
            plt.pause(0.001)

    plt.ioff()
    plt.show()

if __name__ == '__main__':
    main()
