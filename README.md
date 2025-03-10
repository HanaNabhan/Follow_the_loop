# Pure Pursuit Path Tracking for Autonomous Vehicles

## Overview
This project implements a **Pure Pursuit Path Tracking Algorithm** for autonomous vehicles using **ROS (Robot Operating System)**. The algorithm processes odometry data to estimate the car's position, orientation, and speed, then calculates the steering angle needed to follow a predefined reference path.

<p align="center">
  <img src="https://github.com/HanaNabhan/Follow_the_loop/blob/main/real-life.jpg" alt="Real-life Car Trajectory" width="400"/>
  <img src="https://github.com/HanaNabhan/Follow_the_loop/blob/main/infinity.jpg" alt="Infinity Path" width="400"/>
</p>


## Features
- **Proportional Speed Control:** Dynamically adjusts vehicle speed based on target velocity.
- **Look-Ahead Distance Calculation:** Selects target waypoints to ensure smooth navigation.
- **Real-Time Visualization:** Plots the car's current position, trajectory, and look-ahead points using Matplotlib.
- **ROS Integration:**
  - Subscribes to odometry topics: `/odom` and `/aft_mapped_adjusted`
  - Publishes steering angles and velocity to topics: `/in_Car_steering_in_degree` and `/in_Car_velocity_in_KM/H`
- **Noise Reduction:** Implements a median filter to smooth odometry data.
- **Distance Tracking:** Monitors the total distance traveled and stops the car after reaching a set distance.

## How It Works
1. **Path Input:** The reference path is read from a CSV file containing waypoints (X, Y coordinates).
2. **Odometry Processing:** Odometry data is used to continuously update the car's state (position, yaw, velocity).
3. **Pure Pursuit Algorithm:**
   - Calculates the look-ahead point along the path.
   - Computes the steering angle needed to reach that point.
4. **Control Commands:** Steering and speed commands are published to drive the car.
5. **Visualization:** The car's trajectory and target path are plotted in real-time.

## Running the Code
1. Ensure ROS is installed and running.
2. Place your CSV file with waypoints at the specified path.
3. Run the script
4. Visualizations will open, and terminal logs will show real-time data (positions, angles, velocities).

## Video Demonstration
Watch the live demonstration of this project on [LinkedIn](https://www.linkedin.com/in/hana-nabhan/).




