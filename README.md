---
layout: default
---

Project Reflections
-------------------

Author: Roman Stanchak

Course: Self-Driving Car Engineer Nanodegree Program Term 3

Date: November, 26 2017

## The Model

There are three major components to my path planner:

1. Reference lane computation
2. Reference velocity computation
3. Trajectory generation 


### Reference lane computation

The lane planner uses a simple state-machine containing two states as below:

1. KEEP_LANE
2. CHANGE_LANE

The transitions conditions are as follows:

- KEEP_LANE => CHANGE_LANE: a lane change is safe and gets the vehicle to its destination faster
- CHANGE_LANE => KEEP_LANE: the current lane matches the referencelane, i.e. the lane change is complete

To determine the optimal lane, the function *calcBestLane* in PathPlanner.cpp analyzes the sensor fusion data as follows:

1. Discretize the road area around the vehicle's s position into a 5x3 grid in the s x d dimensions. Each d bin corresponds to a lane on the road, while each s-bins corresponds to 4 GM Hummer sized car lengths or about 20 meters. See below:

2. For each vehicle in the sensor fusion data, compute the bounding box in Frenet coordinates
3. For each bin in the grid, mark (a) the minimum vehicle velocity and (b) occupancy for any vehicle bounding box point in that bin.
4. Compute the minimum velocity of each lane in front of the vehicle, store this in the variable *vlanes*
5. Determine the 'best lane' as the lane with the maximum minimum velocity which is not occupied and which is adjacent to the current lane

### Reference velocity computation

The reference velocity refers to the velocity value used to compute the path points.  In meet physical, law and comfort limits, this reference velocity is only modified in increments of *max_acceleartion x dt* at every update iteration.

The target velocity refers to the eventual velocity, and corresponds to the lower of (a) the speed limit or (b) the velocity of the vehicle blocking the lane.

The target velocity is computed using *vlanes[current_lane]* from the *calcBestLanes* function, capped by the speed limit.

### Trajectory generation

This code computes a smooth trajectory based on the past trajectory, the reference lane, reference velocity and map waypoints.

The basic approach is as follows:

1. Select 5 reference points through which to fit a polynomial spline: one point behind the vehicle, one point at the vehicle origin and the 3 nearest map way points in the reference lane.
2. Fit a spline through these points
3. Extend the previous trajectory to contain exactly 50 points starting 

