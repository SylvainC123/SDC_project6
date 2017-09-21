# Extended Kalman Filter Project Starter Code
This Project is the sixth task (Project 2 of Term 2) of the Udacity Self-Driving Car Nanodegree program. The main goal of the project is to apply Extended Kalman Filter to fuse data from LIDAR and Radar sensors of a self driving car using C++.

# Method: #

I have not been able to use the simulator due to communication problems between the simulator and win7/docker.

therefore I have used the old method of a text output, I have then used excel to review and analyse the results (file not attached due to size restriction).

note:
To run the code the Eigen library needs to be linked

# Results: #

## position: ##
as expected there is some discrepancies in the turns between the ground truth and the estimation.

![image](position.jpg)

## speed : ##
some initial discrepancies then fluctuation around the ground truth.

![image](speed.jpg)

## distance error: ##

![image](error.png)