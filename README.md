# RT_Assignment2_Cheima_Ferdjallah

# The purpose of the assignment
The purpose of this assignment is to construct a ROS package for a robot simulation in Gazebo and Rviz. The package will consist of three separate nodes:

- An action client node that allows the user to specify a target location or cancel it, and also publishes the robot's position and velocity as a custom message using values from the topic /odom.
- A service node that, when called, returns the coordinates of the last target sent by the user.
- A node that subscribes to the robot's position and velocity using the custom message and prints the distance of the robot from the target and the robot's average speed. A parameter will be used to set the frequency of publishing the information.
- A launch file will be created to start the entire simulation.
  
# How to RUN the code
-----------------------------

1. Clone the repository inside the src file of your ROS workspace using:
```bash
git clone https://github.com/cheimaferdjallah/RT_Assignment2_Cheima_Ferdjallah.git
```
2. When done, run the command:
```bash
$ catkin_make
```
3. Execute the following command to launch the nodes (excluding action_client_node.py) in separate terminals:

```bash
$ roslaunch assignment_2_2023 assignment1.launch
```
4. Run action_client_node.py in a Separate Terminal to isolate its output using:
```bash
$ rosrun assignment_2_2023 action_client_node.py
```
# NODES
----------------------
## 1. Action Client Node: action_client_node.py
   This code is a Python script that uses the ROS framework to control a robot. The script creates an action client that sends goals to the /reaching_goal action server, and a subscriber to the /odom topic to receive Odometry messages, extracts information, and publishes a CustomMessage on the /custom_topic topic.

### Pseudo code
----------------------
```python
#!/usr/bin/env python

Import rospy
Import actionlib
From assignment_2_2023.msg import PlanningAction, PlanningGoal, CustomMessage
From nav_msgs.msg import Odometry
From actionlib_msgs.msg import GoalStatus

Class ActionClientNode:
    
    Function __init__():
        Initialize ROS node
        Create action client
        Wait for action server to become available
        Initialize goal positions
        Create subscribers and publishers

    Function set_goal(x, y):
        Store goal position (x, y)
        Create goal with target position (x, y)
        Send goal to action server with feedback callback

    Function goal_done_callback(status, result):
        If status is SUCCEEDED:
            Log "Goal reached"

    Function cancel_goal():
        Cancel goal
        Log "Goal cancelled"

    Function odom_callback(odom_msg):
        Extract relevant information from Odometry message
        Create CustomMessage with extracted information
        Publish CustomMessage on /custom_topic topic

    Function main_loop():
        While ROS is running:
            Try:
                Get target position from user input
                Call set_goal function with user-input target position
                Prompt user to cancel immediately or continue
                If user wants to cancel:
                    Call cancel_goal function
            Except ValueError:
                Log error message for invalid input

Main:
    Create ActionClientNode object
    Run main loop
    Keep ROS spinning

```
## 2. Last Target Node: last_target_node.py

This node creates a service client to retrive the most recent target coordinates from the /get_last_target_coordinates service. Additionally, it subscribes to the /reaching_goal topic in order to receive updates on the last target position.

## 3. Robot Information Node: robot_info_node.py

This node offers a service named (/get_robot_pos_vel) which gives details regarding the robot's current position and velocity. It further subscribes to the /custom_topic topic, where it analyzes CustomMessage data to keep track of the robot's position and velocity. Additionally, it computes the distance to the most recent target and calculates the average speed since the last request.


