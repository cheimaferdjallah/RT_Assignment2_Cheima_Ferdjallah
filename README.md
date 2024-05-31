# RT_Assignment2_Cheima_Ferdjallah

# The purpose of assignment
The purpose of this assignment is to construct a ROS package for a robot simulation in Gazebo and Rviz. The package will consist of three separate nodes:

- An action client node that allows the user to specify a target location or cancel it, and also publishes the robot's position and velocity as a custom message using values from the topic /odom.
- A service node that, when called, returns the coordinates of the last target sent by the user.
- A node that subscribes to the robot's position and velocity using the custom message and prints the distance of the robot from the target and the robot's average speed. A parameter will be used to set the frequency of publishing the information.
- A launch file will be created to start the entire simulation.
## How to RUN the code
-----------------------------

1. Clone the repository inside the src file of your ROS workspace using:
```bash
git clone https://github.com/cheimaferdjallah/RT_Assignment2_Cheima_Ferdjallah.git
```
2. When done, run the command:
```bash
$ catkin_make
```
3.Execute the following command to launch the nodes (excluding action_client_node.py) in separate terminals:

```bash
$ roslaunch assignment_2_2023 assignment1.launch
```
4.Run action_client_node.py in a Separate Terminal to isolate its output using:
```bash
$ rosrun assignment_2_2023 action_client_node.py
```
# NODES
1. Action Client Node: action_client_node.py
   This code is a Python script that uses the ROS framework to control a robot. The script creates an action client that sends goals to the /reaching_goal action server, and a subscriber to the /odom topic to receive Odometry messages, extracts information, and publishes a CustomMessage on the /custom_topic topic.

## Pseudo code
----------------------
```python
Initialize picked_up_markers list
Initialize reference_token as None
Initialize ref_token_code as 0
Initialize threshold distances a_th and d_th

Define drive function(speed, seconds):
    Set robot motors power to speed
    Sleep for seconds duration
    Stop robot motors
Define turn function(speed, seconds):
    Set left motor power to speed
    Set right motor power to -speed
    Sleep for seconds duration
    Stop robot motors

Define find_token function():
    While no markers seen:
        Turn robot right slightly
        If no markers seen:
            Continue loop
    Print number of markers seen
    Initialize marker as None
    Initialize dist to a high value
    Iterate over markers seen:
        If marker is closer than dist and not picked or reference code:
            Update dist, rot_y, and marker
        If marker already picked or reference code:
            Print "Already picked or reference"
            Update marker
    If no marker found or all markers picked:
        Return None, -1, -1
    Else:
        Return marker, its distance, and angle

Define save_reference_token function():
    Find a token using find_token
    If a token is found:
        Set it as reference_token
        Update ref_token_code
        Print "Reference token saved"
    Else:
        Print "No token found"

Define displace_token function():
    Find markers in sight
    Set found_it as False
    Iterate over markers:
        If marker code matches reference code:
            Update reference_token and found_it
            Break loop
    If reference token found:
        While token not displaced:
            Get distance and angle to reference token
            If distance is within threshold:
                Release token and print "Released the token"
                Set found_it to False
                Break loop
            Else:
                Print "Not close enough"
                Adjust robot's position towards reference token
                Recursively call displace_token
    Else:
        Print "Reference token not found"
        Adjust robot's position and call displace_token recursively
        
Main code
Call save_reference_token

Print reference_token code
While true:
    Get token, distance, and angle using find_token
    If no token seen:
        Print "No token found"
        Turn right slightly
    Else, if token within grabbing distance:
        Print "Found it"
        If able to grab token:
            Print "Gotcha!"
            Displace token
            Drive backward
            Turn left
            Append token code to picked_up_markers
            Print picked_up_markers
    Else:
        Print "Not close enough"
        Adjust robot's position towards token
    Adjust robot's position based on angle to token
    If all markers picked:
        Print "Done!"
        Break loop
```



