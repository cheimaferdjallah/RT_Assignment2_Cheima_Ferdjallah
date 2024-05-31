#!/usr/bin/env python


import rospy
import actionlib
from assignment_2_2023.msg import PlanningAction, PlanningGoal, CustomMessage
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatus

class ActionClientNode:
    
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('action_client_node')

        # Create the action client
        self.client = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
        self.client.wait_for_server()

        # Initialize goal positions
        self.goal_x = None
        self.goal_y = None

        # Create subscribers and publishers
        self.custom_pub = rospy.Publisher('/custom_topic', CustomMessage, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def set_goal(self, x, y):
        
        # Store the goal position
        self.goal_x = x
        self.goal_y = y

        # Create a goal with the specified target position (x, y)
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Send the goal to the action server with feedback callback
        self.client.send_goal(goal, done_cb=self.goal_done_callback)
        rospy.loginfo("Goal sent: x={}, y={}\n".format(x, y))
        
        
    def goal_done_callback(self, status, result):
       
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal reached\n")


    def cancel_goal(self):
        
        # Cancel the goal
        self.client.cancel_goal()
        rospy.loginfo("Goal cancelled\n")

    def odom_callback(self, odom_msg):
        
        # Process the Odometry message and extract relevant information
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        vel_x = odom_msg.twist.twist.linear.x
        vel_z = odom_msg.twist.twist.angular.z

        # Create a CustomMessage with the extracted information
        custom_msg = CustomMessage()
        custom_msg.x = x
        custom_msg.y = y
        custom_msg.vel_x = vel_x
        custom_msg.vel_z = vel_z

        # Publish the CustomMessage on the /custom_topic topic
        self.custom_pub.publish(custom_msg)

    def main_loop(self):
       
        while not rospy.is_shutdown():
            try:
                # Get target position from user input
                x = float(input("Enter target x position: "))
                y = float(input("Enter target y position: "))
                # Call the set_goal function with user-input target position
                self.set_goal(x, y)
                cancel_input = input("Type 'cancel' to cancel immediately or press 'enter' to continue: \n")
                if cancel_input.lower() == 'cancel':
                    # Call the cancel_goal function
                    self.cancel_goal()
                
            except ValueError:
                rospy.logerr("Invalid input. Please enter numerical values.")

if __name__ == "__main__":
   
    action_client_node = ActionClientNode()
    action_client_node.main_loop()
    rospy.spin()


