#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller')

        # Subscribe to waypoints
        rospy.Subscriber('waypoints', String, self.execute_waypoints)
        
        rospy.loginfo("Robot controller node started.")
        rospy.spin()

    def execute_waypoints(self, msg):
        waypoints = msg.data.split()
        
        # Simulate robot moving to each waypoint
        for key in waypoints:
            rospy.loginfo(f"Moving to key: {key}")
            rospy.sleep(1)  # Simulate delay for movement
            rospy.loginfo(f"Pressing key: {key}")
            rospy.sleep(0.5)  # Simulate key press

        rospy.loginfo("Finished typing the word.")

if __name__ == '__main__':
    try:
        RobotController()
    except rospy.ROSInterruptException:
        pass
