#!/usr/bin/env python3

import rospy
from keyboard_sim.srv import GetShortestPath
from std_msgs.msg import String

class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner')

        # Wait for the shortest path service
        rospy.wait_for_service('get_shortest_path')
        self.get_shortest_path = rospy.ServiceProxy('get_shortest_path', GetShortestPath)
        
        # Subscribe to the word to type
        rospy.Subscriber('word_to_type', String, self.plan_word_path)
        
        # Publisher to send waypoints to the robot controller
        self.waypoint_pub = rospy.Publisher('waypoints', String, queue_size=10)
        
        rospy.loginfo("Path planner node started.")
        rospy.spin()

    def plan_word_path(self, msg):
        word = msg.data.upper()
        waypoints = []

        # Generate waypoints for typing each character
        for i in range(len(word) - 1):
            start_char = word[i]
            end_char = word[i + 1]
            try:
                response = self.get_shortest_path(start=start_char, end=end_char)
                if response.path:
                    waypoints.extend(response.path)
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
        
        # Publish waypoints as a string
        self.waypoint_pub.publish(' '.join(waypoints))

if __name__ == '__main__':
    try:
        PathPlanner()
    except rospy.ROSInterruptException:
        pass
