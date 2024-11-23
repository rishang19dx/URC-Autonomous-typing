#!/usr/bin/env python

import rospy
import networkx as nx
from std_msgs.msg import String
from keyboard_sim.srv import GetShortestPath, GetShortestPathResponse

class KeyboardGraphBuilder:
    def __init__(self):
        rospy.init_node('keyboard_graph_builder')
        
        # Build the keyboard graph
        self.keyboard_graph = self.build_keyboard_graph()
        
        # Start the service to provide shortest paths
        self.path_service = rospy.Service('get_shortest_path', GetShortestPath, self.handle_get_shortest_path)
        
        rospy.loginfo("Keyboard graph builder node started.")
        rospy.spin()
    
    def build_keyboard_graph(self):
        # Create an empty graph
        G = nx.Graph()
        keyboard_layout = [
            "1234567890",
            "QWERTYUIOP",
            "ASDFGHJKL",
            "ZXCVBNM"
        ]
        key_positions = {}
        for row_index, row in enumerate(keyboard_layout):
            for col_index, key in enumerate(row):
                key_positions[key] = (row_index, col_index)
                
        move_directions = [
            (-1, 0), (1, 0), (0, -1), (0, 1),
            (-1, -1), (-1, 1), (1, -1), (1, 1)
        ]
        for key, (row, col) in key_positions.items():
            G.add_node(key)
            for dr, dc in move_directions:
                neighbor_row, neighbor_col = row + dr, col + dc
                for neighbor_key, (n_row, n_col) in key_positions.items():
                    if neighbor_row == n_row and neighbor_col == n_col:
                        G.add_edge(key, neighbor_key, weight=1)
        return G

    def handle_get_shortest_path(self, req):
        try:
            # Compute the shortest path using Dijkstra's algorithm
            path = nx.shortest_path(self.keyboard_graph, source=req.start, target=req.end)
            return GetShortestPathResponse(path)
        except nx.NetworkXNoPath:
            rospy.logerr(f"No path found between {req.start} and {req.end}")
            return GetShortestPathResponse([])

if __name__ == '__main__':
    try:
        KeyboardGraphBuilder()
    except rospy.ROSInterruptException:
        pass
