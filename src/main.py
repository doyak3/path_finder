#!/usr/bin/env python3

import matplotlib.pyplot as plt
from plot_me import *
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, Int32MultiArray
import numpy as np
from collections import deque

def bfs_recursive(graph, start, goal):
    queue = deque([[start]])
    visited = set([start])
    
    while queue:
        path = queue.popleft()
        current = path[-1]
        
        if current == goal:
            return path
            
        for neighbor in graph.get(current, []):
            if neighbor not in visited:
                visited.add(neighbor)
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
    
    return None

def path_publisher(shortest_path):
    rospy.init_node('path_publisher', anonymous=True)
    pub = rospy.Publisher('path_topic', Int32MultiArray, queue_size=10)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        if shortest_path:
            msg = Int32MultiArray()
            msg.data = shortest_path
            pub.publish(msg)
            rospy.loginfo(f"Published path: {shortest_path}")
        rate.sleep()

def main():
    # Define the building graph
    building_graph = {
        100: [101, 402],
        101: [100, 103],
        103: [101, 201],
        201: [103, 301],
        301: [201, 402],
        402: [301, 100]
    }

    # Get user input
    try:
        start_node = int(input("Enter the start node (e.g., 100): ").strip())
        goal_node = int(input("Enter the goal node (e.g., 402): ").strip())
    except ValueError:
        print("Please enter valid integer node numbers")
        return

    # Find the shortest path
    shortest_path = bfs_recursive(building_graph, start_node, goal_node)

    if shortest_path is None:
        print(f"No path found from {start_node} to {goal_node}.")
        shortest_path = []  # Empty list for ROS message
    else:
        print(f"Shortest path from {start_node} to {goal_node}: {shortest_path}")
        # Plot the building layout if plot_building_layout is defined
        try:
            plot_building_layout(building_graph, positions, shortest_path)
        except NameError:
            print("plot_building_layout function not available")

    # Start ROS publisher
    try:
        path_publisher(shortest_path)
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
