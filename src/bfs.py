import matplotlib.pyplot as plt


def bfs_recursive(graph, start, goal, path=[], visited=None):
    if visited is None:
        visited = set()  # Initialize visited set on the first call
    
    # Add the start node to path and mark it as visited
    path.append(start)  
    visited.add(start)  

    # Base case: If we reached the goal, return the current path
    if start == goal:  ##???아닌거같은디
        return path[:]    

    # Explore each neighbor
    for neighbor in graph.get(start, []):
        if neighbor not in visited:  # TODO: Check if the neighbor is unvisited
            result = bfs_recursive(graph, neighbor, goal, path, visited)
            if result is not None:  # Found a valid path
                return result  ##수정필요    

    # Backtrack: Remove the node from the path and mark it as unvisited
    path.pop()            
    visited.remove(start) ##
    return None
