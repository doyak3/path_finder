import matplotlib.pyplot as plt


# Define positions for each room for the specific layout
positions = {
    'R1': (0, 5),
    'R2': (1, 5),
    'R3': (2, 5),
    'R4': (3, 5),
    'R5': (4, 5),
    'R6': (5, 5),
    'R7': (3, 4),
    'R8': (3, 3),
    'R9': (3, 2),
    'R10': (2, 2),  # Rotate R10 to the left
    'R11': (1, 2)   # Rotate R11 to the left, next to R10
}

# Plot the building graph with the layout adjustments
def plot_building_layout(graph, positions, path=None):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Draw edges
    for room, neighbors in graph.items():
        for neighbor in neighbors:
            x_values = [positions[room][0], positions[neighbor][0]]
            y_values = [positions[room][1], positions[neighbor][1]]
            ax.plot(x_values, y_values, 'grey')

    # Draw nodes
    for room, (x, y) in positions.items():
        ax.plot(x, y, 'o', color="lightblue", markersize=10)
        ax.text(x, y + 0.1, room, ha='center', va='center', fontsize=9, fontweight='bold')

    # Highlight the shortest path if provided
    if path:
        for i in range(len(path) - 1):
            x_values = [positions[path[i]][0], positions[path[i + 1]][0]]
            y_values = [positions[path[i]][1], positions[path[i + 1]][1]]
            ax.plot(x_values, y_values, 'red', linewidth=2)
        # Highlight nodes on the shortest path
        for room in path:
            x, y = positions[room]
            ax.plot(x, y, 'o', color="orange", markersize=10)

    # Set plot limits and title
    ax.set_xlim(-1, 6)
    ax.set_ylim(0, 6)
    plt.title("Building Layout with Rooms and Path")
    plt.axis('off')
    plt.show()
