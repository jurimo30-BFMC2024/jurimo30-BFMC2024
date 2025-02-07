import networkx as nx
import matplotlib.pyplot as plt
import socket
import threading

# Global variables
graph = None
pos = None
node_colors = None

# Load the GraphML file
def load_graphml(file_path):
    """Load a GraphML file and return the graph."""
    return nx.read_graphml(file_path)

# Plot the graph
def plot_graph():
    """Plot the graph using node positions from 'x' and 'y' attributes."""
    global graph, pos, node_colors

    # Draw the graph
    nx.draw(graph, pos, with_labels=True, node_size=300, node_color=node_colors, font_size=8, font_weight='bold')
    plt.title("Graph Visualization")
    plt.show()

# UDP listener function
def udp_listener():
    """Listen for UDP datagrams and update node colors dynamically."""
    global node_colors

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', 12345))  # Listen on port 12345

    print("UDP listener started. Waiting for node IDs...")

    while True:
        # Receive data
        data, addr = sock.recvfrom(1024)
        node_id = str(data.decode().strip())  # GraphML stores node IDs as strings

        # Update the node color if the node exists
        if node_id in graph.nodes:
            node_index = list(graph.nodes).index(node_id)  # Find index for color list
            node_colors[node_index] = 'red'

            # Redraw the graph
            plt.clf()  # Clear the figure
            nx.draw(graph, pos, with_labels=True, node_size=300, node_color=['lightblue']*len(graph.nodes), font_size=8, font_weight='bold')

            red_nodes = [node for i, node in enumerate(graph.nodes) if node_colors[i] == 'red']
            nx.draw_networkx_nodes(graph, pos, nodelist=red_nodes, node_size=300, node_color='red')


            plt.draw()
            plt.pause(0.1)
            node_colors[node_index] = 'lightblue'
        else:
            print(f"Node {node_id} not found in the graph.")

# Main function
def main():
    global graph, pos, node_colors

    # Path to your GraphML file
    graphml_file = "../src/core/Auto/pathPlanning/Small_map.graphml"  # Replace with your file path

    # Load the graph
    graph = load_graphml(graphml_file)

    # Extract node positions
    pos = {node: (data['x'], data['y']) for node, data in graph.nodes(data=True)}

    # Initialize node colors
    node_colors = ['lightblue'] * len(graph.nodes)

    # Start the UDP listener in a separate thread
    udp_thread = threading.Thread(target=udp_listener, daemon=True)
    udp_thread.start()

    # Plot the graph
    plot_graph()

if __name__ == "__main__":
    main()