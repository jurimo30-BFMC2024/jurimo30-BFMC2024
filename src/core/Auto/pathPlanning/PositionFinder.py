import xml.etree.ElementTree as ET
import networkx as nx
import numpy as np
import math
import matplotlib.pyplot as plt

class PositionFinder:
    def __init__(self, file_path):
        self.graph = self._parse_graphml(file_path)

    def _parse_graphml(self, file_path):
        tree = ET.parse("src/core/Auto/pathPlanning/" + file_path)
        root = tree.getroot()

        ns = {'graphml': 'http://graphml.graphdrawing.org/xmlns'}
        graph = nx.DiGraph()
        scaling_factor = 100

        for node in root.findall(".//graphml:node", ns):
            node_id = node.get("id")
            x = float(node.find(".//graphml:data[@key='d0']", ns).text) * scaling_factor
            y = float(node.find(".//graphml:data[@key='d1']", ns).text) * scaling_factor
            graph.add_node(node_id, pos=(x, y))

        for edge in root.findall(".//graphml:edge", ns):
            source = edge.get("source")
            target = edge.get("target")
            graph.add_edge(source, target)

        return graph

    def find_best_node(self, x, y, rotation_deg=None):
        """
        Find the best node based on position and optional rotation.
        """
        input_vec = np.array([x, y])
        min_score = float('inf')
        best_node = None

        for node_id, data in self.graph.nodes(data=True):
            node_pos = np.array(data['pos'])
            score = self._calculate_score(node_pos, input_vec, node_id, rotation_deg)

            if score < min_score:
                min_score = score
                best_node = node_id

        length, angle = self._calculate_vector_and_angle(best_node, input_vec, rotation_deg)
        length = np.cos(math.radians(angle)) * length  # Project length onto the direction of the angle

        return best_node, length

    def _calculate_score(self, node_pos, input_vec, node_id, rotation_deg):
        """
        Calculate the score for a node based on distance and rotation.
        """
        dist = np.linalg.norm(node_pos - input_vec)
        score = dist

        if rotation_deg is not None:
            best_heading_diff = self._calculate_heading_diff(node_pos, node_id, rotation_deg)
            score += math.degrees(best_heading_diff)

        return score

    def _calculate_heading_diff(self, node_pos, node_id, rotation_deg):
        """
        Calculate the best heading difference for a node based on its outgoing edges.
        """
        rot_rad = math.radians(rotation_deg)
        heading = np.array([math.cos(rot_rad), math.sin(rot_rad)])
        best_heading_diff = float('inf')

        for succ in self.graph.successors(node_id):
            succ_pos = np.array(self.graph.nodes[succ]['pos'])
            edge_vec = succ_pos - node_pos
            edge_vec_norm = edge_vec / (np.linalg.norm(edge_vec) + 1e-6)
            angle_diff = np.arccos(np.clip(np.dot(heading, edge_vec_norm), -1.0, 1.0))
            best_heading_diff = min(best_heading_diff, angle_diff)

        return best_heading_diff

    def _calculate_vector_and_angle(self, best_node, input_vec, rotation_deg):
        """
        Calculate the vector length and relative angle to the best node.
        """
        best_node_pos = np.array(self.graph.nodes[best_node]['pos'])
        vector = best_node_pos - input_vec
        length = np.linalg.norm(vector)
        angle = math.degrees(math.atan2(vector[1], vector[0]))

        if rotation_deg is not None:
            angle = (angle - rotation_deg) % 360
            if angle > 180:
                angle -= 360

        return length, angle

if __name__ == "__main__":
    pf = PositionFinder("Small_map_roundabout.graphml")

    # Plot the graph
    pos = nx.get_node_attributes(pf.graph, 'pos')
    fig, ax = plt.subplots()
    nx.draw(pf.graph, pos, ax=ax, with_labels=True, node_size=300, node_color="lightblue")
    best_node_marker, = ax.plot([], [], 'ro', markersize=10)  # Marker for the best node

    last_mouse_pos = [None]  # Store the last mouse position in a mutable list

    def on_mouse_move(event, last_mouse_pos):
        if event.xdata is None or event.ydata is None:
            return  # Ignore events outside the plot area

        x, y = event.xdata, event.ydata

        # Calculate rotation_deg based on the last mouse position
        rotation_deg = None
        if last_mouse_pos[0] is not None:
            dx = x - last_mouse_pos[0][0]
            dy = y - last_mouse_pos[0][1]
            rotation_deg = math.degrees(math.atan2(dy, dx))

        last_mouse_pos[0] = (x, y)  # Update the last mouse position
        
        best_node, _ = pf.find_best_node(x, y, rotation_deg)
        best_node_pos = pos[best_node]

        # Update the marker for the best node
        best_node_marker.set_data([best_node_pos[0]], [best_node_pos[1]])
        fig.canvas.draw_idle()

    # Connect the mouse move event to the handler
    fig.canvas.mpl_connect('motion_notify_event', lambda event: on_mouse_move(event, last_mouse_pos))

    plt.title("Move the mouse to find the best node")
    plt.show()