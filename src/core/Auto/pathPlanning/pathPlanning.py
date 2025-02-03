import networkx as nx
import xml.etree.ElementTree as ET
import numpy as np

class PathPlanner:
    """
    Plans a path on a given .graphml file.

    Attributes:
                starting node (int)
                goal node (int)
                mode (string): Can either be \"p2p\" for point to point or \"pacman\" for collecting all
                                important nodes between the starting and ending node
    Example:
            pp = PathPlanner(1, 10, "p2p")
            
            print(pp.planPath())
    """
    
    def __init__(self, start, goal, mode):
        self.start = str(start)
        self.goal = str(goal)
        if mode not in {"p2p", "pacman"}:
            raise ValueError("PathPlanner: mode must be either \"p2p\" or \"pacman\"")
        else:
            self.mode = mode
        self.file_path = "Small_map.graphml" # change this to Competition_track_graph.graphml when in Romania
        

    def planPath(self):
        '''Generates a queue of instructions'''
        graph = self.parse_graphml(self.file_path, self.mode)
        best_path = self.find_greedy_path(graph, self.start, self.goal)
        if best_path:
            instructionQueue = []
            # print("Greedy Path Collecting All Collectibles:", best_path)
            turns = self.determine_turns(graph, best_path)
            # print("Turn Instructions:")
            for node, direction in turns:
                # print(f"Kod node-a {node}, idi {direction}")
                instructionQueue.append(direction)

        return instructionQueue


    def parse_graphml(self, file_path, mode):
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        ns = {'graphml': 'http://graphml.graphdrawing.org/xmlns'}
        graph = nx.DiGraph()
        
        collectibles = {"75", "128", "116", "98", "110", "185", "71", "25", "31", "29", "93", "80", "82", "136",
        "419", "125", "403", "399", "343", "386", "363", "368", "318", "317", "56", "54", "261", "239", "228",
        "225", "198", "42", "289", "6", "8"}
        
        for node in root.findall(".//graphml:node", ns):
            node_id = node.get("id")
            x = float(node.find(".//graphml:data[@key='d0']", ns).text)
            y = float(node.find(".//graphml:data[@key='d1']", ns).text)
            if mode == "p2p":
                graph.add_node(node_id, pos=(x,y))
            else:
            #for pacman use the line under this comment instead
                graph.add_node(node_id, pos=(x, y), collectible=(node_id in collectibles))
        
        for edge in root.findall(".//graphml:edge", ns):
            source = edge.get("source")
            target = edge.get("target")
            graph.add_edge(source, target)
        
        # Improved intersection identification
        for node in graph.nodes():
            graph.nodes[node]['intersection'] = graph.out_degree(node) > 1  # 3 or more connections
        
        return graph

    def find_greedy_path(self, graph, start, goal):
        collectibles = {n for n in graph.nodes if graph.nodes[n].get('collectible', False)}
        visited_collectibles = set()
        path = [start]
        current_node = start
        
        # this part is skipped if p2p
        while visited_collectibles != collectibles:
            nearest = min(
                collectibles - visited_collectibles,
                key=lambda n: nx.shortest_path_length(graph, source=current_node, target=n, method='dijkstra')
            )
            segment = nx.shortest_path(graph, source=current_node, target=nearest, method='dijkstra')
            
            path.extend(segment[1:])
            
            visited_collectibles.add(nearest)
            current_node = nearest
        
        # Finally, go to the goal (this part is skipped if pacman)
        if self.mode == "p2p":
            final_segment = nx.shortest_path(graph, source=current_node, target=goal, method='dijkstra')
            path.extend(final_segment[1:])
        
        return path

    def determine_turns(self, graph, path):
        directions = []
        for i in range(1, len(path) - 1):
            prev_node, current_node, next_node = path[i-1], path[i], path[i+1]
            
            # Skip if not an intersection
            if not graph.nodes[current_node].get('intersection', False):
                continue
            
            # Get coordinates for vectors
            x_prev, y_prev = graph.nodes[prev_node]['pos']
            x_curr, y_curr = graph.nodes[current_node]['pos']
            x_next, y_next = graph.nodes[next_node]['pos']

            # Create direction vectors
            incoming_vec = np.array([x_curr - x_prev, y_curr - y_prev])
            outgoing_vec = np.array([x_next - x_curr, y_next - y_curr])
            
            # Normalize vectors
            with np.errstate(divide='ignore', invalid='ignore'):
                incoming_vec = incoming_vec / np.linalg.norm(incoming_vec)
                outgoing_vec = outgoing_vec / np.linalg.norm(outgoing_vec)
            
            # Handle potential zero vectors
            if np.any(np.isnan(incoming_vec)) or np.any(np.isnan(outgoing_vec)):
                continue
            
            # Calculate angle difference using arctan2
            angle_in = np.arctan2(incoming_vec[1], incoming_vec[0])
            angle_out = np.arctan2(outgoing_vec[1], outgoing_vec[0])
            angle_diff = np.degrees(angle_out - angle_in)
            
            # Normalize angle to [-180, 180)
            angle_diff = (angle_diff + 180) % 360 - 180
            
            # Debugging: Print intermediate values
            # print(f"At node {current_node}:")
            # print(f"  Incoming vector: {incoming_vec}")
            # print(f"  Outgoing vector: {outgoing_vec}")
            # print(f"  Angle difference: {angle_diff:.2f}°")
            
            # Determine turn direction with clear thresholds
            if angle_diff > 45:
                turn = "levo"
            elif angle_diff < -45:
                turn = "desno"
            else:
                turn = "pravo"
            
            directions.append((current_node, turn))
        
        return directions

