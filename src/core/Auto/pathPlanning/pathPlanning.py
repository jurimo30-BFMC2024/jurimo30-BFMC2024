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
    
    def __init__(self, start):
        self.start = str(start)
        self.file_path = "Competition_track_graph.graphml" # change this to Competition_track_graph.graphml when in Romania
           
        self.roundabout_entries = ["317", "367", "397", "405"]
        self.roundabout_exits = ["368", "342", "398", "318"]
        self.hardInstructionQueue =  ["Right", "Straight", "Right", "Exit 1", "Exit 4", "Exit 3", "Right",
                                  "Right","Straight", "Straight", "Left", "Straight", "Straight","Straight",
                                  "Left","Left", "Left", "Right", "Straight", "Right", "Left", "Left",
                                  "Right", "Straight", "Exit 4", "Straight"]

        self.localizationPath = [("223", "243"), ("246","54"), ("55", "317"), ("368", "397"), ("343", "367"), ("318", "56"), ("49", "288"), ("302", "6"),
                                 ("1", "18"), ("13", "91"), ("88", "102"), ("97", "75"), ("70", "185"), ("188", "191"), ("193", "198"), ("201", "42"), ("39", "206"),
                                 ("208", "71"), ("74", "98"), ("99", "26"), ("31", "16"), ("13", "91"), ("92", "80"), ("83", "404"), ("399", "84"), ("79", "93")]
        
        # for arena
        # self.hardInstructionQueue = ["Left", "Right", "Right", "Left", "Straight", "Straight", "Right", "Straight", "Right", "Exit 1", "Exit 1"]
        
        # self.localizationPath = [("183", "93"), ("90", "14"), ("15", "32"), ("25", "100"), ("97", "75"), ("70", "185"), ("188", "191"), ("223", "243"), 
        #                          ("246","54"), ("55", "317"),  ("368", "397"), ("399", "400")]
        


    def planPath(self):
        '''Generates a queue of instructions'''
        graph = self.parse_graphml(self.file_path)
        instructionQueue = []
        segments = []
        # OVO ZAKOMENTARISATI KAD SE BUDE RADILA ARENA
        ###########################################################
        best_path = self.find_greedy_path(graph, self.start, "191")
        if best_path:
            turns = self.determine_turns(graph, best_path)
            for node, direction in turns:
                instructionQueue.append(direction)
            split_paths = self.split_path_by_intersections(graph, best_path)
            for segment_path in split_paths:
                segment = self.calculate_path_segments(graph, segment_path)
                if segment:
                    segments.append(segment)
        ###########################################################    
            # add hardcoded path
            instructionQueue.extend(self.hardInstructionQueue)

            for start, end in self.localizationPath:
                sub_path = self.find_greedy_path(graph, start, end)
                if sub_path:
                    segment = self.calculate_path_segments(graph, sub_path)
                    segments.append(segment)


        return instructionQueue, segments


    def parse_graphml(self, file_path):
        tree = ET.parse("src/core/Auto/pathPlanning/" + file_path)
        root = tree.getroot()
        
        ns = {'graphml': 'http://graphml.graphdrawing.org/xmlns'}
        graph = nx.DiGraph()
        
        
        for node in root.findall(".//graphml:node", ns):
            node_id = node.get("id")
            x = float(node.find(".//graphml:data[@key='d0']", ns).text)
            y = float(node.find(".//graphml:data[@key='d1']", ns).text)

            graph.add_node(node_id, pos=(x,y))
        
        for edge in root.findall(".//graphml:edge", ns):
            source = edge.get("source")
            target = edge.get("target")
            graph.add_edge(source, target)
        
        # intersection identification
        for node in graph.nodes():
            graph.nodes[node]['intersection'] = graph.out_degree(node) > 1  # 3 or more connections
        
        # special intersections
        graph.nodes["270"]['intersection'] = True
        graph.nodes["245"]['intersection'] = True

        # highway lane split nodes
        graph.nodes["401"]['intersection'] = False
        graph.nodes["423"]['intersection'] = False


        return graph

    def find_greedy_path(self, graph, start, goal):
        path = [start]
        current_node = start

        if start not in graph:
             raise ValueError(f"Start ({start}) not in graph.")
        if graph.nodes[goal]['intersection']:
            print("W: Your final point is inside an intersection (reconsider)")
        
        # p2p
        random_start_segment = nx.shortest_path(graph, source=current_node, target=goal, method='dijkstra')
        path.extend(random_start_segment[1:])

        
        return path
    
    def calculate_path_segments(self, graph, path):
        """
        Calculates metrics for a given path between two nodes.
        Assumes there are no intersections within the path.
        Returns a single segment with:
            - a list of nodes and their positions,
            - distances between consecutive nodes,
            - and the total length of the path.
        """
        if not path or len(path) < 2:
            return None  # Not enough nodes to form a segment

        waypoints = []
        segment_lengths = []

        for i in range(len(path)):
            node = path[i]
            waypoints.append({
                "idx": node,
                "pos": graph.nodes[node]['pos']
            })

            if i > 0:
                prev_pos = np.array(graph.nodes[path[i - 1]]['pos'])
                curr_pos = np.array(graph.nodes[node]['pos'])
                distance = np.linalg.norm(curr_pos - prev_pos)
                segment_lengths.append(distance)

        return {
            "nodes": waypoints,
            "distances": segment_lengths,
            "length": sum(segment_lengths)
        }
    
    
    def split_path_by_intersections(self, graph, path):
        """
        Splits a path into sub-paths at intersection nodes.
        Returns a list of sub-paths.
        """
        if not path or len(path) < 2:
            return []

        segments = []
        current_segment = [path[0]]

        for node in path[1:]:
            current_segment.append(node)
            if graph.nodes[node].get('intersection', False):
                if len(current_segment) > 1:
                    segments.append(current_segment[:-1])
                current_segment.clear() 

        # Add remaining segment if it's valid
        if len(current_segment) > 1:
            segments.append(current_segment)

        return segments

    def determine_turns(self, graph, path):
        directions = []
        i = 1
        while i < len(path) - 1:
            prev_node, current_node, next_node = path[i-1], path[i], path[i+1]

            if current_node in self.roundabout_entries:
                counter = 0
                remaining_path = path[i+1:]
                # counts how many nodes there are in between roundabout entry and exit
                for node in remaining_path:
                    if node in self.roundabout_exits:
                        break
                    counter += 1
                if counter == 2:
                    directions.append((current_node, "Exit 1"))
                elif counter == 4:
                    directions.append((current_node, "Exit 2"))
                elif counter == 6:
                    directions.append((current_node, "Exit 3"))
                elif counter == 8:
                    directions.append((current_node, "Exit 4"))
                else:
                    if i + counter + 1 > len(path) - 1:
                        print("W: Your end node is inside the roundabout (reconsider)")
                    else:
                        # theoretically this should never happen
                        raise ValueError("ERROR: ROUNDABOUT MACHINE BROKE")
                # skip over already accounted for nodes in roundabout
                i += counter + 1
                continue

            # skip if not an intersection
            if not graph.nodes[current_node].get('intersection', False):
                i += 1
                continue
            
            # get coordinates for vectors
            x_prev, y_prev = graph.nodes[prev_node]['pos']
            x_curr, y_curr = graph.nodes[current_node]['pos']
            x_next, y_next = graph.nodes[next_node]['pos']

            # create direction vectors
            incoming_vec = np.array([x_curr - x_prev, y_curr - y_prev])
            outgoing_vec = np.array([x_next - x_curr, y_next - y_curr])
            
            # normalize vectors
            with np.errstate(divide='ignore', invalid='ignore'):
                incoming_vec = incoming_vec / np.linalg.norm(incoming_vec)
                outgoing_vec = outgoing_vec / np.linalg.norm(outgoing_vec)
            
            # handle potential zero vectors
            if np.any(np.isnan(incoming_vec)) or np.any(np.isnan(outgoing_vec)):
                i += 1
                continue
            
            # calculate angle difference using arctan2
            angle_in = np.arctan2(incoming_vec[1], incoming_vec[0])
            angle_out = np.arctan2(outgoing_vec[1], outgoing_vec[0])
            angle_diff = np.degrees(angle_out - angle_in)
            
            # normalize angle to [-180, 180)
            angle_diff = (angle_diff + 180) % 360 - 180
            
            if angle_diff > 45:
                turn = "Left"
            elif angle_diff < -45:
                turn = "Right"
            else:
                turn = "Straight"
            
            if current_node == "210":
                if turn == "Right":
                    turn = "Straight"
                elif turn == "Straight":
                    turn = "Left"
            if current_node == "207":
                if turn == "Straight":
                    turn = "Left"
            if current_node == "192":
                if turn == "Straight":
                    turn = "Left"
                    
            directions.append((current_node, turn))
            i += 1
            
        return directions
    
if __name__ == "__main__":
    pathPlanner = PathPlanner(1)
    instructions, segments = pathPlanner.planPath()
    for segment in segments:
        print(segment)