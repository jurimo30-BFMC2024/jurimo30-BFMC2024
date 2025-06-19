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
        self.file_path = "Small_map_roundabout.graphml" # change this to Competition_track_graph.graphml when in Romania
        if self.file_path == "Small_map_roundabout.graphml":
            self.roundabout_entries = ["14", "49", "22"]
            self.roundabout_exits = ["24", "34", "40", "48"]
        else:   
            self.roundabout_entries = ["317", "367", "397", "405"]
            self.roundabout_exits = ["368", "342", "398", "318"]

    def planPath(self):
        '''Generates a queue of instructions'''
        graph = self.parse_graphml(self.file_path, self.mode)
        best_path = self.find_greedy_path(graph, self.start, self.goal)
        if best_path:
            instructionQueue = []
            turns = self.determine_turns(graph, best_path)
            for node, direction in turns:
                instructionQueue.append(direction)

        return instructionQueue


    def parse_graphml(self, file_path, mode):
        tree = ET.parse(file_path)
        root = tree.getroot()
        
        ns = {'graphml': 'http://graphml.graphdrawing.org/xmlns'}
        graph = nx.DiGraph()
        
        if self.file_path == "Competition_track_graph.graphml":
            collectibles = {"75", "128", "116", "98", "110", "185", "71", "25", "31", "29", "93", "80", "82", "136",
                "419", "125", "403", "399", "343", "386", "363", "368", "318", "317", "56", "54", "261", "239", "228",
                "225", "198", "42", "289", "6", "8"}
            self.random_start_nodes = {"1", "7", "8", "9", "10", "11", "12", "13", "18", "21",
                "22", "23", "24", "25", "26", "27", "28", "29", "30", "31",
                "32", "33", "34", "35", "36", "37", "38", "39", "40", "41",
                "42", "43", "44", "45", "46", "47", "48", "49", "50", "51",
                "52", "53", "54", "55", "56", "57", "58", "59", "60", "61",
                "62", "63", "64", "65", "66", "67", "68", "69", "70", "71",
                "72", "73", "74", "75", "76", "77", "78", "79", "80", "81",
                "82", "83", "84", "85", "86", "87", "88", "89", "90", "91",
                "92", "93", "94", "95", "96", "97", "98", "99", "100", "101",
                "102", "103", "104"}
        else:
            collectibles = {"32", "22", "14", "38", "7"}
        
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
        
        # intersection identification
        for node in graph.nodes():
            graph.nodes[node]['intersection'] = graph.out_degree(node) > 1  # 3 or more connections
        
        if self.file_path == "Competition_track_graph.graphml":
            graph.nodes["270"]['intersection'] = True
            graph.nodes["245"]['intersection'] = True

            # highway lane split nodes
            graph.nodes["401"]['intersection'] = False
            graph.nodes["423"]['intersection'] = False

            # random start area nodes
            for node in graph.nodes():
                graph.nodes[node]['start_area'] = node in self.random_start_nodes

        else:
            graph.nodes["39"]['intersection'] = True
            graph.nodes["33"]['intersection'] = True


        return graph

    def find_greedy_path(self, graph, start, goal):
        collectibles = {n for n in graph.nodes if graph.nodes[n].get('collectible', False)}
        visited = set()
        path = [start]
        current = start

        # exit start area first if needed
        if self.mode == "pacman" and graph.nodes[start].get('start_area', False):
            valid_collectibles = collectibles - self.random_start_nodes
            nearest = min(valid_collectibles,
                        key=lambda n: nx.shortest_path_length(graph, source=current, target=n))
            path.extend(nx.shortest_path(graph, source=current, target=nearest)[1:])
            current = nearest

        # lookahead collection
        while len(visited) < len(collectibles):
            unvisited = collectibles - visited
            
            if len(unvisited) <= 3:  # Last few nodes - simple greedy
                nearest = min(unvisited, 
                            key=lambda n: nx.shortest_path_length(graph, source=current, target=n))
            else:
                # get top 3 closest candidates
                candidates = sorted(unvisited,
                                key=lambda n: nx.shortest_path_length(graph, source=current, target=n))[:3]
                
                # precompute paths to candidates
                candidate_paths = {
                    n: nx.shortest_path_length(graph, source=current, target=n)
                    for n in candidates
                }
                
                # evaluate which candidate leads to best two-step cost
                nearest = None
                min_total_cost = float('inf')
                
                for candidate in candidates:
                    # cost to reach this candidate
                    cost = candidate_paths[candidate]
                    
                    # cost from candidate to nearest remaining node
                    remaining = [n for n in unvisited if n != candidate]
                    if remaining:
                        next_cost = min(
                            nx.shortest_path_length(graph, source=candidate, target=m)
                            for m in remaining
                        )
                        total_cost = cost + next_cost
                        
                        if total_cost < min_total_cost:
                            min_total_cost = total_cost
                            nearest = candidate
                
            # add the best segment to path
            segment = nx.shortest_path(graph, source=current, target=nearest)
            path.extend(segment[1:])
            visited.add(nearest)
            current = nearest

        # final path to goal
        path.extend(nx.shortest_path(graph, source=current, target=goal)[1:])
        return path

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
            
            directions.append((current_node, turn))
            i += 1
        return directions