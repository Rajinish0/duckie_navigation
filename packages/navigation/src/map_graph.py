# #!/usr/bin/env python3
# """
# Topological map for Duckiebot navigation: a small graph of intersections
# connected by directed road segments. Lane-following handles everything
# *between* nodes; this module only decides what to do *at* each node.
# """
# from __future__ import annotations
# import heapq
# import math
# import yaml
# from dataclasses import dataclass, field
# from enum import Enum
# from typing import Dict, List, Optional, Tuple

# class Direction(Enum):
#     """Compass heading, aligned with the map's grid axes."""
#     N = 0
#     E = 1
#     S = 2
#     W = 3

#     def turn_to(self, other: "Direction") -> str:
#         """Relative turn needed to go from heading `self` to heading `other`."""
#         delta = (other.value - self.value) % 4
#         return {0: "straight", 1: "right", 3: "left", 2: "uturn"}[delta]

#     @staticmethod
#     def from_angle(theta: float) -> "Direction":
#         """Quantize a continuous heading (radians, standard math convention,
#         0 = +x/East) to the nearest grid-aligned compass direction."""
#         idx = round(math.degrees(theta % (2 * math.pi)) / 90) % 4
#         return [Direction.E, Direction.N, Direction.W, Direction.S][idx]


# # _OPPOSITE = {}  # filled below to avoid forward-reference issues
# _OPPOSITE =  {Direction.N: Direction.S, Direction.S: Direction.N,
#               Direction.E: Direction.W, Direction.W: Direction.E}


# @dataclass
# class Edge:
#     to_node: str
#     direction: Direction   # compass direction of travel along this edge
#     distance: float        # meters


# @dataclass
# class Node:
#     name: str
#     x: float                # map-frame coords (meters) - goal snapping / debug only
#     y: float
#     is_phantom : bool
#     # tag_id: Optional[int] = None   # AprilTag id seen at this intersection
#     edges: List[Edge] = field(default_factory=list)


# def node_dist(node1 : Node, node2 : Node, squared = True):
#     dx = node1.x - node2.x
#     dy = node1.y - node2.y
#     dist_sqrd = dx * dx + dy * dy
#     if squared:
#         return dist_sqrd
#     return math.sqrt(dist_sqrd)

# class RoadGraph:
#     def __init__(self):
#         self.nodes: Dict[str, Node] = {}
#         self._splice_history = dict()

#     def add_node(self, name: str, x: float, y: float, is_phantom: bool = False):
#         self.nodes[name] = Node(name=name, x=x, y=y, is_phantom=is_phantom)

#     def add_node_direct(self, node : Node):
#         self.nodes[node.name] = node

#     def add_edge(self, a: str, b: str, direction: Direction, distance: float,
#                  bidirectional: bool = True):
#         self.nodes[a].edges.append(Edge(to_node=b, direction=direction, distance=distance))
#         if bidirectional:
#             opposite = _OPPOSITE[direction]
#             self.nodes[b].edges.append(Edge(to_node=a, direction=opposite, distance=distance))

#     def shortest_path(self, start: str, goal: str) -> Tuple[List[str], List[Edge]]:
#         """Dijkstra. Returns (node_sequence, edge_sequence)."""
#         if start not in self.nodes or goal not in self.nodes:
#             raise KeyError(f"unknown node in path request: {start} -> {goal}")

#         dist = {start: 0.0}
#         prev: Dict[str, Tuple[str, Edge]] = {}
#         visited = set()
#         pq = [(0.0, start)]

#         while pq:
#             d, u = heapq.heappop(pq)
#             if u in visited:
#                 continue
#             visited.add(u)
#             if u == goal:
#                 break
#             for e in self.nodes[u].edges:
#                 nd = d + e.distance
#                 if e.to_node not in dist or nd < dist[e.to_node]:
#                     dist[e.to_node] = nd
#                     prev[e.to_node] = (u, e)
#                     heapq.heappush(pq, (nd, e.to_node))

#         if goal != start and goal not in prev:
#             raise ValueError(f"no path found from {start} to {goal}")

#         nodes_seq = [goal]
#         edges_seq: List[Edge] = []
#         cur = goal
#         while cur != start:
#             p, e = prev[cur]
#             nodes_seq.append(p)
#             edges_seq.append(e)
#             cur = p
#         nodes_seq.reverse()
#         edges_seq.reverse()
#         return nodes_seq, edges_seq
    
#     # def add_node_with_splice(self, name : str, x : float, y : float):
#     #     a, b, d = self.project_point(x, y)
#     #     try:
#     #         return self.insert_point_on_edge(a, b, d, name)
#     #     except:
#     #         return a

#     def add_node_with_splice(self, name: str, x: float, y: float) -> str:
#         a, b, d = self.project_point(x, y)
        
#         # Find the total length of the edge we are snapping to
#         edge_ab = next(e for e in self.nodes[a].edges if e.to_node == b)
#         total = edge_ab.distance
        
#         # Clamp the distance to be at least 1 millimeter away from either intersection.
#         # This bypasses the ValueError and ensures a safe, temporary node is ALWAYS generated.
#         epsilon = 0.001 
#         d_clamped = max(epsilon, min(total - epsilon, d))
        
#         self.insert_point_on_edge(a, b, d_clamped, name)
        
#         return name

#     def path_to_turns(self, edges_seq: List[Edge], start_heading: Direction) -> List[str]:
#         """One relative turn per edge: turns[i] is the decision made when
#         departing nodes_seq[i] onto edges_seq[i]. start_heading is the
#         direction the robot will be traveling when it reaches nodes_seq[0]
#         (the first intersection it hasn't decided at yet)."""
#         turns = []
#         heading = start_heading
#         for e in edges_seq:
#             turns.append(heading.turn_to(e.direction))
#             heading = e.direction
#         return turns

#     def project_point(self, x: float, y: float) -> Tuple[str, str, float]:
#         """Closest point on any edge to (x, y). Returns (node_a, node_b,
#         distance_from_a_to_projection)."""
#         best = None
#         seen = set()
#         for a_name, a in self.nodes.items():
#             for e in a.edges:
#                 key = tuple(sorted((a_name, e.to_node)))
#                 if key in seen:
#                     continue
#                 seen.add(key)
#                 b = self.nodes[e.to_node]
#                 dx, dy = b.x - a.x, b.y - a.y
#                 seg_len2 = dx * dx + dy * dy
#                 t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0,
#                     ((x - a.x) * dx + (y - a.y) * dy) / seg_len2))
#                 px, py = a.x + t * dx, a.y + t * dy
#                 d = math.hypot(x - px, y - py)
#                 if best is None or d < best[0]:
#                     best = (d, a_name, e.to_node, t * e.distance)
#         _, a_name, b_name, dist_from_a = best
#         return a_name, b_name, dist_from_a

#     def insert_point_on_edge(self, a: str, b: str, distance_from_a: float,
#                               new_name: str = "GOAL") -> str:
#         """Splice a virtual node into edge a->b so a goal can be anywhere
#         along a road segment, not just at an intersection."""
#         edge_ab = next(e for e in self.nodes[a].edges if e.to_node == b)
#         edge_ba = next((e for e in self.nodes[b].edges if e.to_node == a), None)
#         total = edge_ab.distance
#         if not (0 < distance_from_a < total):
#             raise ValueError("distance_from_a must be strictly between 0 and edge length")
        
#         self._splice_history[new_name] = {
#             'a': a,
#             'b': b,
#             'edge_ab': edge_ab,
#             'edge_ba': edge_ba
#         }

#         gx = self.nodes[a].x + (self.nodes[b].x - self.nodes[a].x) * (distance_from_a / total)
#         gy = self.nodes[a].y + (self.nodes[b].y - self.nodes[a].y) * (distance_from_a / total)
#         self.add_node(new_name, gx, gy)

#         self.nodes[a].edges.remove(edge_ab)
#         self.add_edge(a, new_name, edge_ab.direction, distance_from_a, bidirectional=False)
#         self.add_edge(new_name, b, edge_ab.direction, total - distance_from_a, bidirectional=False)

#         if edge_ba is not None:
#             self.nodes[b].edges.remove(edge_ba)
#             self.add_edge(b, new_name, edge_ba.direction, total - distance_from_a, bidirectional=False)
#             self.add_edge(new_name, a, edge_ba.direction, distance_from_a, bidirectional=False)

#         return new_name


#     def restore_graph(self, temp_node_names: List[str]):
#         """Removes temporary spliced nodes and restores original edges."""
#         for name in temp_node_names:
#             if name not in self._splice_history:
#                 continue
            
#             history = self._splice_history.pop(name)
#             a, b = history['a'], history['b']
#             orig_ab, orig_ba = history['edge_ab'], history['edge_ba']
            
#             # 1. Remove the temporary mini-edges pointing to the spliced node
#             self.nodes[a].edges = [e for e in self.nodes[a].edges if e.to_node != name]
#             if orig_ba is not None:
#                 self.nodes[b].edges = [e for e in self.nodes[b].edges if e.to_node != name]
            
#             # 2. Put the original full edges back
#             self.nodes[a].edges.append(orig_ab)
#             if orig_ba is not None:
#                 self.nodes[b].edges.append(orig_ba)
            
#             # 3. Delete the temporary node itself
#             if name in self.nodes:
#                 del self.nodes[name]


# def load_graph_from_yaml(path: str) -> RoadGraph:
#     with open(path) as f:
#         data = yaml.safe_load(f)
#     g = RoadGraph()
#     for n in data["nodes"]:
#         g.add_node(n["name"], n["x"], n["y"], is_phantom=n.get("is_phantom", False))
#     for e in data["edges"]:
#         g.add_edge(e["from"], e["to"], Direction[e["direction"]], e["distance"],
#                    bidirectional=e.get("bidirectional", True))
#     return g

#!/usr/bin/env python3
"""
Topological map for Duckiebot navigation: a small graph of intersections
connected by directed road segments. Lane-following handles everything
*between* nodes; this module only decides what to do *at* each node.
"""
from __future__ import annotations
import heapq
import math
import yaml
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple

class Direction(Enum):
    """Compass heading, aligned with the map's grid axes."""
    N = 0
    E = 1
    S = 2
    W = 3

    def turn_to(self, other: "Direction") -> str:
        """Relative turn needed to go from heading `self` to heading `other`."""
        delta = (other.value - self.value) % 4
        return {0: "straight", 1: "right", 3: "left", 2: "uturn"}[delta]

    @staticmethod
    def from_angle(theta: float) -> "Direction":
        """Quantize a continuous heading (radians, standard math convention,
        0 = +x/East) to the nearest grid-aligned compass direction."""
        idx = round(math.degrees(theta % (2 * math.pi)) / 90) % 4
        return [Direction.E, Direction.N, Direction.W, Direction.S][idx]
    
    @staticmethod
    def to_angle(d : "Direction") -> float:
        if d == Direction.N: return -math.pi/2
        if d == Direction.E: return math.pi
        if d == Direction.S: return math.pi/2
        return 0


# _OPPOSITE = {}  # filled below to avoid forward-reference issues
_OPPOSITE =  {Direction.N: Direction.S, Direction.S: Direction.N,
              Direction.E: Direction.W, Direction.W: Direction.E}


@dataclass
class Edge:
    to_node: str
    direction: Direction   # compass direction of travel along this edge
    distance: float        # meters


@dataclass
class Node:
    name: str
    x: float                # map-frame coords (meters) - goal snapping / debug only
    y: float
    is_phantom : bool
    # tag_id: Optional[int] = None   # AprilTag id seen at this intersection
    edges: List[Edge] = field(default_factory=list)


def node_dist(node1 : Node, node2 : Node, squared = True):
    dx = node1.x - node2.x
    dy = node1.y - node2.y
    dist_sqrd = dx * dx + dy * dy
    if squared:
        return dist_sqrd
    return math.sqrt(dist_sqrd)

class RoadGraph:
    def __init__(self):
        self.nodes: Dict[str, Node] = {}
        self._splice_history = dict()

    def add_node(self, name: str, x: float, y: float, is_phantom: bool = False):
        self.nodes[name] = Node(name=name, x=x, y=y, is_phantom=is_phantom)

    def add_node_direct(self, node : Node):
        self.nodes[node.name] = node

    def add_edge(self, a: str, b: str, direction: Direction, distance: float,
                 bidirectional: bool = True):
        self.nodes[a].edges.append(Edge(to_node=b, direction=direction, distance=distance))
        if bidirectional:
            opposite = _OPPOSITE[direction]
            self.nodes[b].edges.append(Edge(to_node=a, direction=opposite, distance=distance))

    def shortest_path(self, start: str, goal: str) -> Tuple[List[str], List[Edge]]:
        """Dijkstra. Returns (node_sequence, edge_sequence)."""
        if start not in self.nodes or goal not in self.nodes:
            raise KeyError(f"unknown node in path request: {start} -> {goal}")

        dist = {start: 0.0}
        prev: Dict[str, Tuple[str, Edge]] = {}
        visited = set()
        pq = [(0.0, start)]

        while pq:
            d, u = heapq.heappop(pq)
            if u in visited:
                continue
            visited.add(u)
            if u == goal:
                break
            for e in self.nodes[u].edges:
                nd = d + e.distance
                if e.to_node not in dist or nd < dist[e.to_node]:
                    dist[e.to_node] = nd
                    prev[e.to_node] = (u, e)
                    heapq.heappush(pq, (nd, e.to_node))

        if goal != start and goal not in prev:
            raise ValueError(f"no path found from {start} to {goal}")

        nodes_seq = [goal]
        edges_seq: List[Edge] = []
        cur = goal
        while cur != start:
            p, e = prev[cur]
            nodes_seq.append(p)
            edges_seq.append(e)
            cur = p
        nodes_seq.reverse()
        edges_seq.reverse()
        return nodes_seq, edges_seq
    
    def add_node_with_splice(self, name : str, x : float, y : float):
        a, b, d = self.project_point(x, y)
        return self.insert_point_on_edge(a, b, d, name)
        # except:
        #     return a

    # def add_node_with_splice(self, name: str, x: float, y: float) -> str:
    #     a, b, d = self.project_point(x, y)
        
    #     # Find the total length of the edge we are snapping to
    #     edge_ab = next(e for e in self.nodes[a].edges if e.to_node == b)
    #     total = edge_ab.distance
        
    #     # Clamp the distance to be at least 1 millimeter away from either intersection.
    #     # This bypasses the ValueError and ensures a safe, temporary node is ALWAYS generated.
    #     epsilon = 0.001 
    #     d_clamped = max(epsilon, min(total - epsilon, d))
        
    #     self.insert_point_on_edge(a, b, d_clamped, name)
        
    #     return name

    # def path_to_turns(self, edges_seq: List[Edge], start_heading: Direction) -> List[str]:
    #     """One relative turn per edge: turns[i] is the decision made when
    #     departing nodes_seq[i] onto edges_seq[i]. start_heading is the
    #     direction the robot will be traveling when it reaches nodes_seq[0]
    #     (the first intersection it hasn't decided at yet)."""
    #     turns = []
    #     heading = start_heading
    #     for e in edges_seq:
    #         turns.append(heading.turn_to(e.direction))
    #         heading = e.direction
    #     return turns

    def path_to_turns(self, edges_seq: List[Edge], nodes_seq: List[str],
                       start_heading: Optional[Direction] = None) -> List[str]:
        """One relative turn per REAL intersection departure. Phantom nodes
        (START/GOAL splices) never produce a turn: the robot isn't choosing
        a direction there, it's already committed to that edge just by being
        spliced onto it. start_heading is only needed if nodes_seq[0]
        happens to be a real, non-phantom node."""
        turns = []
        heading = start_heading
        for i, e in enumerate(edges_seq):
            node = self.nodes[nodes_seq[i]]
            # if node.is_phantom: continue 
            if not node.is_phantom:
                if heading is None:
                    raise ValueError(
                        f"path starts at real node '{node.name}' but no start_heading was given")
            # print(node.name, e.direction, heading, heading.turn_to(e.direction))
                turns.append(heading.turn_to(e.direction))
            elif (heading.turn_to(e.direction) == 'uturn'):
                turns.append('uturn')
            heading = e.direction
        return turns

    def project_point(self, x: float, y: float) -> Tuple[str, str, float]:
        """Closest point on any edge to (x, y). Returns (node_a, node_b,
        distance_from_a_to_projection)."""
        best = None
        seen = set()
        for a_name, a in self.nodes.items():
            for e in a.edges:
                key = tuple(sorted((a_name, e.to_node)))
                if key in seen:
                    continue
                seen.add(key)
                b = self.nodes[e.to_node]
                dx, dy = b.x - a.x, b.y - a.y
                seg_len2 = dx * dx + dy * dy
                t = 0.0 if seg_len2 == 0 else max(0.0, min(1.0,
                    ((x - a.x) * dx + (y - a.y) * dy) / seg_len2))
                px, py = a.x + t * dx, a.y + t * dy
                d = math.hypot(x - px, y - py)
                if best is None or d < best[0]:
                    best = (d, a_name, e.to_node, t * e.distance)
        _, a_name, b_name, dist_from_a = best
        return a_name, b_name, dist_from_a

    def insert_point_on_edge(self, a: str, b: str, distance_from_a: float,
                              new_name: str = "GOAL") -> str:
        """Splice a virtual node into edge a->b so a goal can be anywhere
        along a road segment, not just at an intersection."""
        edge_ab = next(e for e in self.nodes[a].edges if e.to_node == b)
        edge_ba = next((e for e in self.nodes[b].edges if e.to_node == a), None)
        total = edge_ab.distance
        # print(total)
        if not (0 < distance_from_a < total):
            if (distance_from_a <= 0): return a
            else: return b
            # raise ValueError("distance_from_a must be strictly between 0 and edge length")
        
        self._splice_history[new_name] = {
            'a': a,
            'b': b,
            'edge_ab': edge_ab,
            'edge_ba': edge_ba
        }

        gx = self.nodes[a].x + (self.nodes[b].x - self.nodes[a].x) * (distance_from_a / total)
        gy = self.nodes[a].y + (self.nodes[b].y - self.nodes[a].y) * (distance_from_a / total)
        self.add_node(new_name, gx, gy, is_phantom=True)

        self.nodes[a].edges.remove(edge_ab)
        self.add_edge(a, new_name, edge_ab.direction, distance_from_a, bidirectional=False)
        self.add_edge(new_name, b, edge_ab.direction, total - distance_from_a, bidirectional=False)

        if edge_ba is not None:
            self.nodes[b].edges.remove(edge_ba)
            self.add_edge(b, new_name, edge_ba.direction, total - distance_from_a, bidirectional=False)
            self.add_edge(new_name, a, edge_ba.direction, distance_from_a, bidirectional=False)

        return new_name


    def restore_graph(self, temp_node_names: List[str]):
        """Removes temporary spliced nodes and restores original edges."""
        for name in temp_node_names:
            if name not in self._splice_history:
                continue
            
            history = self._splice_history.pop(name)
            a, b = history['a'], history['b']
            orig_ab, orig_ba = history['edge_ab'], history['edge_ba']
            
            # 1. Remove the temporary mini-edges pointing to the spliced node
            self.nodes[a].edges = [e for e in self.nodes[a].edges if e.to_node != name]
            if orig_ba is not None:
                self.nodes[b].edges = [e for e in self.nodes[b].edges if e.to_node != name]
            
            # 2. Put the original full edges back
            self.nodes[a].edges.append(orig_ab)
            if orig_ba is not None:
                self.nodes[b].edges.append(orig_ba)
            
            # 3. Delete the temporary node itself
            if name in self.nodes:
                del self.nodes[name]


def load_graph_from_yaml(path: str) -> RoadGraph:
    with open(path) as f:
        data = yaml.safe_load(f)
    g = RoadGraph()
    for n in data["nodes"]:
        g.add_node(n["name"], n["x"], n["y"], is_phantom=n.get("is_phantom", False))
    for e in data["edges"]:
        g.add_edge(e["from"], e["to"], Direction[e["direction"]], e["distance"],
                   bidirectional=e.get("bidirectional", True))
    return g