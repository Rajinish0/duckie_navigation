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
import config

# NODE_SNAP_EPSILON = 0.05  # meters - "practically at an intersection" counts as being at it
NODE_SNAP_EPSILON = 0.0

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
        self.edge_coords: Dict[tuple[str, str], tuple[float, float]] = {}
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
    
    # def add_edge_coords(self, a : str, b : str, x : float, y : float):
    #     self.edge_coords[(a, b)] = (x, y)

    def add_edge_coords(self, a: str, direction: Direction, x: float, y: float):
        self.edge_coords[(a, direction)] = (x, y)

    # def shortest_path(self, start: str, goal: str) -> Tuple[List[str], List[Edge]]:
    #     """Dijkstra. Returns (node_sequence, edge_sequence)."""
    #     if start not in self.nodes or goal not in self.nodes:
    #         raise KeyError(f"unknown node in path request: {start} -> {goal}")

    #     dist = {start: 0.0}
    #     prev: Dict[str, Tuple[str, Edge]] = {}
    #     visited = set()
    #     pq = [(0.0, start)]

    #     while pq:
    #         d, u = heapq.heappop(pq)
    #         if u in visited:
    #             continue
    #         visited.add(u)
    #         if u == goal:
    #             break
    #         for e in self.nodes[u].edges:
    #             nd = d + e.distance
    #             if e.to_node not in dist or nd < dist[e.to_node]:
    #                 dist[e.to_node] = nd
    #                 prev[e.to_node] = (u, e)
    #                 heapq.heappush(pq, (nd, e.to_node))

    #     if goal != start and goal not in prev:
    #         raise ValueError(f"no path found from {start} to {goal}")

    #     nodes_seq = [goal]
    #     edges_seq: List[Edge] = []
    #     cur = goal
    #     while cur != start:
    #         p, e = prev[cur]
    #         nodes_seq.append(p)
    #         edges_seq.append(e)
    #         cur = p
    #     nodes_seq.reverse()
    #     edges_seq.reverse()
    #     return nodes_seq, edges_seq

    def shortest_path(self, start: str, goal: str,
                       forbidden_start_directions: "frozenset[Direction]" = frozenset()
                       ) -> Tuple[List[str], List[Edge]]:
        """Dijkstra. forbidden_start_directions excludes specific edges only
        when expanding directly out of `start` (e.g. to rule out an
        immediate uturn) - it has no effect deeper in the graph."""
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
                if u == start: print(e.direction)
                if u == start and e.direction in forbidden_start_directions:
                    continue
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

    # def add_start_node_with_splice(self, name: str, x: float, y: float,
    #                                 heading: Direction) -> str:
    #     """Like add_node_with_splice, but keeps only the outgoing edge that
    #     matches the robot's current heading. This makes a uturn off the
    #     start point structurally unreachable - there's simply no edge
    #     leading backward for Dijkstra to find.

    #     If the projection lands on (or effectively at) an existing
    #     intersection, insert_point_on_edge already returns that real node
    #     instead of creating a phantom - nothing to constrain there; a real
    #     intersection's turn options are handled normally via path_to_turns'
    #     start_heading argument."""
    #     a, b, d = self.project_point(x, y)
    #     node_name = self.insert_point_on_edge(a, b, d, name)

    #     node = self.nodes[node_name]
    #     if not node.is_phantom:
    #         return node_name  # snapped onto a real node

    #     forward_edges = [e for e in node.edges if e.direction == heading]
    #     if forward_edges:
    #         node.edges = forward_edges
    #     else:
    #         print(f"[map_graph] start heading {heading} matches neither direction "
    #               f"at the snap point on edge {a}-{b}; keeping both (check pose/heading)")
    #     return node_name

    # def add_start_node_with_splice(self, name: str, x: float, y: float,
    #                                 heading: Direction) -> str:
    #     """Restrict the start node's outgoing edges so a uturn is never the
    #     robot's very first move - it isn't a maneuver the FSM can perform,
    #     and Dijkstra has no way to know it's physically impossible unless
    #     we tell it.

    #     - Phantom start (mid-edge): the edge only runs in one of two
    #       opposite compass directions, so `heading` must match one of them
    #       exactly - if it doesn't, the input heading is wrong (not a graph
    #       problem), so this raises rather than silently keeping an
    #       ambiguous graph for Dijkstra to exploit.
    #     - Real start (snapped onto an existing intersection): multiple
    #       directions are legitimately available - that's what makes it an
    #       intersection - so only the exact 180-degree reversal is excluded;
    #       left/right/straight all remain valid choices.
    #     """
    #     a, b, d = self.project_point(x, y)
    #     node_name = self.insert_point_on_edge(a, b, d, name)
    #     node = self.nodes[node_name]

    #     if node.is_phantom:
    #         forward_edges = [e for e in node.edges if e.direction == heading]
    #         if not forward_edges:
    #             raise ValueError(
    #                 f"heading {heading} matches neither direction of the edge "
    #                 f"{a}-{b} the start point landed on - check the pose/heading input")
    #         node.edges = forward_edges
    #     else:
    #         reverse = _OPPOSITE[heading]
    #         forward_edges = [e for e in node.edges if e.direction != reverse]
    #         if not forward_edges:
    #             raise ValueError(
    #                 f"every edge at '{node_name}' is a direct reversal of heading "
    #                 f"{heading} - check the pose/heading input")
    #         node.edges = forward_edges

    #     return node_name

    def add_start_node_with_splice(self, name: str, x: float, y: float,
                                    heading: Direction,
                                    snap_epsilon: float = NODE_SNAP_EPSILON
                                    ) -> Tuple[str, "frozenset[Direction]"]:
        """Resolve the robot's current position to a graph node, and work out
        which of its edges must be excluded as a first move so the plan
        never opens with a uturn. Returns (node_name, forbidden_directions) -
        forbidden_directions does NOT get applied by mutating node.edges
        (that would permanently corrupt a real intersection for every future
        plan); pass it into shortest_path's forbidden_start_directions
        instead, scoped to just this one call.

        snap_epsilon exists because pose noise means the projection almost
        never lands exactly on a node even when the robot is physically
        sitting right at one - it lands a few centimeters into the nearest
        edge instead. Treating "close enough" as "there" applies the
        permissive real-intersection rule (exclude only the exact reversal)
        instead of the strict mid-edge rule (heading must match the edge's
        own axis exactly) - which is what let a legal turn like East->South
        get rejected just because noise put the projection a few
        centimeters onto the southbound edge instead of on the corner.
        """
        a, b, d = self.project_point(x, y)
        edge_ab = next(e for e in self.nodes[a].edges if e.to_node == b)
        total = edge_ab.distance

        if d <= snap_epsilon:
            node_name = a
        elif d >= total - snap_epsilon:
            node_name = b
        else:
            node_name = self.insert_point_on_edge(a, b, d, name)

        node = self.nodes[node_name]
        available = {e.direction for e in node.edges}

        if node.is_phantom:
            # if heading not in available:
            #     raise ValueError(
            #         f"heading {heading} matches neither direction available at the "
            #         f"start point (on edge {a}-{b}) - check the pose/heading input")
            print("ADDING " + str(_OPPOSITE[heading]) + " to forbidden directions")
            if heading in available:
                forbidden = available - {heading}
            else:
                forbidden = {_OPPOSITE[heading]}
        else:
            reverse = _OPPOSITE[heading]
            if available <= {reverse}:
                raise ValueError(
                    f"every edge at '{node_name}' is a direct reversal of heading "
                    f"{heading} - check the pose/heading input")
            forbidden = {reverse} if reverse in available else set()

        return node_name, frozenset(forbidden)

    # def path_to_turns(self, edges_seq: List[Edge], nodes_seq: List[str],
    #                    start_heading: Optional[Direction] = None) -> List[str]:
    #     """One relative turn per REAL intersection departure. Phantom nodes
    #     (START/GOAL splices) never produce a turn: the robot isn't choosing
    #     a direction there, it's already committed to that edge just by being
    #     spliced onto it. start_heading is only needed if nodes_seq[0]
    #     happens to be a real, non-phantom node."""
    #     turns = []
    #     heading = start_heading
    #     for i, e in enumerate(edges_seq):
    #         node = self.nodes[nodes_seq[i]]
    #         # if node.is_phantom: continue 
    #         if not node.is_phantom:
    #             if heading is None:
    #                 raise ValueError(
    #                     f"path starts at real node '{node.name}' but no start_heading was given")
    #         # print(node.name, e.direction, heading, heading.turn_to(e.direction))
    #             turns.append(heading.turn_to(e.direction))
    #         elif (heading.turn_to(e.direction) == 'uturn'):
    #             turns.append('uturn')
    #         heading = e.direction
    #     return turns

    def path_to_turns(self, edges_seq: List[Edge], nodes_seq: List[str],
                       start_heading: Optional[Direction] = None) -> List[str]:
        """One relative turn per REAL intersection departure. Phantom nodes
        (START/GOAL splices) never produce a turn - a phantom START only
        ever has one outgoing edge (see add_start_node_with_splice), so
        there's no decision to make there regardless."""
        turns = []
        heading = start_heading
        for i, e in enumerate(edges_seq):
            node = self.nodes[nodes_seq[i]]
            if not node.is_phantom:
                if heading is None:
                    raise ValueError(
                        f"path starts at real node '{node.name}' but no start_heading was given")
                turns.append(heading.turn_to(e.direction))
            heading = e.direction
        return turns

    # def path_to_turns_and_target_coords(self, edges_seq, nodes_seq, start_heading):
    #     turns = []
    #     heading = start_heading
    #     for i, e in enumerate(edges_seq):
    #         node = self.nodes[nodes_seq[i]]
    #         next_node = e.to_node
    #         if not node.is_phantom:
    #             if heading is None:
    #                 raise ValueError(
    #                     f"path starts at real node '{node.name}' but no start_heading was given")
    #             target_coords = self.edge_coords[(node.name, next_node)]
    #             turns.append( ( heading.turn_to(e.direction), target_coords) )
    #         heading = e.direction
    #     return turns

    def path_to_turns_and_target_coords(self, edges_seq, nodes_seq, start_heading):
        turns = []
        heading = start_heading
        for i, e in enumerate(edges_seq):
            node = self.nodes[nodes_seq[i]]
            if not node.is_phantom:
                if heading is None:
                    raise ValueError(
                        f"path starts at real node '{node.name}' but no start_heading was given")
                try:
                    target_coords = self.edge_coords[(node.name, e.direction)]
                except KeyError:
                    raise KeyError(
                        f"no target_coords entry for departing '{node.name}' heading "
                        f"{e.direction} - add a target_coords entry for that direction "
                        f"in the yaml (any 'from: {node.name}' edge in that direction works, "
                        f"even if its original 'to' isn't the current destination)")
                turns.append((heading.turn_to(e.direction), target_coords))
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
        g.add_node(n["name"], n["x"] * config.TILE_SIZE_X , n["y"] * config.TILE_SIZE_X, is_phantom=n.get("is_phantom", False))
    for e in data["edges"]:
        g.add_edge(e["from"], e["to"], Direction[e["direction"]], e["distance"] * config.TILE_SIZE_X,
                   bidirectional=e.get("bidirectional", True))
    # for edge_coords in data['target_coords']:
    #     g.add_edge_coords(
    #         edge_coords['from'], edge_coords['to'], 
    #         edge_coords['x'] * config.TILE_SIZE_X,
    #         edge_coords['y'] * config.TILE_SIZE_Y
    #     )

    for edge_coords in data['target_coords']:
        from_node, to_node = edge_coords['from'], edge_coords['to']
        try:
            direction = next(e.direction for e in g.nodes[from_node].edges if e.to_node == to_node)
        except StopIteration:
            raise ValueError(
                f"target_coords entry '{from_node} -> {to_node}' doesn't match any edge "
                f"in the 'edges' list - check spelling/direction")
        g.add_edge_coords(from_node, direction,
                           edge_coords['x'] * config.TILE_SIZE_X,
                           edge_coords['y'] * config.TILE_SIZE_Y)
    return g