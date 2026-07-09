#!/usr/bin/env python3
import os
import tkinter as tk
from tkinter import messagebox
from PIL import Image, ImageTk
from map_graph import RoadGraph, load_graph_from_yaml

import config

class VisualApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Duckietown Mission Control")
        
        # UI Elements
        self.instruction_label = tk.Label(root, text="Click on the map to set a destination, then press 'Go!'", font=("Arial", 12))
        self.instruction_label.pack(pady=5)
        
        self.coord_label = tk.Label(root, text="Target: None", font=("Arial", 10), fg="blue")
        self.coord_label.pack()

        # Canvas for the map
        self.canvas = tk.Canvas(root, cursor="crosshair")
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        # Load Map Image (Update the path to your actual Duckietown/Duckiematrix map image)
        try:
            self.map_image = Image.open("images/map_image.png")
            self.map_photo = ImageTk.PhotoImage(self.map_image)
            self.canvas.config(width=self.map_image.width, height=self.map_image.height)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=self.map_photo)
            self.scale_x = config.TILE_SIZE_X / (self.map_image.width / config.NUM_TILES_X)
            self.scale_y = config.TILE_SIZE_Y / (self.map_image.height / config.NUM_TILES_Y)
        except Exception as e:
            messagebox.showerror("Error", f"Could not load map image:\n{e}")
            
        # Target state
        self.target_pixel = None
        self.target_world = None
        self.marker = None

        # Bindings and Buttons
        self.canvas.bind("<Button-1>", self.on_map_click)
        
        self.go_button = tk.Button(root, text="Go!", font=("Arial", 14, "bold"), bg="green", fg="white", command=self.send_goal)
        self.go_button.pack(pady=10)

        self.new_goal_callback = None

        self.graph = load_graph_from_yaml("graphs/virtual.yaml")
        self.graph.add_node_with_splice("START", 0.88, 0.185)
        self.visualize_road_graph(self.graph)

    def on_map_click(self, event):
        """Handle map clicks, draw a marker, and calculate coordinates."""
        x, y = event.x, event.y
        self.target_pixel = (x, y)
        self.target_world = self.pixel_to_world(x, y)

        self.coord_label.config(text=f"Target Pixel: ({x}, {y}) | Target World: ({self.target_world[0]:.2f}m, {self.target_world[1]:.2f}m)")
        
        # Draw a visual marker on the map
        if self.marker:
            self.canvas.delete(self.marker)
        r = 5
        self.marker = self.canvas.create_oval(x-r, y-r, x+r, y+r, fill="red", outline="black")

    def register_goal_callback(self, callback):
        self.new_goal_callback = callback

    def pixel_to_world(self, px, py):
        """Map pixel coordinates to Duckietown real-world coordinates."""
        world_x = (px * self.scale_x) + config.OFFSET_X
        py_flipped = self.map_image.height - py
        world_y = (py_flipped * self.scale_y) + config.OFFSET_Y
        return (world_x, world_y)

    def world_to_pixel(self, world_x, world_y):
        px = (world_x - config.OFFSET_X) / self.scale_x
        py_flipped = (world_y - config.OFFSET_Y) / self.scale_y
        py = self.map_image.height - py_flipped
        return (px, py)
    
    def visualize_road_graph(self, graph: RoadGraph):
        # 1. Draw edges first so they appear underneath the nodes
        for _, node in graph.nodes.items():
            start_px, start_py = self.world_to_pixel(node.x, node.y)
            
            for edge in node.edges:
                target_node = graph.nodes[edge.to_node]
                end_px, end_py = self.world_to_pixel(target_node.x, target_node.y)
                
                # Draw a line with an arrow pointing to the destination
                self.canvas.create_line(
                    start_px, start_py, end_px, end_py, 
                    fill='green', width=2, arrow=tk.LAST, arrowshape=(10, 12, 4)
                )

        # 2. Draw nodes on top
        for _, node in graph.nodes.items():
            # print("GOT NODE AT: ", node.x, node.y)
            px, py = self.world_to_pixel(node.x, node.y)
            # print("CONVERTED COORDS: ", px, py)
            
            # Phantom nodes are red, regular intersections are blue
            node_color = 'red' if node.is_phantom else 'blue'
            
            self.canvas.create_oval(
                px - 6, py - 6, px + 6, py + 6, 
                fill=node_color, outline='black', width=2
            )
            
            # Optional: draw the node name next to it for easier debugging
            self.canvas.create_text(
                px + 10, py - 10, text=node.name, fill="black", font=("Arial", 10, "bold")
            )
    
    def send_goal(self):
        """Publish the goal to ROS."""
        if not self.target_world:
            messagebox.showwarning("Hold up", "Please click a destination on the map first.")
            return
        
        # self.canvas.delete("all") # clear canvas to redraw cleanly
        # self.canvas.create_image(0, 0, anchor=tk.NW, image=self.map_photo) #

        # self.graph.restore_graph(["START", "END"])
        # self.graph.add_node_with_splice("END", *self.target_world)
        # self.visualize_road_graph(self.graph)

        if self.new_goal_callback is not None:
            self.new_goal_callback(self.target_world)

    def draw_path(self, nodes_seq: list):
        """Draws a highlighted line showing the route the Duckiebot will take."""
        # Clear any previous paths from the canvas
        self.canvas.delete("path_line")
        
        if not nodes_seq or len(nodes_seq) < 2:
            return
            
        for i in range(len(nodes_seq) - 1):
            n1 = self.graph.nodes[nodes_seq[i]]
            n2 = self.graph.nodes[nodes_seq[i+1]]
            
            px1, py1 = self.world_to_pixel(n1.x, n1.y)
            px2, py2 = self.world_to_pixel(n2.x, n2.y)
            
            # Draw a thick orange line under the nodes but over the edges
            self.canvas.create_line(
                px1, py1, px2, py2,
                fill="orange", width=5, tags="path_line", capstyle=tk.ROUND
            )
            # Push the path below the nodes for a cleaner look
            self.canvas.tag_lower("path_line")

if __name__ == "__main__":
    root = tk.Tk()
    app = VisualApp(root)
    root.mainloop()