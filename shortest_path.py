import torch
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import json
from queue import PriorityQueue
import os

import pathlib

# Force pathlib to use WindowsPath instead of PosixPath
temp = pathlib.PosixPath
pathlib.PosixPath = pathlib.WindowsPath

model = torch.hub.load('yolov5', 'custom', path='best(13).pt', source="local", force_reload=True)

# Restore original pathlib.PosixPath to avoid conflicts
pathlib.PosixPath = temp

# ### Load YOLO Model ###
# model = torch.hub.load('yolov5', 'custom', path='best(13).pt', source="local", force_reload=True)


# Load Image
image_path = "WhatsApp Image 2025-01-29 at 14.29.10_8117a3be.jpg"  # Change to your actual image path
image = cv2.imread(image_path)
image_height, image_width, _ = image.shape

# Perform inference
results = model(image)

# Extract predictions
detections = results.pandas().xywh[0]  # Get detected objects in xywh format
predictions_data = {"predictions": [], "image": {"width": image_width, "height": image_height}}
predictions_file = "predictions.json"

# Load existing predictions if the file exists
if os.path.exists(predictions_file):
    with open(predictions_file, "r") as json_file:
        predictions_data = json.load(json_file)
else:
    predictions_data = {"predictions": []}

for _, row in detections.iterrows():
    class_name = row["name"]  # Class name ('Cones', 'Obstacles')
    prediction = {
        "x": row["xcenter"],
        "y": row["ycenter"],
        "width": row["width"],
        "height": row["height"],
        "confidence": row["confidence"],
        "class": class_name,
        "class_id": row["class"],
        "image_path": image_path,
        "prediction_type": "ObjectDetectionModel"
    }
    predictions_data["predictions"].append(prediction)

# Save Predictions to JSON File
with open("predictions.json", "w") as json_file:
    json.dump(predictions_data, json_file, indent=4)

print("Predictions saved to predictions.json!")
# Load predictions from JSON instead of in-memory variable
with open("predictions.json", "r") as json_file:
    predictions_data = json.load(json_file)

print(json.dumps(predictions_data,indent=4))
### Separate Obstacles & Cones ###
obstacle_list = [p for p in predictions_data["predictions"] if p["class"].lower() == "obstacles"]
waypoint_list = [p for p in predictions_data["predictions"] if p["class"].lower() == "cones"]

### Create Occupancy Grid ###
def create_occupancy_grid(grid_size, obstacles):
    grid = np.zeros(grid_size)
    for obs in obstacles:
        x1, y1 = int(obs["x"] - obs["width"] / 2), int(obs["y"] - obs["height"] / 2)
        x2, y2 = int(obs["x"] + obs["width"] / 2), int(obs["y"] + obs["height"] / 2)
        grid[y1:y2, x1:x2] = 1  # Mark obstacle cells
    return grid

grid_size = (image_height, image_width)
occupancy_grid = create_occupancy_grid(grid_size, obstacle_list)

### A* Pathfinding Algorithm ###
def heuristic(a, b):
    return np.linalg.norm(np.array(a) - np.array(b))

def astar(grid, start, goal):
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    cost_so_far = {start: 0}

    while not open_set.empty():
        current = open_set.get()[1]

        if current == goal:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, 1), (-1, 1), (1, -1)]:
            neighbor = (current[0] + dx, current[1] + dy)
            if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1] and grid[neighbor] == 0:
                new_cost = cost_so_far[current] + (1 if dx == 0 or dy == 0 else np.sqrt(2))
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(neighbor, goal)
                    open_set.put((priority, neighbor))
                    came_from[neighbor] = current

    path = []
    current = goal
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path

def astar_multi(grid, start, waypoints):
    path = []
    current_start = start
    for waypoint in waypoints:
        segment_path = astar(grid, current_start, waypoint)
        path.extend(segment_path[:-1])
        current_start = waypoint
    path.extend(astar(grid, current_start, waypoints[-1]))
    return path

# Convert waypoints to integer grid positions
waypts = [(int(p["x"]), int(p["y"])) for p in waypoint_list]

# Set start & destination
start = (50, 50)  # Modify as needed
destination = (image_width - 50, image_height - 50)  # Modify as needed
waypts.append(destination)

# Compute Path
path_direct = astar_multi(occupancy_grid, start, waypts)
print("Obstacles:",obstacle_list)
print("waypoints:",waypoint_list)

### Visualization ###
def plot_results(image, occupancy_grid, start, destination, path, waypoints):
    fig, ax = plt.subplots()
    ax.imshow(image[:, :, ::-1])  # Convert BGR to RGB

    # Plot obstacles
    for obs in obstacle_list:
        x, y, w, h = obs["x"], obs["y"], obs["width"], obs["height"]
        rect = patches.Rectangle((x - w/2, y - h/2), w, h, linewidth=1, edgecolor="red", facecolor="none")
        ax.add_patch(rect)

    # Plot waypoints
    waypoint_x = [p[0] for p in waypoints]
    waypoint_y = [p[1] for p in waypoints]
    ax.scatter(waypoint_x, waypoint_y, marker="o", color="blue", label="Waypoints")

    # Plot start & destination
    ax.scatter(start[0], start[1], marker="*", color="green", s=100, label="Start")
    ax.scatter(destination[0], destination[1], marker="*", color="red", s=100, label="Destination")

    # Plot path
    for i in range(len(path) - 1):
        ax.plot([path[i][0], path[i + 1][0]], [path[i][1], path[i + 1][1]], color="orange", linewidth=2)

    ax.legend()
    plt.title("Path Planning with Detected Obstacles")
    plt.show()

plot_results(image, occupancy_grid, start, destination, path_direct, waypts)
