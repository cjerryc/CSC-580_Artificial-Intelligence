import numpy as np
import pandas as pd
import quads
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import random

def store_data_in_quadtree(csv_file):
    # Read the CSV file
    data = pd.read_csv(csv_file)
    
    # Determine the bounds of your data
    min_x, max_x = data['X'].min(), data['X'].max()
    min_y, max_y = data['Y'].min(), data['Y'].max()
    
    # Calculate the center of the QuadTree
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    
    # Calculate the width and height required to encompass all points
    # Adding a small margin to ensure all points are within bounds
    width = (max_x - min_x) * 1.1  # 10% margin
    height = (max_y - min_y) * 1.1  # 10% margin
    
    # Initialize the QuadTree with the calculated dimensions
    tree = quads.QuadTree((center_x, center_y), width, height)
    
    # Iterate through the CSV rows and insert each point into the QuadTree
    for index, row in data.iterrows():
        # Create a Point with X, Y coordinates and Z as data
        point = quads.Point(row['X'], row['Y'], data=row['Z'])
        tree.insert(point)
    
    return tree

# Replace 'your_data.csv' with the path to your actual CSV file
true_map = store_data_in_quadtree('elevation_data.csv')
quads.visualize(true_map)

class Drone:
    def __init__(self, shared_map, start):
        # Initialize the drone's position, the map it's sharing, and its initial facing direction
        self.x = start[0]
        self.y = start[1]
        self.map = shared_map
        self.found = False  # Indicates whether the drone has found the treasure
        self.turns = [(0,-1), (-1,0), (0,1), (1,0)]  # Possible directions: up, left, down, right
        self.facing = (0,1)  # Initial direction the drone is facing (right)

    def move(self):
        # If the drone has found the treasure, it stops moving
        if self.found:
            return
        self.map[self.x, self.y] = 1  # Mark the current position as visited
        trn = self.turns.index(self.facing) - 1  # Determine the next direction to check
        for i in range(4):  # Check all four directions
            ck = self.turns[(i + trn) % 4]
            if self.check(ck[0], ck[1]):  # Move if a valid direction is found
                return
        # if all 4 directions have been visited, find a direction search
        while True:
            max_x = shared_map.shape[0]
            max_y = shared_map.shape[1]
            rand_x = random.randint(0, max_x - 1)
            rand_y = random.randint(0, max_y - 1)
            spot = self.map[rand_x, rand_y]

            if spot != -1 and spot != 1:
                coin_flip = random.randint(0, 1)
                if coin_flip == 0 or rand_y == self.y:
                    self.y += (rand_x - self.x) // abs(rand_x - self.x)
                else:
                    self.x += (rand_y - self.y) // abs(rand_y - self.y)
                break

        

    def check(self, x, y):
        # Check if the next move is within the map boundaries and not an obstacle or visited
        new_x, new_y = self.x + x, self.y + y
        if not (0 <= new_x < self.map.shape[0] and 0 <= new_y < self.map.shape[1]):
            return False
        spot = self.map[new_x, new_y]
        if spot == -1 or spot == 1:  # Check for obstacles or visited spots
            return False
        if spot == 2:  # Check if the spot is the treasure
            print(f"Found treasure at {new_x}, {new_y}")
            self.found = True  # Mark as found and stop moving
        self.x = new_x
        self.y = new_y
        self.facing = (x, y)
        return True


# Initialize the shared map and drones
shared_map = np.zeros((40,40))
shared_map[14,10] = 2  # Place the treasure on the map
shared_map[5:10, 4:13] = -1  # Place obstacles
shared_map[17:30, 14:22] = -1  # Place obstacles

drone1 = Drone(shared_map, (0,0))  # Initialize drone 1
drone2 = Drone(shared_map, (3,3))  # Initialize drone 2 at the opposite corner

# Set up the animation plot
fig = plt.figure()
im = plt.imshow(shared_map, animated=True)  # Display the initial map
animation_running = True  # Flag to control the animiation 

def updatefig(fnum):
    global animation_running
    if animation_running:
        # Move each drone if the treasure hasn't been found
        if not drone1.found:
            drone1.move()
        if not drone2.found:
            drone2.move()
        
        im.set_array(shared_map)  # Update the image with the current map state

        # Stop the animation if either drone finds the treasure
        if drone1.found or drone2.found:
            animation_running = False

    return [im]

ani = animation.FuncAnimation(fig, updatefig, blit=True, interval=0.1, frames=200)
plt.show()