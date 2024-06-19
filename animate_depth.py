from typing import Union
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import g2o
import SLAM_g2o as slam
import math

from matplotlib import rc
rc('animation', html='jshtml')


# CONSTANTS
# TODO: get all of the configureable constants here
NUMBER_OF_DRONES = 10
DRONE_MEASUREMENT_RADIUS = 15
LOCATION_REMEASUREMENT_DISINCENTIVE = 30
DRONE_MEASUREMENT_INTERVAL_STEPS = 40
ENVIRONMENT_MEASUREMENT_NOISE_STD = 40
ODOMETER_MEASUREMENT_NOISE_PROBABILITY = 0.25


def points_within_radius(r):
    points = []
    for i in range(-r, r + 1):
        for j in range(-r, r + 1):
            distance = math.sqrt((i) ** 2 + (j) ** 2)
            if distance <= r:
                points.append((i, j))
    return points


class DroneMovementPolicy():
    def __init__(self):
        pass
    
    def get_movement_direction(self, drone_position, all_drone_positions, map_construct):
        # TODO: implement learning
        return (1,0)

class Environment():
    """ Environment class to introduce noise when measured """
    def __init__(self, environment_array):
        self.array = environment_array
        self.width = len(self.array)
        self.length = len(self.array[0])
    
    def measure_spot(self, x, y):
        return self.array[x, y] + np.random.normal(scale=ENVIRONMENT_MEASUREMENT_NOISE_STD)

class DroneMap():
    """ Drone Map class to handle comparing measurements to existing data """
    def __init__(self, environment):
        # Zeros array same size as the environment
        self.width = environment.width
        self.length = environment.length
        self.map = np.zeros((self.width, self.length+1))
        # Color control
        self.map[0, self.length] = -50
        self.map[self.width-1, self.length] = 50
        # Keep a history of measurements made at each point
        self.measurements = {}
        # Keep a dictionary of each drone's location at any given time
        self.locations = {}
        # Class to handle learning implementation
        self.movement_policy = DroneMovementPolicy()
        # index into list of Pose Graphs with an index corresponding to each individual Mapping Drone
        self.pose_graphs = []  # List of Pose Graphs, 1 for each exploration agent
        self.pose_id = []  # List of most curr pose ID's, 1 for each exploration agent
        
    def get_locations_map(self):
        out_map = self.map.copy()
        for id in self.locations.keys():
            x, y = self.locations[id]
            out_map[x, y] = -500
        return out_map
        
    def update_spot(self, x, y, measurement, id):
        # Add measurement to list of measurements at that spot
        if not (x, y) in self.measurements.keys():
            self.measurements[(x, y)] = []
        self.measurements[(x, y)].append(measurement)
        
        if self.map[x, y] == 0:
            self.map[x, y] = measurement
        else:
            # TODO: better system of comparing current map data to measurement, instead of just averaging them
            self.map[x, y] = np.average(self.measurements[(x, y)])
        self.locations[id] = (x, y)
        
    def get_next_direction(self, id, surroundings):
        # TODO: have the movement_policy suggest a next move
        # return self.movement_policy.get_movement_direction(self.locations[id], self.locations, self.map)
        minPenalty = 10000000
        currDir = (0, 0)
        for d in surroundings.keys():
            if (surroundings[d] > 0) and (surroundings[d] <= minPenalty):
                minPenalty = surroundings[d]
                currDir = d
        return currDir

        # TODO: call receive_exploration_packet method somewhere in mapping drone to get packet info from exploration agents

    def receive_exploration_packet(self, exploration_drone_id, x, y, landmark=0, odometry=0, dir=0):
        drone_id = exploration_drone_id
        if drone_id == -1:
            drone_id = self.add_pose_graph()
        elif landmark == 1:
            self.add_landmark_2D_graph_SLAM(x, y, drone_id)
        elif odometry == 1:
            self.add_odometry_2D_graph_SLAM(x, y, dir, drone_id)
        return drone_id

        # Initialize a new pose graph for each Mapping Agent and add to List of Pose Graphs.

    def add_pose_graph(self):
        # Intitialize 2D SLAM GRAPH for each Drone
        self.pose_graphs.append(slam.GraphSLAM2D(verbose=True))
        drone_id = len(self.pose_graphs) - 1
        # Add initial fixed node
        self.add_node_2D_graph_SLAM(drone_id)
        self.pose_id.append(0)
        # Return the id of the exploring agent from the mapping agent's perspective.
        return drone_id

        # Initialize and assign a fixed node for 2D SLAM Graph

    def add_node_2D_graph_SLAM(self, drone_id):
        # Add fixed pose with ID
        self.pose_graphs[drone_id].add_fixed_pose(g2o.SE2())
        return

        # Add a landmark to the drone's 2D SLAM Graph for the fixed pose.

    def add_landmark_2D_graph_SLAM(self, x, y, drone_id):
        landmark_x = x
        landmark_y = y
        poseId = self.get_pose_id(drone_id)
        self.pose_graphs[drone_id].add_landmark(landmark_x, landmark_y, np.eye(2), pose_id=poseId)
        self.update_pose_id(drone_id)
        return

        # Add another pose and edge with Odometry to the drone's 2D SLAM Graph

    def add_odometry_2D_graph_SLAM(self, global_x, global_y, global_dir, drone_id):
        # global_x = northings, global_y = eastings, global_dir = headings ??????
        self.pose_graphs[drone_id].add_odometry(global_x, global_y, global_dir, 0.1 * np.eye(3))
        self.update_pose_id(drone_id)
        return

    def get_pose_id(self, drone_id):
        return self.pose_id[drone_id]

    def update_pose_id(self, drone_id):
        self.pose_id[drone_id] += 1
        return


class Drone():
    def __init__(self, environment, start, map_agent, id):
        self.id = id
        self.map_drone_id = -1  # The id of the drone relative to the central mapping drone
        # TODO: x_icp, and x_komonfilter combines x_odometer and x_icp to give the actual percieved location
        self.x_actual = start[0] # Drone's true position x
        self.y_actual = start[1] # Drone's true position y
        self.x_odom = start[0] # where the drone think's it is based on movements
        self.y_odom = start[1] # where the drone think's it is based on movements
        self.odom_points = []  # List of last k points to odometry packet
        
        self.env = environment
        self.found = False # Has the drone has found the target yet
        self.turns = [(0,-1), (-1,0), (0,1), (1,0)] # All 4 2d directions in clockwise order
        self.facing = (0,1) # Direction the drone is facing
        
        self.map = map_agent
        self.traversed = np.zeros((self.env.length, self.env.width+1))
        
        # Variable to save a map value when indicating drone's location
        self.saved_value = 0
        self.odometry_noise = ODOMETER_MEASUREMENT_NOISE_PROBABILITY
        self.measurement_interval = DRONE_MEASUREMENT_INTERVAL_STEPS
        self.measurement_interval_timer = self.measurement_interval
        self.odom_measurement_interval = 5
        self.odom_measurement_interval_timer = self.odom_measurement_interval
      
    def get_map(self):
        return self.map.map
    
    def observe_local_radius(self, r):
        points = points_within_radius(r)
        for p in points:
            self.measure_spot(p[0], p[1])
        
    def move(self):
        if self.found:
            return
        
        # Take a sonar measurement at intervals
        if self.measurement_interval_timer <= 0:
            self.observe_local_radius(DRONE_MEASUREMENT_RADIUS)
            self.measurement_interval_timer = self.measurement_interval
        else:
            self.measurement_interval_timer -= 1
        
        # Check/map spots around and then move to the best one
        # TODO: get next direction from central mapping agent
        surroundings = self.check_spots()
        moveDirection = self.map.get_next_direction(self.id, surroundings)
        if self.odom_measurement_interval_timer <= 0:
            x, y = self.calculate_new_odometry_position()
            self.send_exploration_packet(x, y)
            self.odom_points = []
            self.odom_measurement_interval_timer = self.odom_measurement_interval
        else:
            self.take_spot(moveDirection)
            self.odom_points.append((self.x_odom, self.y_odom))
            self.odom_measurement_interval_timer -= 1
        self.traversed[self.x_odom, self.y_odom] = LOCATION_REMEASUREMENT_DISINCENTIVE + self.traversed[self.x_odom, self.y_odom]
        return
                
    def take_spot(self, d):
        self.x_actual = self.x_actual + d[0]
        self.y_actual = self.y_actual + d[1]
        self.x_odom = self.x_odom + d[0]
        self.y_odom = self.y_odom + d[1]
        # Adds noise to odometry measurement
        # The drone TRIES to move 1 square, but in reality sometimes it goes 2 or 0
        # Thus the odometry data reflexts a movement of 1, but the actual data reflects a bit differently
        if np.random.rand() <= self.odometry_noise:
            # Either go a bit farther or a bit shorter than we thought
            if np.random.rand() <= 0.5:
                self.x_actual = self.x_actual + d[0]
                self.y_actual = self.y_actual + d[1]
            else:
                self.x_actual = self.x_actual - d[0]
                self.y_actual = self.y_actual - d[1]
        self.facing = d
        
    def check_spots(self):
        """ Check all adjacent spots and get penalties for moving to each """
        surroundings = {}
        trn = self.turns.index(self.facing) - 1 # Always try to turn left first
        for i in range(4): # Try moving left, straight, right, backwards
            d = self.turns[(i + trn) % 4] # direction to try
            measurement = self.measure_spot(d[0], d[1]) # Returns value or None if spot is off the map
            if measurement:
                # Compare current altiture to measured location
                surroundings[d] = (measurement - self.map.map[self.x_odom, self.y_odom])**2
                # Add a penalty for traversing a spot multiple times
                surroundings[d] = surroundings[d] + self.traversed[self.x_odom + d[0], self.y_odom + d[1]]**2
            else:
               surroundings[d] = -1 # Spot was not moveable
        return surroundings
        
    def measure_spot(self, x_d, y_d) -> Union[float, None]:
        """ Measure the spot (x, y), update the map with the spot and return most up to date measurement from map """
        # Make sure the space is on the map
        if ((self.x_actual + x_d) < 0) or ((self.x_actual + x_d) > (self.map.length-1)):
            return None
        if ((self.x_odom + x_d) < 0) or ((self.x_odom + x_d) > (self.map.length-1)):
            return None
        if ((self.y_actual + y_d) < 0) or ((self.y_actual + y_d) > (self.map.width-1)):
            return None
        if ((self.y_odom + y_d) < 0) or ((self.y_odom + y_d) > (self.map.width-1)):
            return None
        
        # Value of the spot we are checking
        measurement = self.env.measure_spot(self.x_actual + x_d, self.y_actual + y_d)
        self.map.update_spot(self.x_odom + x_d, self.y_odom + y_d, measurement, self.id)
        return self.map.map[self.x_odom + x_d, self.y_odom + y_d] # Our most up to date measurement

    # Send the exploration odometry packet to the mapping drone to construct pose graph, and  get map-relative drone ID.
    def send_exploration_packet(self, x, y):
        # Send Odometry data with calculated x, y positions, direction = theta = 0 (Because we are in 2D)
        self.map_drone_id = self.map.receive_exploration_packet(self.map_drone_id, x, y, 0, 1, 0)
        return

    def calculate_new_odometry_position(self):
        x = self.x_odom
        y = self.y_odom
        # x = self.odom_points[0][0]
        # y = self.odom_points[0][1]
        # vec = (0, 0)
        # for i in range(len(self.odom_points)):
        #     # TODO: Iterate through every pair and find vectors, then add all vectors together.
        #     #  Finally, add the vector to the initial point and return taht as the (x, y) to send in packet.
        #
        #     vec += self.odom_points[i+1] - self.odom_points[i]
        return x, y
        
def compare_map_to_environment(mapping_agent, environment):
    map_grid = mapping_agent.map[:,:-1]
    diff = map_grid - environment.array
    return np.linalg.norm(diff) # Euclidean distance, sum of squared differences

import csv
max_x = 0
max_y = 0
environment_grid = np.zeros(((201,201)))
with open('interpolated_elevation_data.csv', mode ='r')as file:
    csvFile = csv.reader(file)
    for line in list(csvFile)[1:]:
        x = int(float(line[0])//1)
        y = int(float(line[1])//1)
        val = float(line[2])
        environment_grid[x,y] = val
        if x > max_x:
            max_x = x
        if y > max_y:
            max_y = y

# Initialize Environment and Agents
environment = Environment(environment_grid[90:190,90:190])
mapping_agent = DroneMap(environment)
drones = []
for i in range(NUMBER_OF_DRONES):
    start = (100*np.random.rand((2))).astype(int)
    drones.append(Drone(environment, start, mapping_agent, i))

# Set up animation plot
fig = plt.figure()
im = plt.imshow(mapping_agent.map)
plt.clim(-100, -5)
plt.set_cmap("viridis")
def updatefig(fnum):
    # Each animation step, have the drone move
    for d in drones:
        d.move()
    # Update the plot with the mapping agent's updated environment
    im.set_array(mapping_agent.get_locations_map())
    print("Map accuracy score: ",compare_map_to_environment(mapping_agent, environment))
    return im,
# Animate
ani = animation.FuncAnimation(fig, updatefig,  blit=True, interval=0.1, frames=1000)
plt.show()

# writergif = animation.PillowWriter(fps=30)
# ani.save('drone_sim.gif',writer=writergif)
# ani.save('drone_sim.mp4',
#           writer = 'ffmpeg', fps = 30)