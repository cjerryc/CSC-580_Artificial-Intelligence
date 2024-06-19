import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

from matplotlib import rc
rc('animation', html='jshtml')

class Drone():
    def __init__(self, environment, start):
        self.timer = 0
        self.totalTime = 0
        self.x = start[0] # Drone position x
        self.y = start[1] # Drone position y
        self.env = environment # Drone's environment
        self.found = False # Has the drone has found the treasure yet
        self.turns = [(0,-1), (-1,0), (0,1), (1,0)] # All 4 2d directions in clockwise order
        self.facing = (0,1) # Direction the drone is facing
        
        self.map = np.zeros((len(self.env), len(self.env[0])+1))
        self.map[0,len(self.env[0])] = -50
        self.map[len(self.env)-1,len(self.env[0])] = 50

        self.traversed = np.zeros((len(self.env), len(self.env[0])+1))
        
        self.open_spots = []
        
    def get_map(self):
        return self.map
        
    def move(self):
        self.totalTime = self.totalTime + 1
        if self.timer > 0:
           self.timer = self.timer - 1
           return
        if self.found:
            return
        
        self.map[self.x, self.y] = self.env[self.x, self.y]
        
        # Check/map spots around and then move to the best one
        surroundings = self.check_spots()
        print(surroundings)
        canMove = False,
        minPenalty = 1000000
        for d in surroundings.keys():
            if (surroundings[d] > 0) and (surroundings[d] < minPenalty):
                minPenalty = surroundings[d]
                canMove = True
                currDir = d
                # self.history.append(d)
            elif surroundings[d] > 0:
                self.open_spots.append(d)
        if not canMove:
           currDir = self.open_spots[-1]
        print(currDir, self.x, self.y)
        oldX = self.x
        oldY = self.y
        self.x = self.x + currDir[0]
        self.y = self.y + currDir[1]
        self.facing = currDir
        self.timer = abs(self.env[oldX, oldY] - self.env[self.x, self.y])**2
        self.traversed[self.x, self.y] = 1
        self.map[self.x, self.y] = 50
        return
                
                # self.take_spot(d)
                # self.history.append(d)
                
        # TODO: no direction found, so find nearest unexplored space
        # d = self.turns[(self.turns.index(self.history[-1])+2) % len(self.turns)]
        # self.history = self.history[:-1]
        # self.take_spot(d)
        
        
    def take_spot(self, d):
        # self.history.append(d)
        self.x = self.x + d[0]
        self.y = self.y + d[1]
        self.facing = d
        
    def check_spots(self):
        surroundings = {}
        trn = self.turns.index(self.facing) - 1 # Always try to turn left first
        for i in range(4): # Try moving left, straight, right, backwards
            d = self.turns[(i + trn) % 4] # direction to try
            if self.try_move(d[0], d[1]) == True:
               surroundings[d] = (self.env[d[0] + self.x, d[1]+self.y] - self.env[self.x, self.y])**2
               if self.traversed[self.x + d[0], self.y + d[1]] == 1:
                  surroundings[d] = surroundings[d] + 1000
            else:
               surroundings[d] = -1
            #surroundings[d] = self.try_move(d[0], d[1])
        return surroundings
        
    def try_move(self,x_d,y_d):
        # Make sure the space is on the map
        x = self.x + x_d
        y = self.y + y_d
        if ((x)<0) or ((x)>(len(self.env)-1)):
            return False
        if ((y)<0) or ((y)>(len(self.env[0])-1)):
            return False
        # Value of the spot we are checking
        spot = self.env[x, y]
        visited = self.map[x, y]
        print(spot, x, y)
        # if spot < 0:
        #     self.map[x,y] = spot
        #     return False # There was an obstacle there
        # if spot == 2: 
        #     print(f"Found {spot} at {x}, {y}")
        #     self.found = True # This was the treasure
        if visited == 0: # Movable space
            self.map[x, y] = spot
            # self.x = x
            # self.y = y
            # self.facing = (x_d,y_d)
            return True # We move
        return True
        
# Create a 2d array to represent the map
# environment = []
# environment = np.zeros((40,40))
# # Add in a treasure for the drones to find
# environment[25,25] = 2
# # Add in some obstacles
# environment[5:10, 4:13] = -1
# environment[17:30, 14:22] = -1
# Initialize a drone with the map given as its environment
import csv
max_x = 0
max_y = 0
environment = np.zeros(((201,201)))
with open('interpolated_elevation_data.csv', mode ='r')as file:
    csvFile = csv.reader(file)
    for line in list(csvFile)[1:]:
      x = int(float(line[0])//1)
      y = int(float(line[1])//1)
      val = float(line[2])
      environment[x,y] = val
      # print(val)
      if x > max_x:
         max_x = x
      if y > max_y:
         max_y = y
      # print(len(line))
      # print(f"line: {int(float(line[0])//1), int(float(line[1])//1), float(line[2])}")
    #   print(f"max_x: {max_x}, max_y = {max_y}")
# plt.imshow(environment)
# plt.show()
# exit()

# for i in range(len(environment)):
#     for j in range(len(environment[0])):
#         environment[i,j] = 10*np.sin(0.2*i) + 10*np.sin(0.05*j)
dr = Drone(environment[:50,:50], (25,25))

# Set up animation plot
fig = plt.figure()
im = plt.imshow(dr.get_map(), animated=True)
def updatefig(fnum):
    # Each animation step, have the drone move
    dr.move()
    # Update the plot with the drone's updated environment
    im.set_array(dr.get_map())
    return im,
# Animate
ani = animation.FuncAnimation(fig, updatefig,  blit=True, interval=0.1, frames=1000)
plt.show()

