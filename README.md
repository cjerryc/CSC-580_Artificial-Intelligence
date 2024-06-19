# CSC-580_Artificial-Intelligence
Cal Poly SLO Winter '24, Artificial Intelligence
Authors: Jerry Chang, Ryan Maier, Noah Ravetch, Nicholas Zarate, Alexander Arrieta, Jakob Frabosilio

SLAM (Simultaneous Localization and Mapping), is an algorithm for creating a representation of an unknown environment while also keeping track of where an agent is in the environment
The multi-agent version of the algorithm has additional complexity:
Creating the central map requires the stitching of several individual maps together.
The individual agents must move in different directions in order to efficiently search the environment.
Our project is an implementation of the multi-agent SLAM algorithm using drones as our searching agents.
Potential user audiences include: Search and rescue teams may find value in this project as SLAM drones can cover more area in less time than traditional on-foot search methods. Without finding a point of interest, this model may also be used for standard mapping of a given space, in varying land and sea elevations, when utilized through the appropriate equipment.

In this project, we have attempted to create a realistic simulation of Multi-Agent exploration in a space of varying elevation in which the agents  map out the region bounded between a user-defined space. In doing so, we simulate Multi-Agent interaction in a physical space and may visualize their progress and movement choices in the environment.
Simulating this physical space mapping reflects the exploratory needs Search and Rescue initiatives seeks to fill.
Model for how Multi-Agent SLAM can be implemented practically with drones by coordinating drones to employ autonomous searching behavior and make efficient mapping actions with regard to the difficulty of reaching a space due to the contours of an environment.

Our map data is gathered using the Google Maps API, as a 2D representation of a 3D physical space.
Referring to the Flowchart, it may be seen that there are three main components: the Environment, Exploration Agent, and Mapping Agent (Figure 3).
The Exploration Agents interact with the environment to gather sonar and position data, sending the data to the Mapping Agent.
The Mapping Agent receives packets of data from all agents in the Multi-Agent System, processing them and performing SLAM to stitch together an aggregate map, which is then sent back Exploration Agent-specific map fragments.
The SLAM calculation takes potential noise into account by running ICP on drone position and sonar data to identify potential drift and make adjustments through loop closure (Figure 4).
The Exploration Agents then take the map fragments to update their own maps with what other Agents have explored, updating individual knowledge of the currently searched space. 
Matplotlib is used to visualize environment and drone mapping.

Visual analysis of the program output was used to discern patterns and trends.
Reward shaping was implemented by comparing the differences between the mapping agent's map and the true environmental map. 
The quantification of rewards, coupled with the steps required to attain them, establishes a basis for evaluation.

Future Work:
Combine the advanced features of our branched simulations together, including the noising of data and the drone’s Q-learning search policy.
Tune our Q-learning implementation to increase the efficiency of our search algorithm
Expand our simulation to a fully 3D environment, allowing continuous movement and a realistic cone of vision.

References:
[1] “Coordinate Frame - DGC Wiki,” www.nast-group.caltech.edu. http://www.nast-group.caltech.edu/~murray/dgc05/wiki/c/o/o/Coordinate_Frame_812c.html 
[2] M. Massot, “miquelmassot/g2o-python-examples,” GitHub, Jan. 31, 2024. https://github.com/miquelmassot/g2o-python-examples/tree/main 
[3] X. Wang, X. Fan, P. Shi, J. Ni, and Z. Zhou, “An Overview of Key SLAM Technologies for Underwater Scenes,” Remote Sensing, vol. 15, no. 10, p. 2496, Jan. 2023, doi: https://doi.org/10.3390/rs15102496.
‌[4] “Graph based SLAM — PythonRobotics documentation,” atsushisakai.github.io. https://atsushisakai.github.io/PythonRobotics/modules/slam/graph_slam/graph_slam.html 
‌[5] H. Bai, “ICP Algorithm: Theory, Practice And Its SLAM-oriented Taxonomy.” Available: https://arxiv.org/ftp/arxiv/papers/2206/2206.06435.pdf


