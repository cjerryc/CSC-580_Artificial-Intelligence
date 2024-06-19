import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pandas as pd

# Load your dataset
data = pd.read_csv('elevation_data.csv')  # Replace with your CSV file path

# Extract 'X', 'Y', and 'Z' columns
x = data['X']
y = data['Y']
z = data['Z']

# Create a 3D scatter plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the points
scatter = ax.scatter(x, y, z)

# Label the axes
ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

# Show the plot
plt.show()
