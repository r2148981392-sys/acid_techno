import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import griddata

class GridSquare:
    def __init__(self, x1, x2, y1, y2, ph):
        self.x1 = float(x1) # left
        self.x2 = float(x2) # right
        self.y1 = float(y1) # top
        self.y2 = float(y2) # bottom
        self.ph = float(ph) # pH 6 = black, pH 9 = white (higher pH = higher RGB values)
    
    def set_ph(self, ph):
        self.ph = ph

    def info(self):
        print("x1:", self.x1, "  x2:", self.x2, "  y1:", self.y1, "  y2:", self.y2, "  ph:", self.ph)


grid = [] # 2D array of GridSquares

grid_size_x = float(input("Search area x: "))
grid_size_y = float(input("Search area y: "))
square_size = float(input("Size of grid squares: "))

num_squares_x = int(grid_size_x // square_size)
num_squares_y = int(grid_size_y // square_size)

# [!] Update this whenever the location changes
ACIDITY_MAP_PATH = "C:/Users/20254973/OneDrive - TU Eindhoven/Documents/2IRR10 CBL Autonomous Systems Twinning/AcidityMap.jpg"

# Returns the index of the grid square in the grid which contains the given xy coordinates
# Format: (ix, iy), to be used as: ix, iy = grid_pos(x, y)
def grid_pos(x, y):
    if not -grid_size_x / 2 <= x < grid_size_x / 2:
        print("ERROR: Invalid x")
        return (-1, -1)
    if not -grid_size_y / 2 <= y < grid_size_y / 2:
        print("ERROR: Invalid y")
        return (-1, -1)
    
    ix = int((x + grid_size_x / 2) // square_size)
    iy = int((y + grid_size_y / 2) // square_size)

    return (ix, iy)


# Construct grid
for iy in range(num_squares_y):
    row = []
    y1 = - grid_size_y / 2 + square_size * iy
    y2 = - grid_size_y / 2 + square_size * (iy + 1)

    for ix in range(num_squares_x):
        x1 = - grid_size_x / 2 + square_size * ix
        x2 = - grid_size_x / 2 + square_size * (ix + 1)
        row.append(GridSquare(x1, x2, y1, y2, 7))

    grid.append(row)

# Load acidity map (this example one is 400x400)
acidity_map = mpimg.imread(ACIDITY_MAP_PATH)
acidity_map_height, acidity_map_width, _ = acidity_map.shape

# Create arrays for map generation
samples_x = []
samples_y = []
samples_ph = []

# Generate a sample from each grid square, and update the pH values set for each part of the grid
for iy in range(num_squares_y):
    image_y = int(acidity_map_height / num_squares_y * (0.5 + iy))
    for ix in range(num_squares_x):
        image_x = int(acidity_map_width / num_squares_x * (0.5 + ix))
        sample_value, _, _ = acidity_map[image_y, image_x] # brightness value; map indexing is row, column
        sample_ph = 6 + sample_value * (3 / 255)
        grid[iy][ix].set_ph(sample_ph)
        samples_x.append(image_x)
        samples_y.append(image_y)
        samples_ph.append(sample_ph)

# for row in grid:
#     for square in row:
#         square.info()

samples_x = np.array(samples_x)
samples_y = np.array(samples_y)
samples_ph = np.array(samples_ph)

# The grid is scaled to fit the points, as forcing it to have axes from 0-400 (like the original map)
# would cause errors for the interpolation. I believe the image can still be rendered as 400x400 though
grid_x, grid_y = np.mgrid[min(samples_x):max(samples_x):200j, min(samples_y):max(samples_y):200j]
grid_z = griddata((samples_x, samples_y), samples_ph, (grid_x, grid_y), method='linear')

plt.figure(figsize=(8, 6))

plt.imshow(
    grid_z.T, 
    extent=(min(samples_x), max(samples_x), min(samples_y), max(samples_y)), 
    origin='lower', 
    cmap='gray', 
    vmin=6.0, 
    vmax=9.0
)

plt.colorbar(label="Interpolated pH")
plt.title("Acidity map of search area from samples taken (scaled to 400x400)")
plt.xlabel("Image X (pixels)")
plt.ylabel("Image Y (pixels)")
plt.show()
