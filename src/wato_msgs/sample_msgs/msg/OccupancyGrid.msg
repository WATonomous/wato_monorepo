# Took this from wato_monorepo_v1

# Occupancy Grid
#########################

GridY[] x_grid # 2d array of uint8

# grid values are assigned as follows:
#  0 = no obstacle
#  1 = obstacle

# See software/integration wiki for specification of the grid's coordinate system

# Metadata
#########################

uint8 resolution # number of grid spaces per 1 m

uint8 x_offset
uint8 y_offset

# you can compute the actual position of x_grid[a][b] in relation to (0,0) on
# car (in meters) by doing:
#  (x, y) = ((a * resolution - x_offset), (b * resolution - y_offset))
#  NOTE: resolution is applied before the offsets
