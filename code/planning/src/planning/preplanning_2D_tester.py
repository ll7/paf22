import preplanning_trajectory as pt
from visualizer import TestVisualizer

"""
x_start = 983.5
y_start = -5373.2
x_target = 983.6123657226562
y_target = -5370.88720703125
action = 0

# Initialise
conv = pt.OpenDriveConverter("Town12.xodr")
conv.convert_roads()
conv.convert_junctions()
conv.filter_geometry()

# Trajectory for the starting road segment
conv.initial_road_trajectory(x_start, y_start, x_target, y_target)


# preplanning until first target point is reached
conv.target_road_trajectory(x_target, y_target, conv.rad_to_degree(action))
print("follow", conv.follow_id)
"""

"""
# preplanning until second target point is reached
conv.target_road_trajectory(332., -46., conv.rad_to_degree(0.001))
print("follow", conv.follow_id)

# preplanning until third target point is reached
conv.target_road_trajectory(100., -55., conv.rad_to_degree(4.712))
print("follow", conv.follow_id)

# visualize the whole trajectory and the reference line
# Two red points are the start and the endpoint of the first part
# of the trajectory
vis = TestVisualizer((x_start, y_start), (x_target, y_target),
                     conv.waypoints[0], conv.waypoints[1],
                     conv.reference[0], conv.reference[1])
vis.visualize()"""


x_start = 332.
y_start = -30.
x_target = 332.
y_target = -46.
action = -0

# Initialise
conv = pt.OpenDriveConverter("town1.xodr")
conv.convert_roads()
conv.convert_junctions()
conv.filter_geometry()

# Trajectory for the starting road segment
conv.initial_road_trajectory(x_start, y_start, x_target, y_target)

# preplanning until first target point is reached
conv.target_road_trajectory(x_target, y_target, conv.rad_to_degree(action))

# preplanning until second target point is reached
# conv.target_road_trajectory(332., -46., conv.rad_to_degree(0.001))

# preplanning until third target point is reached
conv.target_road_trajectory(100., -55., conv.rad_to_degree(4.712))
print("follow", conv.follow_id)

# visualize the whole trajectory and the reference line
# Two red points are the start and the endpoint of the first part
# of the trajectory
vis = TestVisualizer((x_start, y_start), (x_target, y_target),
                     conv.waypoints[0], conv.waypoints[1],
                     conv.reference[0], conv.reference[1])
vis.visualize()
