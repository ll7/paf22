import preplanning_trajectory as pt
from visualizer import TestVisualizer


x_start = 399.
y_start = -241.
x_target = 348.5
y_target = -0.0006
action = -1.571

# Initialise
conv = pt.OpenDriveConverter("town1.xodr")
conv.convert_roads()
conv.convert_junctions()
conv.filter_geometry()

# Trajectory for the starting road segment
conv.initial_road_trajectory(x_start, y_start, x_target, y_target)

# preplanning until first target point is reached
conv.target_road_trajectory(x_target, y_target, conv.rad_to_degree(action))
print("follow", conv.follow_id)

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
vis.visualize()
