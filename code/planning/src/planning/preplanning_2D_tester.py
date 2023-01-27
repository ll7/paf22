import preplanning_trajectory as pt
from visualizer import TestVisualizer
import math
import numpy as np


x_start = 983.5
y_start = -5373.2
x_target = 983.2598876953125
y_target = -5381.88720703125

end_x = list()
end_y = list()

# x_target = 983.61236
# y_target = -5370.88720703125
action = 0

# Initialise
conv = pt.OpenDriveConverter("Town12.xodr")
conv.convert_roads()
conv.convert_junctions()
conv.filter_geometry()

print(conv.rad_to_degree(-3.141))
print(conv.rad_to_degree(-1.572))
print(conv.rad_to_degree(-0.9044))
print("Numpy", np.degrees(-0.9044))
yaw = -0.9044
yaw = yaw * 180. / math.pi
if yaw < 0:
    yaw += 360
print(yaw)


# Trajectory for the starting road segment
end_x.append(x_target)
end_y.append(y_target)
conv.initial_road_trajectory(x_start, y_start, x_target, y_target,
                             983.2598876953125, -5568.28662109375, -90,
                             conv.rad_to_degree(-2.1377235095443834), 4)
# preplanning until first target point is reached

end_x.append(983.2598876953125)
end_y.append(-5568.28662109375)
conv.target_road_trajectory(983.2598876953125, -5568.28662109375,
                            conv.rad_to_degree(-2.137723509544383), 2)
print("current", conv.road_id)
print("Follow", conv.follow_id)

"""end_x.append(974.2578735351562)
end_y.append(-5575.4443359375)
conv.target_road_trajectory(974.2578735351562, -5575.4443359375, 0, 4)
print("current", conv.road_id)
print("Follow", conv.follow_id)

end_x.append(805.0078735351562)
end_y.append(-5575.51806640625)
conv.target_road_trajectory(805.0078735351562, -5575.51806640625, 0, 6)
print("current", conv.road_id)
print("Follow", conv.follow_id)"""


"""conv.target_road_trajectory(805.0078735351562, -5575.51806640625, 0, 6)
print("current", conv.road_id)
print("Follow", conv.follow_id)
conv.target_road_trajectory(805.0078735351562, -5575.51806640625, 0, 6)
print("current", conv.road_id)
print("Follow", conv.follow_id)
conv.target_road_trajectory(805.0078735351562, -5575.51806640625, 0, 6)
print("current", conv.road_id)
print("Follow", conv.follow_id)"""

''' preplanning until second target point is reached
conv.target_road_trajectory(332., -46., conv.rad_to_degree(0.001))
print("follow", conv.follow_id)

# preplanning until third target point is reached
conv.target_road_trajectory(100., -55., conv.rad_to_degree(4.712))
print("follow", conv.follow_id)'''

# visualize the whole trajectory and the reference line
# Two red points are the start and the endpoint of the first part
# of the trajectory
vis = TestVisualizer((x_start, y_start),
                     [end_x, end_y],
                     conv.waypoints[0], conv.waypoints[1],
                     conv.reference[0], conv.reference[1])
vis.visualize()


"""x_start = 384.
y_start = -0.002
x_target = 348.5
y_target = -0.0006
action = -1.571

# Initialise
conv = pt.OpenDriveConverter("town1.xodr")
conv.convert_roads()
conv.convert_junctions()
conv.filter_geometry()

# Trajectory for the starting road segment
conv.initial_road_trajectory(x_start, y_start, x_target, y_target,
                             conv.rad_to_degree(1.57),
                             conv.rad_to_degree(action),
                             4)

# preplanning until first target point is reached
conv.target_road_trajectory(x_target, y_target, conv.rad_to_degree(action),
                            4)
print("follow", conv.follow_id)

# preplanning until second target point is reached
conv.target_road_trajectory(332., -46., conv.rad_to_degree(0.001), 4)
print("follow", conv.follow_id)

# preplanning until third target point is reached
conv.target_road_trajectory(100., -55., conv.rad_to_degree(4.712), 4)
print("follow", conv.follow_id)

# visualize the whole trajectory and the reference line
# Two red points are the start and the endpoint of the first part
# of the trajectory
vis = TestVisualizer((x_start, y_start), (x_target, y_target),
                     conv.waypoints[0], conv.waypoints[1],
                     conv.reference[0], conv.reference[1])
vis.visualize()"""
