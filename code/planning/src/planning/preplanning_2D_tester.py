import preplanning_trajectory as pt
from visualizer import TestVisualizer


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

x_start = 983.5
y_start = -5373.2
# muss zu beginn punkt eingeben der ein turn commando
# ist erster Punkt mit lane change info -> target nächster punkt mit turn
# filter vorab ob punkt vor aktuellem gps punkt liegt
# wenn ja dann passt, wenn nein dann kurz hinter gps punkt lane change
x_target = 983.2598876953125
y_target = -5568.28662109375
# Trajectory for the starting road segment
end_x.append(x_target)
end_y.append(y_target)
# target must be a turn command point to choose the correct direction
conv.initial_road_trajectory(x_start, y_start, x_target, y_target,
                             974.2578735351562, -5575.4443359375,
                             983.5927734375, -5381.88720703125,
                             -90, 4)
# preplanning until first target point is reached
print()

end_x.append(983.2598876953125)
end_y.append(-5568.28662109375)
conv.target_road_trajectory(983.2598876953125, -5568.28662109375,
                            974.2578735351562, -5575.4443359375, 2)
print("current", conv.road_id)
print("Follow", conv.follow_id)
print()

end_x.append(974.2578735351562)
end_y.append(-5575.4443359375)
conv.target_road_trajectory(974.2578735351562, -5575.4443359375,
                            805.0078735351562, -5575.51806640625, 4)
print("current", conv.road_id)
print("Follow", conv.follow_id)
print("Hallo")
print()


end_x.append(805.0078735351562)
end_y.append(-5575.51806640625)
conv.target_road_trajectory(805.0078735351562, -5575.51806640625,
                            780.8638305664062, -5575.548828125, 3)
print("current", conv.road_id)
print("Follow", conv.follow_id)
print()

end_x.append(780.8638305664062)
end_y.append(-5575.548828125)
conv.target_road_trajectory(780.8638305664062, -5575.548828125,
                            605.5011596679688, -5575.97021484375, 4)
print("current", conv.road_id)
print("Follow", conv.follow_id)
print()

end_x.append(605.5011596679688)
end_y.append(-5575.97021484375)
conv.target_road_trajectory(605.5011596679688, -5575.97021484375,
                            600.8721923828125, -5579.2314453125, 5)
print("current", conv.road_id)
print("Follow", conv.follow_id)
print()

end_x.append(600.8721923828125)
end_y.append(-5579.2314453125)
# conv.target_road_trajectory(600.8721923828125, -5579.2314453125,
#                           528.1724853515625, -5579.40625, 5)
print("current", conv.road_id)
print("Follow", conv.follow_id)

"""conv.target_road_trajectory(805.0078735351562, -5575.51806640625, 0, 6)
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
