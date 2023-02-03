import preplanning_trajectory as pt
from visualizer import TestVisualizer


end_x = list()
end_y = list()

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
# x_first and y_first None if first point is turn command
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
# print("Follow", conv.follow_id)
print()

end_x.append(974.2578735351562)
end_y.append(-5575.4443359375)
conv.target_road_trajectory(974.2578735351562, -5575.4443359375,
                            805.0078735351562, -5575.51806640625, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()


end_x.append(805.0078735351562)
end_y.append(-5575.51806640625)
conv.target_road_trajectory(805.0078735351562, -5575.51806640625,
                            780.8638305664062, -5575.548828125, 3)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(780.8638305664062)
end_y.append(-5575.548828125)
conv.target_road_trajectory(780.8638305664062, -5575.548828125,
                            605.5011596679688, -5575.97021484375, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(605.5011596679688)
end_y.append(-5575.97021484375)
conv.target_road_trajectory(605.5011596679688, -5575.97021484375,
                            600.8721923828125, -5579.2314453125, 5)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(600.8721923828125)
end_y.append(-5579.2314453125)
conv.target_road_trajectory(600.8721923828125, -5579.2314453125,
                            528.1724853515625, -5579.40625, 5)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(528.1724853515625)
end_y.append(-5579.40625)
conv.target_road_trajectory(528.1724853515625, -5579.40625,
                            519.0783081054688, -5593.16650390625, 1)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(519.0783081054688)
end_y.append(-5593.16650390625)
conv.target_road_trajectory(519.0783081054688, -5593.16650390625,
                            483.8909912109375, -5791.52587890625, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(483.8909912109375)
end_y.append(-5791.52587890625)
conv.target_road_trajectory(483.8909912109375, -5791.52587890625,
                            488.34564208984375, -5833.22021484375, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(488.34564208984375)
end_y.append(-5833.22021484375)
conv.target_road_trajectory(488.3456420898437515625, -5833.22021484375,
                            492.93499755859375, -5849.07861328125, 3)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(492.93499755859375)
end_y.append(-5849.07861328125)
conv.target_road_trajectory(492.93499755859375, -5849.07861328125,
                            554.9659423828125, -6026.10888671875, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(554.9659423828125)
end_y.append(-6026.10888671875)
conv.target_road_trajectory(554.9659423828125, -6026.10888671875,
                            550.3844604492188, -6034.73193359375, 2)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(550.3844604492188)
end_y.append(-6034.73193359375)
conv.target_road_trajectory(550.3844604492188, -6034.73193359375,
                            522.0089111328125, -6042.43603515625, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(522.0089111328125)
end_y.append(-6042.43603515625)
conv.target_road_trajectory(522.0089111328125, -6042.43603515625,
                            456.49993896484375, -6069.91748046875, 3)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(456.49993896484375)
end_y.append(-6069.91748046875)
conv.target_road_trajectory(456.49993896484375, -6069.91748046875,
                            309.1519470214844, -6170.18603515625, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(309.1519470214844)
end_y.append(-6170.18603515625)
conv.target_road_trajectory(309.1519470214844, -6170.18603515625,
                            299.12628173828125, -6166.791015625, 2)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(299.12628173828125)
end_y.append(-6166.791015625)
conv.target_road_trajectory(299.12628173828125, -6166.791015625,
                            280.88623046875, -6108.14501953125, 4)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()

end_x.append(280.88623046875)
end_y.append(-6108.14501953125)
conv.target_road_trajectory(280.88623046875, -6108.14501953125,
                            268.74639892578125, -6100.86669921875, 1)
print("current", conv.road_id)
# print("Follow", conv.follow_id)
print()


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
