import copy
from xml.etree import ElementTree as eTree
import help_functions
from typing import Tuple
from math import sin, cos, degrees


SMALL_DIST = 0.001
STEP_SIZE = 0.5
# difference to the target point
TARGET_DIFF = 5
LINE = 0.0
# Value to convert mph in m/s
MPH = 2.24
# Default speed for passing a junction in m/s
DEFAULT_MS = 12


class OpenDriveConverter:
    def __init__(self, path=None, roads=None, road_ids=None,
                 junctions=None, junction_ids=None):

        if roads is None or road_ids is None:
            self.roads, self.road_ids = self.list_xodr_properties(
                path, name="road")
        else:
            self.roads = roads
            self.road_ids = road_ids

        if junctions is None or junction_ids is None:
            self.junctions, self.junction_ids = self.list_xodr_properties(
                path, name="junction")
        else:
            self.junctions = junctions
            self.junction_ids = junction_ids

        # All x, y coordinates for each road
        self.road_coord = None
        # hold the expected action after reaching a waypoint
        self.next_action = None
        self.follow_type = None
        self.follow_tag = None
        # list that contains trajectory data
        self.waypoints = None
        self.reference = None
        # id of the current road
        self.road_id = None
        # id of the following road
        self.follow_id = None
        # id of the predecessor road
        self.old_id = None
        self.geometry_data = None
        """ A list of geometry data for each road
            format: [[road0], [road1],...]
                road0:  [[x_points], [y_points], [heading], [curvature]]
                    x_points:   x positions of all road segments (m)
                    y_points:   y positions of all road segments (m)
                    heading:    orientation value in rad
                    curvature:  curvature value (m) (if value 0 -> line)
                    length:     length of the reference line (m)
            Example of accessing the ith road, first y value:
                geometry_data[i][1][0]
        """

    def list_xodr_properties(self, path: str, name: str):
        """ Filter properties out of the xodr file
            :param path: reference to the xodr file
            :param name: name of the property to filter
            :return elements: list of the preferred elements
            :return element_ids: list of the id values for each element
        """
        # find reference to root node of xodr file
        root = eTree.parse(path).getroot()
        elements = root.findall(name)
        element_ids = [int(element.get("id")) for element in elements]
        return elements, element_ids

    def convert_roads(self):
        # filter all road elements in a list
        # find max id to extract the list -> every road id is
        # the correct index
        max_id = int(max(self.road_ids))
        # print(max_id)
        roads_extracted = list()
        j = 0
        for i in range(max_id + 1):
            if int(self.roads[j].get("id")) == i:
                roads_extracted.append(self.roads[j])
                j += 1
            else:
                roads_extracted.append(None)
        self.roads = roads_extracted

    def convert_junctions(self):
        # filter all junction elements in a list
        # find max id to extract the list -> every junction id is the
        # correct index
        max_id = int(max(self.junction_ids))
        # print(max_id)
        junctions_extracted = list()
        j = 0
        for i in range(max_id + 1):
            if int(self.junctions[j].get("id")) == i:
                junctions_extracted.append(self.junctions[j])
                j += 1
            else:
                junctions_extracted.append(None)
        self.junctions = junctions_extracted

    def filter_geometry(self):
        """ Extract all the geometry information for each road
        use the initialised roads object from the init function
        this function is only used once when the OpenDrive map is received
        """
        geometry_data = list()
        for road in self.roads:
            if road is None:
                geometry_data.append(None)
            else:
                plan_view = road.find("planView")
                geometry = plan_view.findall("geometry")
                x = list()
                y = list()
                heading = list()
                curvature = list()
                length = list()
                for geo in geometry:
                    x.append(float(geo.get("x")))
                    y.append(float(geo.get("y")))
                    heading.append(float(geo.get("hdg")))
                    length.append(float(geo.get("length")))
                    if geo[0].tag == "arc":
                        # arc values in the file have the unit 1/m
                        # we need to transform them in m
                        arc_value = float(geo.find("arc").get("curvature"))
                        radius = 1 / arc_value
                        curvature.append(radius)
                    else:
                        curvature.append(0.0)
                geometry_data.append([x, y, heading, curvature, length])
        assert (len(self.roads) == len(geometry_data))
        self.geometry_data = geometry_data

    def find_current_road(self, x_curr: float, y_curr: float):
        """ Extract the current road that fits to the x and y coordinate
        Needed to find the starting road of the agent and for every new
        waypoint we receive from Carla. Assumption that we do not start
        on a junction
            :param x_curr: the current global x position of the agent
            :param y_curr: the current global y position of the agent
            :return: curr_road_id: The id of the current road
        """
        min_diff = 0
        diff_list = list()
        diff_index_list = list()
        # the current road index
        j = 0
        for road in self.geometry_data:
            if road is None:
                j += 1
                continue
            for i in range(len(road[0])):
                x_diff = abs(road[0][i] - x_curr)
                y_diff = abs(road[1][i] - y_curr)
                diff_list.append(x_diff + y_diff)
                diff_index_list.append(j)
            j += 1
        min_diff = min(diff_list)
        print("minDiff", min_diff)
        min_diff_index = diff_list.index(min_diff)
        selected_road_id = diff_index_list[min_diff_index]
        return selected_road_id

    def initial_road_trajectory(self, x_curr: float, y_curr: float,
                                x_target: float, y_target: float):
        """ Create the trajectory on the initial road.
        The agent has to be located on the map. This case has some
        special requirements. We have to define the driving direction
        and the trajectory to the next road segment. The start case
        is treated in an extra method because this kind of planning
        is only needed at the beginning, if we are placed in
        a new town. It is assumed that the agent does not start on
        a junction
            :param x_curr: the current global x position of the agent
            :param y_curr: the current global y position of the agent
            :param x_target: x position of the target waypoint
            :param y_target: y position of the target waypoint
        """
        self.road_id = self.find_current_road(x_curr=x_curr,
                                              y_curr=y_curr)
        print("First ID ", self.road_id)
        predecessor, successor = self.get_pred_succ(road_id=self.road_id)
        print("predecessor", predecessor)
        print("successor", successor)
        self.follow_id, follow_section_id = self.\
            get_initial_next_road_id(predecessor=predecessor,
                                     successor=successor,
                                     x_target=x_target, y_target=y_target)
        if follow_section_id == predecessor:
            before_section_id = successor
        else:
            before_section_id = predecessor
        # Check if it is a junction or not
        # if it is a junction -> then the initial chosen road is correct
        before_start = None
        before_end = None
        if self.geometry_data[before_section_id] is not None:
            before_start, before_end = self.get_endpoints(before_section_id)

        # Calculate the interval between the agent and the calculated
        # road points. If x or y coordinate of the agent is in one of
        # the intervals, the current road is found.
        agent_position = (x_curr, y_curr)

        # special case when the start point is chosen on the wrong road
        # start point lay on the road before but is indicated to road
        # afterwards because of the distance measure
        if (before_end is not None) and (before_start is not None):
            if (before_start[0] < agent_position[0] < before_end[0]) or\
               (before_end[0] < agent_position[0] < before_start[0]) or\
               (before_start[1] < agent_position[1] < before_end[1]) or\
               (before_start[1] < agent_position[1] < before_end[1]):
                # agent is on the road before
                self.road_id = before_section_id
                predecessor, successor = self.get_pred_succ(
                    road_id=self.road_id)
                self.follow_id, follow_section_id = self. \
                    get_initial_next_road_id(predecessor=predecessor,
                                             successor=successor,
                                             x_target=x_target,
                                             y_target=y_target)

        follow_start, follow_end = self.get_endpoints(self.follow_id)
        if (follow_start[0] < agent_position[0] < follow_end[0]) or\
           (follow_end[0] < agent_position[0] < follow_start[0]) or\
           (follow_start[1] < agent_position[1] < follow_end[1]) or\
           (follow_start[1] < agent_position[1] < follow_end[1]):
            # agent is on the following road
            self.old_id = self.road_id
            self.road_id = self.follow_id
            self.follow_id = None

        # Interpolate the road_id
        points = self.interpolation(self.road_id)
        points = self.check_point_order(points=points,
                                        x_target=x_target,
                                        y_target=y_target)
        self.reference = copy.deepcopy(points)
        points, _a, _b, _c = self.calculate_midpoints(points,
                                                      x_curr, y_curr)
        # Find and remove the points that are not needed to pass the
        # first road from the agent position to the end of the road
        min_dist = float("inf")
        index = None
        for i in range(len(points[0])):
            x = points[0][i]
            y = points[1][i]
            point = (x, y)
            dist = help_functions.euclid_dist(point, agent_position)
            if dist < min_dist:
                min_dist = dist
                index = i
        del points[0][0:index]
        del points[1][0:index]
        del points[2][0:index]
        del points[3][0:index]
        # agent is on initial road or road before and for the next road
        # is this initial road the old road
        if self.old_id is None:
            self.old_id = self.road_id
        # agent is on follow road -> calculate the following road for it
        if self.follow_id is None:
            pred, succ = self.get_pred_succ(self.road_id)
            if pred == self.old_id:
                self.follow_id = succ
            else:
                self.follow_id = pred
            self.old_id = follow_section_id
        print("inital road: ", self.road_id)
        print("intial follow ", self.follow_id)
        self.waypoints = points

    def calculate_midpoints(self, points: list,
                            x_agent: float, y_agent: float):
        """ Calculate the trajectory points in the middle
        of a lane and return the points
            :param points: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param x_agent: last x coordinate of the agent
            :param y_agent: last y coordinate of the agent
            :return: points: same format as in the parameter,
                     widths: list of all lane width values on this road
                     size: number of width values
                     direction: True or False depending on the side
                                of the reference line
        """
        # all lane widths in this road
        # first width is for the road next to the reference line
        # and so on
        widths = self.lane_widths(self.road_id)
        size = len(widths)
        direction = self.right_or_left(points, x_agent, y_agent,
                                       widths[size - 1])
        # Calculate points in the middle of the road
        points = self.update_points(points, direction, widths[size - 1])
        return points, widths, size, direction

    def target_road_trajectory(self, x_target: float, y_target: float,
                               action: float):
        """ Calculate the trajectory to the next waypoint
        The waypoints are given by the Carla Leaderboard and the next
        action for the agent
            :param x_target: x position of the target waypoint
            :param y_target: y position of the target waypoint
            :param action: next action the agent has to take after
                           he reaches the target point
        """
        while True:
            self.road_id = self.follow_id
            predecessor, successor = self.get_pred_succ(road_id=self.road_id)
            self.follow_id, follow_section_id = self. \
                get_initial_next_road_id(predecessor=predecessor,
                                         successor=successor,
                                         x_target=x_target, y_target=y_target)
            # Interpolate the road_id
            points = self.interpolation(self.road_id)
            points = self.check_point_order(points=points,
                                            x_target=x_target,
                                            y_target=y_target)

            self.reference[0] += copy.deepcopy(points[0])
            self.reference[1] += copy.deepcopy(points[1])
            self.reference[2] += copy.deepcopy(points[2])
            self.reference[3] += copy.deepcopy(points[3])
            reference_line = points.copy()

            # all lane widths in this road
            # first width is for the road next to the reference line
            # and so on
            points, widths, size, direction = \
                self.calculate_midpoints(points, self.waypoints[0][-1],
                                         self.waypoints[1][-1])
            target = (x_target, y_target)
            # Check if points contain the target point
            min_dist = float("inf")
            index = None
            for i in range(len(points[0])):
                point = (points[0][i], points[1][i])
                dist = help_functions.euclid_dist(point, target)
                if dist < min_dist:
                    min_dist = dist
                    index = i
            # If min_dist is smaller than 8m, the target point is reached
            # delete points bigger than the target point and calculate
            # last points new based on the target lane for the next action
            if min_dist <= TARGET_DIFF:
                first_widths = []
                min_width = []
                self.follow_id = self.next_action_id(action,
                                                     follow_section_id,
                                                     points)
                point1_x = reference_line[0][-2]
                point1_y = reference_line[1][-2]
                point2_x = reference_line[0][-1]
                point2_y = reference_line[1][-1]
                point3_x = reference_line[0][-3]
                point3_y = reference_line[1][-3]
                start, end = self.get_endpoints(self.follow_id)
                # Choose the startpoint of the follow road
                dist1 = help_functions.euclid_dist(start,
                                                   (point2_x, point2_y))
                dist2 = help_functions.euclid_dist(end,
                                                   (point2_x, point2_y))
                # choose the point of the follow road which is nearer
                # to the last trajectory point
                if dist1 < dist2:
                    ref_point = start
                else:
                    ref_point = end
                # Iterate over all width values and calculate for each value
                # the midpoint on the lane -> choose the nearest midpoint
                # to the follow road start point
                min_dist = float("inf")
                for width in widths:
                    point, v = self.update_one_point(point1_x, point1_y,
                                                     point2_x, point2_y,
                                                     point3_x, point3_y,
                                                     direction, width)
                    dist = help_functions.euclid_dist(ref_point, point)
                    # Choose the width value for the correct lane
                    if dist < min_dist:
                        min_dist = dist
                        min_width = width
                # if we need to change the lane for the next action
                # we want a smooth change in the trajectory points
                points = reference_line
                last_width = widths[size-1]
                diff = abs(min_width - last_width)
                step_size = STEP_SIZE
                i = 0
                # if diff != 0 we need to change the lane and calculate
                # a smooth change
                if diff != 0:
                    steps = int(diff / step_size)
                    first_widths = [last_width - step_size * i
                                    for i in range(steps)]
                    for i in range(len(first_widths)):
                        p1_x = points[0][i]
                        p1_y = points[1][i]
                        p2_x = points[0][i+1]
                        p2_y = points[1][i+1]
                        if i != len((points[0])) - 2:
                            p3_x = points[0][i + 2]
                            p3_y = points[1][i + 2]
                        point, v = self.update_one_point(p1_x, p1_y,
                                                         p2_x, p2_y,
                                                         p3_x, p3_y,
                                                         direction,
                                                         first_widths[i])
                        points[0][i] = point[0]
                        points[1][i] = point[1]
                    p = [points[0][i+1:], points[1][i+1:], points[2][i+1:],
                         points[3][i+1:]]
                    points = self.update_points(p, direction, min_width)
                del points[0][index+1:]
                del points[1][index+1:]
                del points[2][index+1:]
                del points[3][index+1:]
                self.add_waypoints(points)
                break
            self.add_waypoints(points)

    def rad_to_degree(self, radians):
        """ Convert radians value to degrees
            :param radians: heading value in rad
            :return: deg: degree value
        """
        radians = abs(radians)
        deg = degrees(radians)
        if deg > 360:
            deg = 360 - degrees(radians)
        return deg

    def next_action_id(self, action: float, sec_id: int, points: list):
        """ Calculate the next road id for the given action from
        the leaderboard
        :param action: yaw value for the new global heading of the agent
        :param sec_id: section id of the road or the junction
        :param points: points: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        :return: action_id: the id of the next road to take
        """
        action_id = None
        current_road = self.roads[sec_id]
        # if current road is not a junction, the follow_road is needed
        # to calculate the new action road
        if current_road is None:
            junction = sec_id
            incoming_road = self.road_id
            possible_road_ids = self.filter_road_ids(junction,
                                                     incoming_road)
            last_point_x = points[0][-1]
            last_point_y = points[1][-1]
            last_point = (last_point_x, last_point_y)
            action_id = self.calculate_action_id(possible_road_ids,
                                                 last_point,
                                                 action)
        else:
            action_id = self.road_id
        return action_id

    def calculate_action_id(self, possible_road_ids: list,
                            last_point: Tuple[float, float],
                            action: float):
        """ Calculate the next road to take from the junction based
        on the next action from the leaderboard
            :param possible_road_ids: list of the next possible road ids
            :param last_point: last calculated point of the trajectory
            :param action: the next action to take (global heading)
            :return: possible_road_ids: the id of the next road
        """
        yaw_values = list()
        # find the hdg values for the endpoints of the possible roads
        for road_id in possible_road_ids:
            start, end = self.get_endpoints(road_id)
            dist_start = help_functions.euclid_dist(start, last_point)
            dist_end = help_functions.euclid_dist(end, last_point)
            if dist_start < dist_end:
                hdg = self.geometry_data[road_id][2][-1]
                hdg = self.rad_to_degree(hdg)
                yaw_values.append(hdg)
            else:
                hdg = self.geometry_data[road_id][2][0]
                hdg = self.rad_to_degree(hdg)
                yaw_values.append(hdg)
        min_diff = float("inf")
        yaw = None
        for hdg in yaw_values:
            diff = abs(hdg - action)
            if diff < min_diff:
                min_diff = diff
                yaw = hdg
        index = yaw_values.index(yaw)
        return possible_road_ids[index]

    def filter_road_ids(self, junction: int, incoming: int):
        """ Filter the road id values of all connecting roads
        that are linked to the incoming road
            :param junction: id value of the junction
            :param incoming: id value of the incoming road
            :return: road_ids: list of the connecting road ids
        """
        j = self.junctions[junction]
        jun = j.findall("connection")
        roads = [j for j in jun if j.get("incomingRoad") == str(incoming)]
        road_ids = [int(r.get("connectingRoad")) for r in roads]
        return road_ids

    def lane_widths(self, road_id: int):
        """ Filter all lane width values from a given road
            :param road_id: the id value of the examined road
            :return: widths: list of all width values
        """
        road = self.roads[road_id]
        lanes = road.find("lanes")
        direction = lanes.find("laneSection").find("right")
        if direction is None:
            direction = lanes.find("laneSection").find("left")
        lanes = direction.findall("lane")
        driving = [lane for lane in lanes if lane.get("type") == "driving"]
        width = driving[0].find("width").get("a")
        # agent drives in the middle of the street
        width = float(width) / 2.0
        widths = [width*i for i in range(1, len(driving)+1)]
        return widths

    def right_or_left(self, points: list, x_agent: float, y_agent: float,
                      width: float):
        """ Define on which side of the reference line the trajectory
        is running. If it returns true the update point function
        will choose the correct function for the update of the
        trajectory points
            :param points: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param x_agent: current x position of the agent
            :param y_agent: current y position of the agent
            :param width: width of the lane
            :return: direction: True or False if the agent is on the right
                                side of the reference line or not
        """
        direction = True
        x_start = points[0][0]
        y_start = points[1][0]
        x_follow = points[0][1]
        y_follow = points[1][1]
        point_x = points[0][2]
        point_y = points[1][2]
        point1, v = self.update_one_point(x_start, y_start,
                                          x_follow, y_follow,
                                          point_x, point_y,
                                          True, width)
        point2, v = self.update_one_point(x_start, y_start,
                                          x_follow, y_follow,
                                          point_x, point_y,
                                          False, width)
        agent = (x_agent, y_agent)
        dist_1 = help_functions.euclid_dist(point1, agent)
        dist_2 = help_functions.euclid_dist(point2, agent)
        if dist_1 > dist_2:
            direction = False
        return direction

    def update_one_point(self, point1_x: float, point1_y: float,
                         point2_x: float, point2_y: float,
                         point3_x: float, point3_y: float,
                         right: bool, width: float):
        """ Update the coordinates of a point width the given
        width value for the correct lane
            :param point1_x: x coordinate of the point to update
            :param point1_y: y coordinate of the point to update
            :param point2_x: x coordinate of the following point
            :param point2_y: y coordinate of the following point
            :param point3_x: x coordinate of the next following point
            :param point3_y: y coordiante of the next following point
            :param right: True or False for the correct side of the lane
            :param width: value for the width of the lane
            :return: point: coordinates of the updated point
                     vector: last added vector
        """
        first = (point1_x, point1_y)
        second = (point2_x, point2_y)
        vector = help_functions.sub_vector(second, first)
        # if dist to the next point is to small -> outliners
        if vector[0] < SMALL_DIST and vector[1] < SMALL_DIST:
            vector = help_functions.sub_vector((point3_x, point3_y), first)
        if right is True:
            vec = help_functions.perpendicular_vector_right(vector)
        else:
            vec = help_functions.perpendicular_vector_left(vector)
        vector = help_functions.unit_vector(vec, width)
        point = help_functions.add_vector(first, vector)
        return point, vector

    def update_points(self, p_list: list, right: bool, width: float):
        """ Update the coordinates of a point list width the given
        width value for the correct lane
            :param p_list: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param right: True or False if the agent is on the right
                          side of the reference line or not
            :param width: width of the lane
            :return: p_list: point list of the format from the parameter
        """
        i = 0
        for i in range(len(p_list[0]) - 1):
            point1_x = p_list[0][i]
            point1_y = p_list[1][i]
            point2_x = p_list[0][i + 1]
            point2_y = p_list[1][i + 1]
            # point if dist is too small
            if i != len((p_list[0])) - 2:
                point3_x = p_list[0][i + 2]
                point3_y = p_list[1][i + 2]
            point, vector = self.update_one_point(point1_x, point1_y,
                                                  point2_x, point2_y,
                                                  point3_x, point3_y,
                                                  right, width)
            p_list[0][i] = point[0]
            p_list[1][i] = point[1]
        point = (p_list[0][i + 1], p_list[1][i + 1])
        point = help_functions.add_vector(point, vector)
        p_list[0][i + 1] = point[0]
        p_list[1][i + 1] = point[1]
        return p_list

    def add_waypoints(self, points: list):
        """ Add calculated points to the trajectory list
            :param points: list of all trajectory points
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        """
        x = points[0].copy()
        y = points[1].copy()
        hdg = points[2].copy()
        speed = points[3].copy()
        self.waypoints[0] += x
        self.waypoints[1] += y
        self.waypoints[2] += hdg
        self.waypoints[3] += speed

    def check_point_order(self, points: list, x_target: float,
                          y_target: float):
        """ Check if the trajectory points have the correct order
            :param points: list of all trajectory points
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
            :param x_target: x coordinate of the target point
            :param y_target: y coordinate of the target point
            :return: points: same format as the parameter points
        """
        target = (x_target, y_target)
        start_x = points[0][0]
        start_y = points[1][0]
        end_x = points[0][-1]
        end_y = points[1][-1]
        start = (start_x, start_y)
        end = (end_x, end_y)
        dist_start = help_functions.euclid_dist(start, target)
        dist_end = help_functions.euclid_dist(end, target)
        # if start distance is smaller, this point is nearer to the target,
        # but on the first list entry -> sort list
        if dist_start < dist_end:
            x = points[0][::-1]
            y = points[1][::-1]
            yaw = points[2][::-1]
            speed = points[3][::-1]
            points = [x, y, yaw, speed]
        return points

    def get_speed(self, road_id: int):
        """ Filter and calculate the max_speed for the road
            :param road_id: id value for the road
            :return: speed: speed value for the road in m/s
        """
        road = self.roads[road_id]
        if road.get("junction") == "-1":
            speed = road.find("type").find("speed")
            speed = float(speed.get("max")) / MPH
        else:
            speed = DEFAULT_MS
        return speed

    def interpolation(self, road_id: int):
        """ Interpolate over a complete road
            :param road_id: id value of the current road
            :return: waypoints: list of all trajectory points to reach
                    the following road
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        """
        x = list()
        y = list()
        yaw = list()
        speed = list()
        size = len(self.geometry_data[road_id][0])
        max_speed = self.get_speed(road_id)
        for i in range(size):
            x_start = self.geometry_data[road_id][0][i]
            y_start = self.geometry_data[road_id][1][i]
            start = (x_start, y_start)
            length = self.geometry_data[road_id][4][i]
            hdg = self.geometry_data[road_id][2][i]
            if self.geometry_data[road_id][3][i] == LINE:
                xd = length * cos(hdg)
                yd = length * sin(hdg)
                end = help_functions.add_vector(start, (xd, yd))
                points = help_functions.linear_interpolation(
                    start=start,
                    end=end,
                    interval_m=1)
            else:
                radius = self.geometry_data[road_id][3][i]
                end = help_functions.end_of_circular_arc(
                    start_point=start, angle=hdg,
                    length=length, radius=radius)
                points = help_functions.circular_interpolation(
                    start=start,
                    end=end,
                    arc_radius=radius)
            for j in range(len(points)):
                x.append(points[j][0])
                y.append(points[j][1])
                yaw.append(hdg)
                speed.append(max_speed)
        return [x, y, yaw, speed]

    def get_endpoints(self, road_id: int):
        """ Calculate the startpoint and endpoint of a given road
            :param road_id: the road id of the examined road
            :return: start_point, end_point
                start_point: x and y coordinate of the starting point
                end_point: x and y coordinate of the ending point
        """
        size = len(self.geometry_data[road_id][0])
        x_start = self.geometry_data[road_id][0][0]
        y_start = self.geometry_data[road_id][1][0]
        start_point = (x_start, y_start)

        length = self.geometry_data[road_id][4][size - 1]
        hdg = self.geometry_data[road_id][2][size - 1]
        # calculate the start point of the last part of the road
        # to calculate the endpoint of the whole road
        x = self.geometry_data[road_id][0][size - 1]
        y = self.geometry_data[road_id][1][size - 1]
        last_start = (x, y)

        # check the last curvature value to see if it is line or arc
        if self.geometry_data[road_id][3][size-1] == LINE:
            xd = length * cos(hdg)
            yd = length * sin(hdg)
            end_point = help_functions.add_vector(last_start, (xd, yd))
        else:
            radius = self.geometry_data[road_id][3][size-1]
            end_point = help_functions.end_of_circular_arc(
                start_point=last_start, angle=hdg,
                length=length, radius=radius)
        return start_point, end_point

    def get_initial_next_road_id(self, predecessor: int,
                                 successor: int,
                                 x_target: float, y_target: float):
        """ Find the next road to drive
        When the agent starts driving it is not sure if he has to follow his
        successor or his predecessor. This function calculates the next road
        id, based on the dist to the target point. The road, who is nearer to
        this point is chosen
            :param predecessor: id value for predecessor road
            :param successor: id value for successor road value
            :param x_target: x position of the target waypoint
            :param y_target: y position of the target waypoint
            :return: final_id: the id value of the chosen road
                     section_id: the id value of the chosen predecessor
                                 or successor
        """
        # value of predecessor or successor (can also be a junction id)
        section_id = None

        # When succ or pred are a junction than i should calculate
        # the next id
        if predecessor is None:
            final_id = successor
        elif successor is None:
            final_id = predecessor
        else:
            x_road_p, y_road_p, pred = self.\
                get_next_road_point(predecessor)
            x_road_s, y_road_s, succ = self.\
                get_next_road_point(successor)

            dist_p = help_functions.\
                euclid_dist((x_road_p, y_road_p), (x_target, y_target))
            dist_s = help_functions.\
                euclid_dist((x_road_s, y_road_s), (x_target, y_target))
            if dist_p > dist_s:
                final_id = succ
                section_id = successor
            else:
                final_id = pred
                section_id = predecessor
        return final_id, section_id

    def get_pred_succ(self, road_id: int):
        """ Find the predecessor and the successor road of the current road
        If there is only a successor or only a predecessor, this
        function handles these cases
            :param road_id: id of the current road
            :return: predecessor: road id of the predecessor or None
                                  if there is no predecessor
                     successor: road id of the successor or None
                                if there is no successor
        """
        # start road reference
        curr_road = self.roads[road_id]
        link = curr_road.find("link")
        # curr_type = curr_road.get("junction")
        # Road needs a successor or predecessor
        assert (len(link) > 0)
        # if only one following road
        if len(link) == 1:
            next_road_id = link[0].get("elementId")
            next_road_tag = link[0].tag
            if next_road_tag == "predecessor":
                predecessor = int(next_road_id)
                successor = None
            else:
                predecessor = None
                successor = int(next_road_id)
        # predecessor and successor -> only choose which direction
        # to drive
        else:
            predecessor = int(curr_road.find("link").find("predecessor").
                              get("elementId"))
            successor = int(curr_road.find("link").find("successor").
                            get("elementId"))
        return predecessor, successor

    def get_next_road_point(self, road_id: int):
        """ The function returns the x and y coordinate for a given road
            :param road_id: the id value of the preferred road
            :return: x, y, road_id
                x: value of the x coordinate
                y: value of the y coordinate
                road_id: id of the chosen road
        """
        # check if it is a junction
        if self.geometry_data[road_id] is None:
            connections = self.junctions[road_id].findall("connection")
            # search through all possible connections and select the first
            # connectingRoad which is linear (if we have no command at a
            # junction we pass it linear)
            for c in connections:
                # line indicates if the whole road is of the type line
                line = True
                id = int(c.get("connectingRoad"))
                road_data = self.geometry_data[id]
                for i in range(len(road_data[0])):
                    if road_data[3][i] != 0.0:
                        line = False
                        break
                if line is True:
                    road_id = id
                    break
        # first x value on this road
        x = self.geometry_data[road_id][0][0]
        # first value on this road
        y = self.geometry_data[road_id][1][0]
        return x, y, road_id
