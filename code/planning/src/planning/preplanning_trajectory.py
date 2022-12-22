from xml.etree import ElementTree as eTree
import help_functions


ROAD = -1
JUNCTION = 1
ARC = 2
LINE = 3
TARGET_DIST = 2
LEFT = -4
RIGHT = 4
U_TURN = 5
FORWARD = 0


class OpenDriveConverter:
    def __init__(self, path: str):
        # find reference to root node of xodr file
        root = eTree.parse(path).getroot()

        roads = root.findall("road")
        road_ids = [int(road.get("id")) for road in roads]
        self.roads = roads
        self.road_ids = road_ids

        junctions = root.findall("junction")
        junction_ids = [int(junction.get("id")) for junction in junctions]
        self.junctions = junctions
        self.junction_ids = junction_ids

        # All x, y coordinates for each road
        self.road_coord = None
        # hold the expected action after reaching a waypoint
        self.next_action = None
        self.follow_id = None
        self.follow_type = None
        self.follow_tag = None
        print(self.roads[11].get("name"))

    def convert_roads(self):
        # filter all road elements in a list
        # find max id to extract the list -> every road id is the correct index
        max_id = int(max(self.road_ids))
        print(max_id)
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
        print(max_id)
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
        args:
            no parameter
        return:
            :return geometry_data: A list of geometry data for each road
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading], [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m) (if value 0 -> line)
                Example of accessing the ith road, first y value:
                    geometry_data[i][1][0]
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
                for geo in geometry:
                    x.append(float(geo.get("x")))
                    y.append(float(geo.get("y")))
                    heading.append(float(geo.get("hdg")))
                    if geo[0].tag == "arc":
                        curvature.append(
                            float(geo.find("arc").get("curvature")))
                    else:
                        curvature.append(0.0)
                geometry_data.append([x, y, heading, curvature])
        assert (len(self.roads) == len(geometry_data))
        return geometry_data

    def find_current_road(self, x_curr: float, y_curr: float,
                          geometry_data):
        """ Extract the current road that fits to the x and y coordinate
        Needed to find the starting road of the agent and for every new
        waypoint we receive from Carla.

        args:
            :param x_curr: the current global x position of the agent
            :param y_curr: the current global y position of the agent
            :param geometry_data: The geometry information about all roads
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading], [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m) (if value 0 -> line)
        return:
            :return curr_road_id: The id of the current road
        """
        min_diff = 0
        diff_list = list()
        diff_index_list = list()
        # the current road index
        j = 0
        for road in geometry_data:
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
        min_diff_index = diff_list.index(min_diff)
        selected_road_id = diff_index_list[min_diff_index]
        return selected_road_id

    def create_preplanning_trajectory(self,
                                      x_curr: float, y_curr: float,
                                      x_target: float, y_target: float,
                                      next_action: float,
                                      geometry_data, current_id: int):
        """ Create a trajectory from the current position to the
        target position
        Take the extracted geometry data and the road id of the
        current position to calculate the trajectory to the
        target position. When the first Carla waypoint is
        received, this function does not contain a value for the
        next action. The first action value is set at the
        end of the function, after the agent
        reaches the first target point.

        args:
            :param x_curr: the current global x position of the agent
            :param y_curr: the current global y position of the agent
            :param geometry_data: The geometry information about all roads
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading], [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m) (if value 0 -> line)
            :param x_target: the global x value of the target position
            :param y_target: the global y value of the target position
            :param next_action: expected action after reaching a target
                    waypoint (degrees)
            :param current_id: id of the current road

        return:
            :return waypoints: list of all trajectory points to reach
                    the target
                    format: [x_points, y_points, heading, speed]
                        x_points:   list of x_values (m)
                        y_points:   list of y_values (m)
                        heading:    list of yaw values (rad)
                        speed:      list of speed limitations (m/s)
        """
        x_points = list()
        y_points = list()
        heading = list()
        speed = list()
        curr_type = None   # road or junction of initial starting road
        curr_road = None
        follow_type = None  # road or junction for following roads
        follow_tag = None  # predecessor or successor
        follow_id = None   # road id
        x_start = x_curr  # start x position of agent
        y_start = y_curr  # start y position of agent
        x_end = None  # end x position of agent
        y_end = None  # end y position of agent
        x = 0
        y = 0

        # At the first start position we do not have a next action
        # Check to follow predecessor or successor
        if self.next_action is None:
            curr_id = current_id  # id of start road
            curr_road = self.roads[curr_id]  # start road reference
            link = curr_road.find("link")
            curr_type = curr_road.get("junction")
            # Road needs a successor or predecessor
            assert (len(link) > 0)
            # if only one following road
            if len(link) == 1:
                next_road = link[0].get("elementId")
                next_road_type = link[0].get("elementType")
                next_road_tag = link[0].tag
                follow_id = next_road
                follow_type = next_road_type
                follow_tag = next_road_tag
            # predecessor and successor -> only choose which direction
            # to drive
            else:
                predecessor = curr_road.find("link").find("predecessor").\
                    get("elementId")
                successor = curr_road.find("link").find("successor").\
                    get("elementId")
                follow_id, follow_type, follow_tag = self.\
                    get_initial_next_road_id(geometry_data,
                                             int(predecessor),
                                             int(successor),
                                             x_target, y_target)
            # next waypoint for type Road
            if curr_type == ROAD:
                x_end, y_end = self.get_next_road_point(geometry_data,
                                                        follow_id)
            # next waypoint for junction depends on the chosen road
            # Kreuzung durch distanzen lösbar -> was mit leichten
            # abzweigungen?
            else:
                # was wenn ich auf einer junction starte?
                pass
        # next_action is not None
        # choose first road id based on the yaw value
        # calculate road id und next waypoint based on yaw (initial)
        # this part not reaady
        else:
            assert (curr_type != ROAD)
            curr_id = self.follow_id
            # curr_type = self.follow_type
            # curr_tag = self.follow_tag

            steering = FORWARD
            angle = heading[curr_id][0] - self.next_action
            if angle < 0:
                steering = LEFT
            elif angle > 0:
                steering = RIGHT
            if angle > 140:
                steering = U_TURN
            print(steering)
            # calculate the correct next roads out of the junction with
            # the difference in the heading

            '''if self.roads[curr_id] is None:
                self.get_next_junction_point(geometry_data, curr_id)
            else:
                new_curr_road = self.roads[curr_id]
                follow_id = new_curr_road.find("link").find(follow_tag). \
                    get("elementId")
                x_end, y_end = self.get_next_road_point(geometry_data,
                                                        follow_id)
                if self.roads[follow_id] is None:
                    follow_type = JUNCTION
                else:
                    follow_type = ROAD'''

        dist = help_functions.euclid_dist((x_start, y_start),
                                          (x_target, y_target))
        # the agent reached a carla waypoint -> next action
        # select the next road id and check to follow
        # successors or predecessors
        while dist > TARGET_DIST:
            reference_line = self.\
                check_reference_line(geometry_data[curr_id])
            if reference_line == LINE:
                x, y = help_functions.linear_interpolation(
                    (x_start, y_start),
                    (x_end, y_end))
            else:
                # fill wit values
                pass
                # x, y = help_functions.end_of_circular_arc()
                # help_functions.circular_interpolation()
            x_points.extend(x)
            y_points.extend(y)

            # declare the next road and the desired information
            x_start = x_end
            y_start = y_end
            curr_type = follow_type
            curr_id = follow_id

            # search for new endpoint -> junction or road case
            if curr_type != ROAD:
                junction_id = curr_id
                x_end, y_end, follow_id, follow_type = self.\
                    get_next_junction_point(geometry_data,
                                            junction_id, follow_tag)
            else:
                new_curr_road = self.roads[curr_id]
                follow_id = int(new_curr_road.find("link").
                                find(follow_tag).get("elementId"))
                x_end, y_end = self.get_next_road_point(geometry_data,
                                                        follow_id)
                if self.roads[follow_id] is None:
                    follow_type = JUNCTION
                else:
                    follow_type = ROAD

            # Calculate new distance to the target, to check if the target
            # position is reached
            dist = help_functions.euclid_dist((x_end, y_end),
                                              (x_target, y_target))

        self.follow_id = follow_id
        self.follow_type = follow_type
        self.follow_tag = follow_tag
        # if last calculated point is near to the target point -> directly
        # interpolate to the target point
        x, y = help_functions.linear_interpolation((x_start, x_end),
                                                   (x_target, y_target))
        x_points.extend(x)
        y_points.extend(y)
        self.next_action = next_action
        return [x_points, y_points, heading, speed]

    def check_reference_line(self, geometry_data):
        """ Check the form of the refernce line

        :param geometry_data: The geometry information about all roads
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading], [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m) (if value 0 -> line)
        :return: ARC or LINE constant
        """
        curv_data = geometry_data[3]
        if max(curv_data) > 0:
            return ARC
        return LINE

    def get_initial_next_road_id(self, geometry_data, predecessor: int,
                                 successor: int,
                                 x_target: float, y_target: float):
        """ Find the next road to drive
        When the agent starts driving it is not sure if he has to follow his
        successor or his predecessor. This function claculates the next road
        id, based on the dist to the target point. The road, who is nearer to
        this point is chosen.
        args:
            :param geometry_data: The geometry information about all roads
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading], [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m) (if value 0 -> line)
            :param predecessor: id value for predecessor road
            :param successor: id value for successor road value
            :param x_target: x position of the target waypoint
            :param y_target: y position of the target waypoint

        return:
            :return: final_id: the id value of the chosen road
        """
        final_id = 0
        final_type = 0
        final_tag = None
        x_road_p, y_road_p = self.get_next_road_point(geometry_data,
                                                      predecessor)
        x_road_s, y_road_s = self.get_next_road_point(geometry_data,
                                                      successor)

        dist_p = help_functions.\
            euclid_dist((x_road_p, y_road_p), (x_target, y_target))
        dist_s = help_functions.\
            euclid_dist((x_road_s, y_road_s), (x_target, y_target))
        if dist_p > dist_s:
            final_id = successor
            final_tag = "successor"
            # if chosen id has no road assigned -> next segment is a junction
            if self.roads[final_id] is None:
                final_type = ROAD
            else:
                final_type = JUNCTION
        else:
            final_id = predecessor
            final_tag = "predecessor"
            # if chosen id has no road assigned -> next segment is a junction
            if self.roads[final_id] is None:
                final_type = ROAD
            else:
                final_type = JUNCTION
        return final_id, final_type, final_tag

    def get_next_road_point(self, geometry_data, road_id: int):
        """ The function returns the x and y coordinate for a given road

        args:
            :param geometry_data: The geometry information about all roads
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading], [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m) (if value 0 -> line)
            :param road_id: the id value of the preferred road

        args:
            :return: x, y
                x: value of the x coordinate
                y: value of the y coordinate
        """
        # check if it is a junction
        _connect = list()
        if geometry_data[road_id] is None:
            connections = self.junctions[road_id].find("connection")
            for connect in connections:
                lanelink = connect.findall("laneLink")
                for link in lanelink:
                    if link.get("from") == link.get("to")\
                            and abs(int(link.get("from"))) == 1:
                        _connect.append(connect)

            # search through all possible connections and select the first
            # connectingRoad which is linear (if we have no command at a
            # junction
            # we pass it linear)
            for c in _connect:
                id = int(c.get("connectingRoad"))
                planView = self.roads[id].find("planView")
                if len(planView) == 1 \
                        and planView[0][0].tag == "line":
                    road_id = id
                    break

        # first x value on this road
        x = geometry_data[road_id][0][0]
        # first value on this road
        y = geometry_data[road_id][1][0]
        return x, y

    def get_next_junction_point(self, geometry_data,
                                road_id: int, follow_tag: str):
        """ Calculate the next waypoints from a junction
        In this case there is no driving command and the junction has to be
        passed straight forward.

        :param geometry_data: the geometry information about all roads
                format: [[road0], [road1],...]
                    road0:  [[x_points], [y_points], [heading],
                            [curvature]]
                        x_points:   x positions of all road segments (m)
                        y_points:   y positions of all road segments (m)
                        heading:    orientation value in rad
                        curvature:  curvature value (1/m)
                                    (if value 0 -> line)
        :param road_id: the id value of the preferred road
        :param follow_tag: the tag that indicates the next road

        :return: x, y
                x: value of the x coordinate
                y: value of the y coordinate
        """
        _connect = list()
        connections = self.junctions[road_id].find("connection")
        for connect in connections:
            lanelink = connect.findall("laneLink")
            for link in lanelink:
                if link.get("from") == link.get("to") \
                        and abs(int(link.get("from"))) == 1:
                    connect.append(connect)

        # search through all possible connections and select the first
        # connectingRoad which is linear (if we have no command at a junction
        # we pass it linear)
        for c in _connect:
            id = int(c.get("connectingRoad"))
            planView = self.roads[id].find("planView")
            if len(planView) == 1 and planView[0][0].tag == "line":
                road_id = id
                break

        follow_id = self.roads[road_id].find("link").find(follow_tag). \
            get("elementId")
        follow_type = ROAD
        # first x value on this road
        x = geometry_data[follow_id][0][0]
        # first value on this road
        y = geometry_data[follow_id][1][0]
        return x, y, follow_id, follow_type

# Visualize in 2D -> best case would be in Carla
    def visualize_trajectory_2d(self):
        pass

# Add speed limitation and steering
# Use LaneOffset -> on what lane am i driving?


conv = OpenDriveConverter("town1.xodr")
conv.convert_roads()
conv.convert_junctions()
g = conv.filter_geometry()
ind = conv.find_current_road(335.0, -323.0, g)
print(ind)
