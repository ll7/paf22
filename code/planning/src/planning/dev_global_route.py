#!/usr/bin/env python
import rospy
import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaRoute, CarlaWorldInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from ros_compatibility.qos import QoSProfile, DurabilityPolicy
import carla
from agents.navigation.global_route_planner import GlobalRoutePlanner
import xmltodict


class DevGlobalRoute(CompatibleNode):

    def __init__(self):
        super(DevGlobalRoute, self).__init__('DevGlobalRoute')

        self.sampling_resolution = self.get_param('sampling_resolution', 100.0)
        self.seq = 0    # consecutively increasing sequence ID for header_msg

        self.role_name = self.get_param("role_name", "hero")
        routes = self.get_param('routes',
                                "/opt/leaderboard/data/routes_devtest.xml")
        with open(routes, 'r', encoding='utf-8') as file:
            my_xml = file.read()

        # Use xmltodict to parse and convert the XML document
        self.routes_dict = xmltodict.parse(my_xml)
        route = self.routes_dict['routes']['route'][0]
        self.town = route['@town']
        self.waypoints = route['waypoints']['position']

        self.map_sub = self.new_subscription(
            msg_type=CarlaWorldInfo,
            topic="/carla/world_info",
            callback=self.world_info_callback,
            qos_profile=10)

        self.global_plan_pub = self.new_publisher(
            msg_type=CarlaRoute,
            topic='/carla/' + self.role_name + '/global_plan',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.global_plan_sub = self.new_subscription(
            msg_type=CarlaRoute,
            topic='/carla/' + self.role_name + '/global_plan',
            callback=self.global_route_callback,
            qos_profile=10)

        self.loginfo('DevGlobalRoute-Node started')

    def global_route_callback(self, data: CarlaRoute) -> None:
        """
        # TODO: do we need to preprocess the global route before we can use it?
        # TODO: here would be the place to do so
        :param data: global Route
        """

        # self.loginfo("NavUpdate called")
        # self.loginfo(f"nav data: {data}")

    def world_info_callback(self, data: CarlaWorldInfo) -> None:
        """
        publishes the global_route (with route instructions) according to the
        given map and waypoints of a file containing the test routes
        :param data: updated CarlaWorldInformation
        """
        self.loginfo("MapUpdate called")

        if self.town not in data.map_name:
            self.logerr(f"Map '{data.map_name}' doesnt match routes "
                        f"'{self.town}'")
            return

        # Convert data into a carla.Map
        carla_map = carla.Map(data.map_name, data.opendrive)
        # carla_map.save_to_disk(".\map.xml")
        # Create GlobalRoutePlanner
        grp = GlobalRoutePlanner(carla_map, self.sampling_resolution)

        # plan the route between given waypoints
        route_trace = []
        prepoint = self.waypoints[0]
        for waypoint in self.waypoints[1:]:
            start = carla.Location(float(prepoint['@x']),
                                   float(prepoint['@y']),
                                   float(prepoint['@z']))
            origin = carla.Location(float(waypoint['@x']),
                                    float(waypoint['@y']),
                                    float(waypoint['@z']))
            part_route_trace = grp.trace_route(start, origin)
            route_trace.extend(part_route_trace)
            prepoint = waypoint

        # parse the global route into the CarlaRoute format
        road_options = []
        poses = []
        for waypoint, road_option in route_trace:
            location = waypoint.transform.location
            position = Point(location.x, location.y, location.z)

            rotation = waypoint.transform.rotation
            quaternion = tf.transformations.quaternion_from_euler(
                rotation.roll, rotation.pitch, rotation.yaw)
            orientation = Quaternion(quaternion[0], quaternion[1],
                                     quaternion[2], quaternion[3])

            poses.append(Pose(position, orientation))
            road_options.append(road_option)

            # self.loginfo(f"{location}: {road_option}")

        self.seq += 1
        header = Header(self.seq, rospy.Time.now(), self.role_name)
        self.global_plan_pub.publish(header, road_options, poses)


if __name__ == "__main__":
    """
          main function starts the NavManager node
          :param args:
        """
    roscomp.init('DevGlobalRoute')

    try:
        DevGlobalRoute()
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
