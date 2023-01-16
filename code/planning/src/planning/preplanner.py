#!/usr/bin/env python
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from carla_msgs.msg import CarlaRoute, CarlaWorldInfo
from xml.etree import ElementTree as eTree
from preplanning_trajectory import OpenDriveConverter


class PrePlanner(CompatibleNode):

    def __init__(self):
        super(PrePlanner, self).__init__('DevGlobalRoute')

        self.odc = None

        self.map_sub = self.new_subscription(
            msg_type=CarlaWorldInfo,
            topic="/carla/world_info",
            callback=self.world_info_callback,
            qos_profile=10)

        self.global_plan_sub = self.new_subscription(
            msg_type=CarlaRoute,
            topic='/carla/hero/global_plan',
            callback=self.global_route_callback,
            qos_profile=10)

        self.loginfo('PrePlanner-Node started')

    def global_route_callback(self, data: CarlaRoute) -> None:
        """
        # TODO:
        :param data: global Route
        """
        # TODO: check if self.odc got init

        # TODO: delete the following line (just to run the linter)
        x_start = y_start = x_target = y_target = action = None

        # Trajectory for the starting road segment
        # TODO: receive current x, y, (z) of agent to start with
        self.odc.initial_road_trajectory(x_start, y_start, x_target, y_target)
        for pose, road_option in zip(data.poses, data.road_options):
            # preplanning until first target point is reached
            # TODO expected action is a float but in reality its a road_option
            # TODO: extract x,y,(z) from pose
            self.odc.target_road_trajectory(x_target, y_target,
                                            self.odc.rad_to_degree(action))

        # TODO: transform calculated trajectory into nav_msgs.msg Path
        # TODO: publish trajectory in "/carla/ self.role_name /trajectory"

    def world_info_callback(self, data: CarlaWorldInfo) -> None:
        """
        TODO:
        :param data: updated CarlaWorldInformation
        """
        self.loginfo("MapUpdate called")
        # Convert data into a carla.Map
        # carla_map = carla.Map(data.map_name, data.opendrive)

        root = eTree.fromstring(data.opendrive)

        roads = root.findall("road")
        road_ids = [int(road.get("id")) for road in self.roads]
        junctions = root.findall("junction")
        junction_ids = [int(junction.get("id")) for junction in self.junctions]

        self.odc = OpenDriveConverter(
            roads=roads, road_ids=road_ids,
            junctions=junctions, junction_ids=junction_ids)

        self.odc.convert_roads()
        self.odc.convert_junctions()
        self.odc.filter_geometry()


if __name__ == "__main__":
    """
          main function starts the NavManager node
          :param args:
        """
    roscomp.init('PrePlanner')

    try:
        PrePlanner()
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
