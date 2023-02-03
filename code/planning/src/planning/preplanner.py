#!/usr/bin/env python
import rospy
import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from xml.etree import ElementTree as eTree

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from carla_msgs.msg import CarlaRoute, CarlaWorldInfo
from nav_msgs.msg import Path
from std_msgs.msg import Header
from ros_compatibility.qos import QoSProfile, DurabilityPolicy

from preplanning_trajectory import OpenDriveConverter


class PrePlanner(CompatibleNode):

    def __init__(self):
        super(PrePlanner, self).__init__('DevGlobalRoute')

        self.odc = None
        self.global_route_backup = None
        self.agent_pos = None
        self.agent_ori = None
        self.seq = 0  # consecutively increasing sequence ID for header_msg

        self.role_name = self.get_param("role_name", "hero")

        self.map_sub = self.new_subscription(
            msg_type=CarlaWorldInfo,
            topic="/carla/world_info",
            callback=self.world_info_callback,
            qos_profile=10)

        self.global_plan_sub = self.new_subscription(
            msg_type=CarlaRoute,
            topic='/carla/' + self.role_name + '/global_plan',
            callback=self.global_route_callback,
            qos_profile=10)

        self.current_pos_sub = self.new_subscription(
            msg_type=PoseStamped,
            topic="/carla/" + self.role_name + "/current_pos",
            callback=self.position_callback,
            qos_profile=1)

        self.path_pub = self.new_publisher(
            msg_type=Path,
            topic='/carla/' + self.role_name + '/trajectory',
            qos_profile=QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        )

        self.loginfo('PrePlanner-Node started')

    def global_route_callback(self, data: CarlaRoute) -> None:
        """
        when the global route gets updated a new trajectory is calculated with
        the help of OpenDriveConverter and published into
        '/carla/ self.role_name /trajectory'
        :param data: global Route
        """
        if data is None:
            self.logwarn("global_route_callback got called with None")
            return

        if self.odc is None:
            self.logerr("PrePlanner: global route got updated before map... "
                        "therefore the OpenDriveConverter couldn't be "
                        "initialised yet")
            self.global_route_backup = data
            return

        if self.agent_pos is None or self.agent_ori is None:
            self.logerr("PrePlanner: global route got updated before current "
                        "pose... therefore there is no pose to start with")
            self.global_route_backup = data
            return

        # TODO: this isnt clean... replace this
        if abs(self.agent_pos.x - data.poses[0].position.x) > 100 or \
           abs(self.agent_pos.y - data.poses[0].position.y) > 100:
            self.logerr("PrePlanner: current agent-pose doesnt match the "
                        "given global route")
            self.global_route_backup = data
            return

        self.global_route_backup = None
        x_start = self.agent_pos.x
        y_start = self.agent_pos.y
        # z_start = self.agent_pos.z
        roll_start, pitch_start, yaw_start = \
            tf.transformations.euler_from_quaternion(
             [self.agent_ori.x, self.agent_ori.y,
              self.agent_ori.z, self.agent_ori.w])

        self.loginfo(f"x_start = {x_start}")
        self.loginfo(f"y_start = {y_start}")

        x_target = data.poses[0].position.x
        y_target = data.poses[0].position.y
        # z_target = data.poses[0].position.z
        roll_target, pitch_target, yaw_target = \
            tf.transformations.euler_from_quaternion(
             [data.poses[0].orientation.x, data.poses[0].orientation.y,
              data.poses[0].orientation.z, data.poses[0].orientation.w])

        self.loginfo(f"x_target = {x_target}")
        self.loginfo(f"y_target = {y_target}")

        self.loginfo(f"Road Options: {data.road_options}")
        for i in range(20):
            x_target = data.poses[i].position.x
            y_target = data.poses[i].position.y
            self.loginfo(f"x_target = {x_target}")
            self.loginfo(f"y_target = {y_target}")
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [data.poses[i].orientation.x, data.poses[i].orientation.y,
                 data.poses[i].orientation.z, data.poses[i].orientation.w])
            self.loginfo(f"yaw = {yaw}")
            self.loginfo(f"yaw = {roll}")
            self.loginfo(f"yaw = {pitch}")
            self.loginfo("\n")

        # Trajectory for the starting road segment
        self.odc.initial_road_trajectory(x_start, y_start, x_target, y_target,
                                         yaw_start, yaw_target,
                                         data.road_options[0])

        n = len(data.poses)
        i = 1
        # iterating through global route to create trajectory
        for pose, road_option in zip(data.poses, data.road_options):
            self.loginfo(f"Preplanner going throug global plan {i}/{n}")
            i += 1
            x_target = pose.position.x
            y_target = pose.position.y
            # z_target = pose.position.z

            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [pose.orientation.x, pose.orientation.y,
                 pose.orientation.z, pose.orientation.w])
            action = yaw    # TODO: action should be road_option
            self.loginfo("road option {}".format(road_option))
            self.loginfo("Yaw {}".format(yaw))

            self.odc.target_road_trajectory(x_target, y_target,
                                            self.odc.rad_to_degree(action),
                                            road_option)

        # trajectory is now stored in the waypoints
        waypoints = self.odc.waypoints
        way_x = waypoints[0]
        way_y = waypoints[1]
        way_yaw = waypoints[2]
        # way_speed = waypoints[3]    # TODO publish max_speed as well

        # Transforming the calculated waypoints into a Path msg
        stamped_poses = []
        for i in range(len(way_x)):
            # TODO: add z, roll and pitch
            position = Point(way_x[i], way_y[i], 0)
            quaternion = tf.transformations.quaternion_from_euler(0,
                                                                  0,
                                                                  way_yaw)
            orientation = Quaternion(quaternion)
            pose = Pose(position, orientation)
            header = Header(self.seq, rospy.Time.now(), "path_pose")
            self.seq += 1
            stamped_poses.append(PoseStamped(header, pose))

        header = Header(self.seq, rospy.Time.now(), "path")
        self.seq += 1
        self.path_pub.publish(Path(header, stamped_poses))
        self.loginfo("PrePlanner: published trajectory")

    def world_info_callback(self, data: CarlaWorldInfo) -> None:
        """
        when the map gets updated a mew OpenDriveConverter instance is created
        (needed for the trajectory preplanning)
        :param data: updated CarlaWorldInformation
        """
        self.loginfo("PrePlanner: MapUpdate called")
        # Convert data into a carla.Map
        # carla_map = carla.Map(data.map_name, data.opendrive)

        root = eTree.fromstring(data.opendrive)

        roads = root.findall("road")
        road_ids = [int(road.get("id")) for road in roads]
        junctions = root.findall("junction")
        junction_ids = [int(junction.get("id")) for junction in junctions]

        self.odc = OpenDriveConverter(
            roads=roads, road_ids=road_ids,
            junctions=junctions, junction_ids=junction_ids)

        self.odc.convert_roads()
        self.odc.convert_junctions()
        self.odc.filter_geometry()

        # self.logwarn(self.odc.geometry_data[10878])
        if self.global_route_backup is not None:
            self.loginfo("PrePlanner: Received a map update retrying "
                         "route preplanning")
            self.global_route_callback(self.global_route_backup)

    def position_callback(self, data: PoseStamped):
        """
        when the position gets updated it gets stored into self.agent_pos and
        self.agent_ori
        (needed for the trajectory preplanning)
        :param data: updated CarlaWorldInformation
        """
        self.agent_pos = data.pose.position
        self.agent_ori = data.pose.orientation
        if self.global_route_backup is not None:
            self.loginfo("PrePlanner: Received a pose update retrying "
                         "route preplanning")
            self.global_route_callback(self.global_route_backup)


if __name__ == "__main__":
    """
    main function starts the PrePlanner node
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
