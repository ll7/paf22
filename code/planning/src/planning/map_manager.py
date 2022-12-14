#!/usr/bin/env python
import rospy
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from std_msgs.msg import String
# import lxml.etree as et
# import carla


class MapManager(CompatibleNode):

    def __init__(self):
        super(MapManager, self).__init__('MapManager')
        self.loginfo('MapManager node started')
        self.new_subscription(msg_type=String, topic='/carla/hero/OpenDRIVE',
                              callback=map_update, qos_profile=10)
        rospy.Subscriber(name='/carla/hero/OpenDRIVE', data_class=String,
                         callback=map_update, queue_size=10)


def map_update(opendrive):
    """_summary_

    :param opendrive:
    :return: _description_
    """

    rospy.loginfo("Map update recieved!")
    # connecting a client (might be obsolete OR Cheating because of
    # privilege information)
    # client = carla.Client('localhost', 2000)
    # world = client.get_world()  # .load_world('Town01')

    # retrieve Map object from opendrive
    # map = opendrive.get_map()
    # map2 = world.get_map()
    # waypoint_tuple_list = map.get_topology()
    # info_map = map.to_opendrive()

    f = open("opendrivemap.xodr", "a")
    f.write(opendrive)
    f.close()
    rospy.loginfo("map saved")

    """
    # Load the xodr file using lxml
    tree = et.parse('path/to/file.xodr')
    root = tree.getroot()

    # Extract the road information from the xodr file
    roads = root.findall('.//road')

    # Iterate over the roads in the xodr file
    for road in roads:
        # Extract the relevant information for each road
        id = road.get('id')
        name = road.find('name').text

        # Extract the geometry information for the road
        geometry = road.find('planView')
        points = geometry.findall('geometry')

        # Create an empty list to store the Waypoint objects for the road's
        # route
        waypoints = []

        # Iterate over the points in the road's geometry
        for point in points:
            # Extract the global coordinates for each point
            x = point.get('x')
            y = point.get('y')
            z = point.get('z')
            h = point.get('hdg')
            p = point.get('pitch')
            r = point.get('roll')

            # Create a Waypoint object for the current point
            waypoint = world.get_map().get_waypoint(carla.Location(x, y, z),
                                                    project_to_road=True)

            # Set the orientation for the Waypoint object
            waypoint.transform.rotation.yaw = h
            waypoint.transform.rotation.pitch = p
            waypoint.transform.rotation.roll = r

            # Add the Waypoint object to the list of waypoints for the road's
            # route
            waypoints.append(waypoint)

        # Use the debug.draw_waypoints method to visualize the road's route in
        # Carla
        world.debug.draw_waypoints(waypoints, waypoint_size=0.5, width=0.5,
                                   life_time=10.0)

    pass """


if __name__ == "__main__":
    """
          main function starts the MapManager node
          :param args:
        """
    roscomp.init('MapManager')

    try:
        node = MapManager()
        # node.run()
        while True:
            pass
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()
