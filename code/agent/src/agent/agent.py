from leaderboard.autoagents.ros1_agent import ROS1Agent
from leaderboard.autoagents.autonomous_agent import Track


def get_entry_point():
    return 'PAF22Agent'


class PAF22Agent(ROS1Agent):

    def setup(self, path_to_conf_file):
        self.track = Track.MAP

    def get_ros_entrypoint(self):
        return {
            'package': 'agent',
            'launch_file': 'agent.launch',
            'parameters': {
                'role_name': 'hero',
            }
        }

    def sensors(self):
        sensors = [
            {'type': 'sensor.camera.rgb', 'id': 'Center',
             'x': 0.7, 'y': 0.0, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': 0.0, 'width': 300, 'height': 200, 'fov': 100},
            {'type': 'sensor.lidar.ray_cast', 'id': 'LIDAR',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0},
            {'type': 'sensor.other.radar', 'id': 'RADAR',
             'x': 0.7, 'y': -0.4, 'z': 1.60, 'roll': 0.0, 'pitch': 0.0,
             'yaw': -45.0, 'horizontal_fov': 30, 'vertical_fov': 30},
            {'type': 'sensor.other.gnss', 'id': 'GPS',
             'x': 0.7, 'y': -0.4, 'z': 1.60},
            {'type': 'sensor.other.imu', 'id': 'IMU',
             'x': 0.0, 'y': 0.0, 'z': 0.0, 'roll': 0.0, 'pitch': 0.0,
             'yaw': 0.0},
            {'type': 'sensor.opendrive_map', 'id': 'OpenDRIVE',
             'reading_frequency': 1},
            {'type': 'sensor.speedometer', 'id': 'Speed'},
        ]
        return sensors

    def destroy(self):
        super(PAF22Agent, self).destroy()
