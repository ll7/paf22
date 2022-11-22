from leaderboard.autoagents.ros1_agent import ROS1Agent
from leaderboard.autoagents.autonomous_agent import Track

def get_entry_point():
    return 'PAF22Agent'


class PAF22Agent(ROS1Agent):
    def get_ros_entry_point(self):
        return {
            'package': 'agent',
            'launch_file': 'agent.launch',
            'parameters': {}
        }

    def setup(self, path_to_conf_file):
        self.track = Track.MAP
