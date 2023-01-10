#!/usr/bin/env python

import rospy
import smach
from std_msgs.msg import Float32


# define state Keep
class Keep(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['update', 'keep_change_state_machine'],
                             output_keys=['keep_target_speed'])
        self.target_speed = 0
        self.new_target_speed = 0
        self.sub = rospy.Subscriber("target_speed", Float32, self.callback,
                                    queue_size=1)

    def callback(self, data):
        self.new_target_speed = data

    def execute(self, userdata):
        rospy.loginfo('Executing state Keep')
        self.target_speed = self.new_target_speed
        while self.new_target_speed is self.target_speed:
            print("Waiting for new target_speed")

        if self.new_target_speed is not self.target_speed:
            userdata.new_target_speed = self.new_target_speed
            return 'update'

        if False:
            return 'change_state_machine'

        return 'update'


# define state Bar
class UpdateTargetSpeed(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['target_speed_updated',
                                       'update_change_state_machine'],
                             input_keys=['update_target_speed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state update target speed')
        if "Intersection detected ....":
            return 'update_change_state_machine'
        else:
            # publish new target_speed
            return 'target_speed_updated'

        return 'target_speed_updated'


def main():
    rospy.init_node('driving_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['change_sm'])

    # Define userdata
    sm.userdata.target_speed = None

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Keep', Keep(),
                               transitions={'update': 'UpdateTargetSpeed',
                                            'keep_change_state_machine':
                                                'change_sm'},
                               remapping={'keep_target_speed': 'target_speed'})
        smach.StateMachine.add('UpdateTargetSpeed', UpdateTargetSpeed(),
                               transitions={'target_speed_updated': 'Keep',
                                            'update_change_state_machine':
                                                'change_sm'},
                               remapping={'update_target_speed':
                                                'target_speed'})

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
