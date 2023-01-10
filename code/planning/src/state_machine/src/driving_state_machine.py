#!/usr/bin/env python

import rospy
import smach


# define state Keep
class Keep(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['update', 'change_state_machine'],
                             output_keys=['new_target_speed'])
        self.target_speed = 0
        self.new_target_speed = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state Keep')
        if self.new_target_speed is not self.target_speed:
            userdata.new_target_speed = self.new_target_speed
            return 'update'
        else:
            return 'change_state_machine'


# define state Bar
class UpdateTargetSpeed(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['target_speed_updated',
                                       'change_state_machine'],
                             input_keys=['new_target_speed'])

    def execute(self, userdata):
        rospy.loginfo('Executing state update target speed')
        if "Intersection detected ....":
            return 'change_state_machine'
        else:
            # publish new target_speed
            return 'target_speed_updated'


def main():
    rospy.init_node('driving_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Keep', Keep(),
                               transitions={'update': 'UpdateTargetSpeed',
                                            'change_state_machine': 'outcome'})
        smach.StateMachine.add('UpdateTargetSpeed', UpdateTargetSpeed(),
                               transitions={'target_speed_updated': 'Keep',
                                            'change_state_machine': ''})

    # Execute SMACH plan
    sm.execute()


if __name__ == '__main__':
    main()
