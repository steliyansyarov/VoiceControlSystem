#!/usr/bin/env python2
import time
import json
import rospy
import smach
# import smach_ros
from std_msgs.msg import String

ROS_NODE_NAME = "smach_state_machine"
ROS_SUBSCRIBER_TOPIC = "parsed_text"
intent_name = None


class IdleState(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move', 'bring', 'pickUp', 'putDown', 'aborted'])

    def execute(self, userdata):
        global intent_name
        while True:
            if not rospy.is_shutdown():
                if intent_name == "moveAction":
                    intent_name = None
                    return 'move'
                if intent_name == "bringAction":
                    intent_name = None
                    return 'bring'
                if intent_name == "pickUpAction":
                    intent_name = None
                    return 'pickUp'
                if intent_name == "putDownAction":
                    intent_name = None
                    return 'putDown'
            else:
                break
        return 'aborted'


class MoveActionSM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'aborted', 'failed'])

    def execute(self, userdata):
        time.sleep(5)
        # Publish action
        return 'finished'


class BringActionSM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'aborted', 'failed'])

    def execute(self, userdata):
        time.sleep(5)
        # Publish action
        return 'finished'


class PickUpActionSM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'aborted', 'failed'])

    def execute(self, userdata):
        time.sleep(5)
        # Publish action
        return 'finished'


class PutDownActionSM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished', 'aborted', 'failed'])

    def execute(self, userdata):
        time.sleep(5)
        # Publish action
        return 'finished'


def callback(msg):
    if not rospy.is_shutdown():
        global intent_name
        # Currently, only intentName is relevant, the rest of the json is not used
        intent_name = json.loads(msg.data)["intent"]["intentName"]
        print(intent_name)


def main():
    # Initialize Ros node and the Topic subscriber
    rospy.init_node(ROS_NODE_NAME)
    rospy.Subscriber(str(ROS_SUBSCRIBER_TOPIC), String, callback)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['aborted'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Idle', IdleState(),
                               transitions={
                                   'move': 'MoveAction',
                                   'bring': 'BringAction',
                                   'pickUp': 'PickUpAction',
                                   'putDown': 'PutDownAction',
                                   'aborted': 'aborted'})

        smach.StateMachine.add('MoveAction', MoveActionSM(),
                               transitions={
                                   'finished': 'Idle',
                                   'aborted': 'Idle',
                                   'failed': 'Idle'})

        smach.StateMachine.add('BringAction', BringActionSM(),
                               transitions={
                                   'finished': 'Idle',
                                   'aborted': 'Idle',
                                   'failed': 'Idle'})

        smach.StateMachine.add('PickUpAction', PickUpActionSM(),
                               transitions={
                                   'finished': 'Idle',
                                   'aborted': 'Idle',
                                   'failed': 'Idle'})

        smach.StateMachine.add('PutDownAction', PutDownActionSM(),
                               transitions={
                                   'finished': 'Idle',
                                   'aborted': 'Idle',
                                   'failed': 'Idle'})

    # Create and start the introspection server
    # sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    # sis.start()

    print("ROS node '%s' started. Listening from '%s' (ctrl-C to exit)..." % (ROS_NODE_NAME, ROS_SUBSCRIBER_TOPIC))

    # Execute the state machine
    outcome = sm.execute()

    rospy.spin()
    # sis.stop()
    print("Ctrl-C received. Shutting down ROS node '%s'!" % ROS_NODE_NAME)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
