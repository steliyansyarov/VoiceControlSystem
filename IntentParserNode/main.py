from __future__ import unicode_literals, print_function

import rospy
import json
from std_msgs.msg import String
from snips_nlu import SnipsNLUEngine
from snips_nlu.default_configs import CONFIG_EN

DATASET_PATH = "robot_commands.json"
ROS_NODE_NAME = "snips_parser"
ROS_SUBSCRIBER_TOPIC = "recognized_text"
ROS_PUBLISHER_TOPIC = "parsed_text"

nlu_engine = None
publisher = None


# Maybe it would be good to take a look only at messages with certain probability for example > 0.6
def callback(msg):
    if not rospy.is_shutdown() and nlu_engine is not None and publisher is not None:
        parsed_text_dict = nlu_engine.parse(msg.data) # nlu_engine.parse(unicode(msg.data, "utf-8"))
        parsed_text_str = json.dumps(parsed_text_dict, indent=2)
        print(parsed_text_str)
        intent_name = parsed_text_dict["intent"]["intentName"]
        if intent_name:
            publisher.publish(parsed_text_str)


def main():
    # Load dataset for parsing
    with open(DATASET_PATH) as f:
        dataset = json.load(f)

    global publisher
    global nlu_engine
    nlu_engine = SnipsNLUEngine(config=CONFIG_EN)
    nlu_engine.fit(dataset)

    # Initialize Ros node and the Topic subscriber and publisher
    rospy.init_node(str(ROS_NODE_NAME))

    publisher = rospy.Publisher(str(ROS_PUBLISHER_TOPIC), String, queue_size=10)
    rospy.Subscriber(str(ROS_SUBSCRIBER_TOPIC), String, callback)
    print("ROS node '%s' started. Listening from '%s' (ctrl-C to exit)..." % (ROS_NODE_NAME, ROS_SUBSCRIBER_TOPIC))
    rospy.spin()
    print("Ctrl-C received. Shutting down ROS node '%s'!" % ROS_NODE_NAME)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
