#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

my_state = {"id": "node1", "role": "tx", "timestamp": rospy.Time.now().to_sec()}

def state_callback(msg):
    peer = json.loads(msg.data)
    rospy.loginfo(f"Peer state received: {peer}")
    if peer["role"] == "tx" and my_state["role"] == "tx":
        rospy.logwarn("Collision risk! Switching to RX.")
        my_state["role"] = "rx"

def publish_state(pub):
    msg = String()
    msg.data = json.dumps(my_state)
    pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("sdr_coord_node")
    pub = rospy.Publisher("/sdr_handshake", String, queue_size=10)
    sub = rospy.Subscriber("/sdr_handshake", String, state_callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        publish_state(pub)
        rate.sleep()
