# cav_ipfs_node.py
import rospy
from std_msgs.msg import String
import ipfsApi

client = ipfsApi.Client('127.0.0.1', 5001)

def metadata_callback(msg):
    rospy.loginfo(f"Received metadata: {msg.data}")
    ipfs_hash = client.add_json({"timestamp": rospy.get_time(), "data": msg.data})
    rospy.loginfo(f"Stored to IPFS: {ipfs_hash}")

def main():
    rospy.init_node("cav_ipfs_node")
    rospy.Subscriber("/sdr/packet_metadata", String, metadata_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
