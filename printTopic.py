import sys
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

topicname = "/initialPose"
try:
    topicname = sys.argv[1]
except:
    pass

def print_topic(msg):
    print(topicname)
    #print("Position", msg.pose.pose.position)
    print(topicname+":")
    print(" pose:")
    print("  x:", msg.pose.pose.position.x)
    print("  y:", msg.pose.pose.position.y)
    print("  z:", msg.pose.pose.position.z)
    print("  orientation:")
    #print("Orientation", msg.pose.pose.orientation)
    print("  x:",msg.pose.pose.orientation.x)
    print("  y:",msg.pose.pose.orientation.y)
    print("  z:",msg.pose.pose.orientation.z)
    print("  w:",msg.pose.pose.orientation.w)



rclpy.init()
node = Node("topic_saver")
sub = node.create_subscription(PoseWithCovarianceStamped, topicname, print_topic, 1)
rclpy.spin(node)
sub.destory()
node.destroy_node()
rclpy.shutdown()

