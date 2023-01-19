from typing import Dict
from typing import Optional
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.clock import ClockType
import sys
import yaml

filename = "initialPose.yaml"
try:
  filename = sys.argv[1]
except:
  pass
print(filename)

class PoseSetter(Node):
    def __init__(self, name):
        super().__init__(name)

        self.__initial_pose = self.get_pose(filename, "initialPose")
        self.__initial_pose_counter = 0
        self.__timer = self.create_timer(
            1.0,
            self.timer_cb,
            #clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self.__pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",20
        )
        #self.__prev_time = 0
        self.set_parameters([rclpy.Parameter(name="use_sim_time", value=True)])

    def timer_cb(self) -> None:
        self.__current_time = self.get_clock().now().to_msg()
        print(self.__current_time.sec)
        if self.__current_time.sec > 0:
            self.__initial_pose_counter += 1
            if self.__initial_pose_counter <= 5:
                self.__initial_pose.header.stamp = self.__current_time
                self.__pub_initial_pose.publish(self.__initial_pose)
                print("publish pose")
            else:
                rclpy.shutdown()
    def get_pose(self, filename, topicname) -> Optional[PoseWithCovarianceStamped]:
      with open(filename, "r") as f:
        topic=yaml.safe_load(f)
      pose=topic[topicname]["pose"]
      orientation=topic[topicname]["orientation"]
      print(topicname)
      print(pose)
      print(orientation)

      ros_init_pose = PoseWithCovarianceStamped()
      ros_init_pose.header.frame_id = "map"

      ros_init_pose.pose.pose.position.x = pose["x"]
      ros_init_pose.pose.pose.position.y = pose["y"]
      ros_init_pose.pose.pose.position.z = pose["z"]
      ros_init_pose.pose.pose.orientation.x = orientation["x"] 
      ros_init_pose.pose.pose.orientation.y = orientation["y"]
      ros_init_pose.pose.pose.orientation.z = orientation["z"]
      ros_init_pose.pose.pose.orientation.w = orientation["w"]
      ros_init_pose.pose.covariance = np.array(
            [
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.25,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.06853891909122467,
            ]
        )
      return ros_init_pose

def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    poseSetter = PoseSetter("PoseSetter")
    executor.add_node(poseSetter)
    executor.spin()
    poseSetter.destroy_node()
    #rclpy.shutdown()


if __name__ == "__main__":
    main()
