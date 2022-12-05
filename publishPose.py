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

class PoseSetter(Node):
    def __init__(self, name):
        super().__init__(name)
        self.__initial_pose = set_initial_pose()
        self.__goal_pose = set_goal_pose()
        self.__initial_pose_counter = 0
        self.__timer = self.create_timer(
            1.0,
            self.timer_cb,
            #clock=Clock(clock_type=ClockType.SYSTEM_TIME),
        )
        self.__pub_initial_pose = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose",20
        )
        self.__pub_goal_pose = self.create_publisher(
            PoseStamped, "/planning/mission_planning/goal",20
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
                self.__goal_pose.header.stamp = self.__current_time
                self.__pub_goal_pose.publish(self.__goal_pose)
                print("publish pose")
            else:
                rclpy.shutdown()
        #self.__prev_time = self.__current_time

"""            if self.__current_time == self.__prev_time:
                self.__counter += 1
            else:
                self.__counter = 0
            self.__prev_time = self.__current_time
            if self.__counter >= 5:
                rclpy.shutdown()
"""
"""
ros2 topic echo /initialpose
header:
  stamp:
    sec: 1585897285
    nanosec: 189880315
  frame_id: map
pose:
  pose:
    position:
      x: 89566.8359375
      y: 42297.51953125
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.31243059418364677
      w: 0.9499405896255058
  covariance:
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.06853891909122467
"""
def set_initial_pose() -> Optional[PoseWithCovarianceStamped]:
    ros_init_pose = PoseWithCovarianceStamped()
    ros_init_pose.header.frame_id = "map"
    ros_init_pose.pose.pose.position.x = 89566.8359375
    ros_init_pose.pose.pose.position.y = 42297.51953125
    ros_init_pose.pose.pose.position.z = 0.0
    ros_init_pose.pose.pose.orientation.x = 0.0
    ros_init_pose.pose.pose.orientation.y = 0.0
    ros_init_pose.pose.pose.orientation.z = 0.31243059418364677
    ros_init_pose.pose.pose.orientation.w = 0.9499405896255058
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

"""
ros2 topic echo /planning/mission_planning/goal
header:
  stamp:
    sec: 1585897285
    nanosec: 189880315
  frame_id: map
pose:
  position:
    x: 89578.84375
    y: 42328.15234375
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.8816745987679437
    w: 0.47185792553202444
"""
def set_goal_pose() -> Optional[PoseStamped]:
    ros_init_pose = PoseStamped()
    ros_init_pose.header.frame_id = "map"
    ros_init_pose.pose.position.x = 89578.84375
    ros_init_pose.pose.position.y = 42328.15234375
    ros_init_pose.pose.position.z = 0.0
    ros_init_pose.pose.orientation.x = 0.0
    ros_init_pose.pose.orientation.y = 0.0
    ros_init_pose.pose.orientation.z = 0.8816745987679437
    ros_init_pose.pose.orientation.w = 0.47185792553202444
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
