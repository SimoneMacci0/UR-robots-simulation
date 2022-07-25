import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import Range

from threading import Thread


MAX_RANGE = 0.1
      
class ArmController(Node):

    def __init__(self):
        super().__init__('ur_controller_node')

        # Declare robot name parameter and get value from launch file
        self.declare_parameter("robot_name", "ur5")
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        # Define action client and subscriber
        self.action_client = ActionClient(self, FollowJointTrajectory, 'ur_joint_trajectory_controller/follow_joint_trajectory')
        self.subscriber = self.create_subscription(Range, '{0}/distance_sensor'.format(self.robot_name), self.range_callback, 10)

        # Utility variable
        self.detected = False

    def range_callback(self, msg):
        if msg.range < 0.5 * MAX_RANGE and not self.detected:
            self.detected = True
            
    def log_info(self, msg):
        self.get_logger().info(msg)
        
    def close_gripper(self):
        gripper_msg = FollowJointTrajectory.Goal()
        gripper_msg.trajectory.joint_names = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
        point = JointTrajectoryPoint()
        point.positions = [0.4, 0.4, 0.4]
        gripper_msg.trajectory.points = [point]
        return self.action_client.send_goal(gripper_msg)

    def open_gripper(self):
        gripper_msg = FollowJointTrajectory.Goal()
        gripper_msg.trajectory.joint_names = ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
        point = JointTrajectoryPoint()
        point.positions = [0.05, 0.05, 0.05]
        gripper_msg.trajectory.points = [point]
        return self.action_client.send_goal(gripper_msg)
        
    def move_arm(self):
        move_arm_msg = FollowJointTrajectory.Goal()
        move_arm_msg.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [-0.45, -2.3, -1.1, -0.7071, 1.75]
        move_arm_msg.trajectory.points = [point]
        return self.action_client.send_goal(move_arm_msg)

    def move_to_rest_position(self):
        move_arm_msg = FollowJointTrajectory.Goal()
        move_arm_msg.trajectory.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]
        move_arm_msg.trajectory.points = [point]
        return self.action_client.send_goal(move_arm_msg)

    def pick_and_place_routine(self):

        self.log_info("{0}: Initializing grasp...".format(self.robot_name))
        self.close_gripper()
        self.log_info("{0}: Object grasped!".format(self.robot_name))

        self.log_info("{0}: Moving to the other conveyor belt...".format(self.robot_name))
        self.move_arm()

        self.log_info("{0}: Releasing object...".format(self.robot_name))
        self.open_gripper()
        self.log_info("{0}: Object released!".format(self.robot_name))

        self.log_info("{0}: Moving back to rest position...".format(self.robot_name))
        self.move_to_rest_position()

        self.detected = False


def main(args=None):
    rclpy.init(args=args)

    controller = ArmController()

    spin_thread = Thread(target=rclpy.spin, args=(controller,))
    spin_thread.start()

    while rclpy.ok():
        
        # Wait until object is detected by range sensor
        if not controller.detected:
            time.sleep(0.1)
        # Then, perform pick and place routine
        else:
            controller.pick_and_place_routine()

        
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
