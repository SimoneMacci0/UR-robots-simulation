# UR-robots-simulation
ROS2 package containing all the necessary files (launch, urdf and yaml files) to spawn a simple Webots simulation with two UR5 robots playing together. 

You only need to implement the control of the two arms, by modifying the content of the 
ur_controller.py script.

The two robots are endowed with a distance sensor (inspect the topic) on their hand, which enables them to detect when the cube is within reach to be grasped.

To move and control the arms, you will need to use the ur_joint_trajectory_controller/follow_joint_trajectory action, which is directly exposed by the webots_ros2 driver node of the simulation. Make sure to inspect the action interface.

You will need to implement 4 different action requests:
1) close_gripper(), which correspond to the configuration [0.4, 0.4, 0.4] for the joints ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"];
2) open_gripper(), which correspond to the configuration [0.05, 0.05, 0.05] for the joints ["finger_1_joint_1", "finger_2_joint_1", "finger_middle_joint_1"]
3) move_arm(), which corresponds to the configuration [-0.45, -2.3, -1.1, -0.7071, 1.75] for the joints ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]
4) move_arm_to_rest(), which corresponds to the configuration [0.0, 0.0, 0.0, 0.0, 0.0] for the joints ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint"]

Since the actions are executed one after the other sequentially, you need to perform synchronous calls to the respective action servers, therefore you will need the action_client.send_goal() method.

Happy coding :)
