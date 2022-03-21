import time
import rclpy
from rclpy.node import Node

from threading import Thread

MAX_RANGE = 0.1
      
class ArmController(Node):

    def __init__(self):
        super().__init__('ur_controller_node')
        self.detected = False
        
    def do_something(self):
        self.get_logger().info("Do something!")


def main(args=None):
    rclpy.init(args=args)

    controller = ArmController()

    spin_thread = Thread(target=rclpy.spin, args=(controller,))
    spin_thread.start()

    while rclpy.ok():
        
        if not controller.detected:
            time.sleep(0.1)
        else:
            controller.do_something()

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()