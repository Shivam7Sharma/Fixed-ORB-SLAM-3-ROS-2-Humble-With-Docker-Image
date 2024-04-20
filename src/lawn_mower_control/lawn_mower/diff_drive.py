import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

from .motor_driver import MotorDriver


class DiffDrive(Node):
    def __init__(self):
        super().__init__('diff_drive')

        self.cmd_vel_sub_ = self.create_subscription(
            Twist,
            '/lawn_mower/cmd_vel',
            self.diff_drive_callback,
            10
        )

        self.motor_driver = MotorDriver()
        

    def diff_drive_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        self.get_logger().info("Received command linear={:.2f} angular={:.2f}".format(linear,angular))

        body_radius = None
        if abs(angular) > 1e-6:
            body_radius = linear / angular

        if body_radius is None: # forward/backward
            left_speed = right_speed = linear
        else:
            left_radius = body_radius - 1
            right_radius = body_radius + 1
            if left_radius == 0:
                left_speed = 0
                right_speed = max(abs(linear), abs(angular)) * (1 if linear > 0 else -1)
            elif right_radius == 0:
                left_speed = max(abs(linear), abs(angular)) * (1 if linear > 0 else -1)
                right_speed = 0
            elif body_radius == 0:
                left_speed = -angular
                right_speed = angular
            elif body_radius < 0:
                if linear > 0:
                    left_speed = max(abs(linear), abs(angular))
                    right_speed = left_speed * right_radius / left_radius
                else:
                    right_speed = -max(abs(linear), abs(angular))
                    left_speed = right_speed * left_radius / right_radius
            else:
                if linear > 0:
                    right_speed = max(abs(linear), abs(angular))
                    left_speed = right_speed * left_radius / right_radius
                else:
                    left_speed = -max(abs(linear), abs(angular))
                    right_speed = left_speed * right_radius / left_radius
        self.get_logger().info("Sending speeds left_speed={:.2f} right_speed={:.2f}".format(left_speed, right_speed))
        motor_driver_log = self.motor_driver.update(left_speed, right_speed)
        self.get_logger().info("Motor Driver:\n" + motor_driver_log)

def main(args=None):
    rclpy.init(args=args)

    diff_drive = DiffDrive()

    rclpy.spin(diff_drive)

    diff_drive.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()