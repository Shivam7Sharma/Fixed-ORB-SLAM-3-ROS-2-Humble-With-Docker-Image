import rclpy
from rclpy.node import Node

from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

from threading import Thread
from queue import Queue
import time

class ManualControl(Node):
  def __init__(self):
    super().__init__("manual_control")
    self.joystick_sub_ = self.create_subscription(
      Joy
      , '/joy'
      , self.process_joy
      , 10)

    self.cmd_vel_pub = self.create_publisher(
      Twist
      , '/lawn_mower/cmd_vel'
      , 1)

    self.toggle_manual_sub_ = self.create_subscription(
      Empty
      , '/lawn_mower/toggle_manual'
      , self.toggle_manual
      , 1)
    
    self.toggle_perimeter_pub = self.create_publisher(
      Empty
      , '/lawn_mower/toggle_perimeter'
      , 1
    )

    self.manual_control = True
    self.a_held = False
    self.toggle_perimeter_start = None
    self.a_release_queue = Queue()

  def process_joy(self, msg):
    if not self.manual_control:
      return
    sec = msg.header.stamp.sec
    nano = msg.header.stamp.nanosec
    time = sec + 1e-9 * nano
    # process movement
    yaw, _, lt, _, _, rt, _, _ = msg.axes
    lt = (1 - lt) * 0.5
    rt = (1 - rt) * 0.5
    forward = rt - lt
    if abs(forward) < 1e-3:
      forward = 0.0
    if abs(yaw) < .1: #deadzone of .1
      yaw = 0.0
    vel_msg = Twist()
    vel_msg.linear.x = forward * .2
    vel_msg.angular.z = yaw * .2
    self.cmd_vel_pub.publish(vel_msg)

    #process buttons
    a, b, x, y, lb, rb, select, start, home, l_stick, r_stick = msg.buttons
    if a:
      if not self.a_held:
        self.a_held = True
        t = Thread(target=self.check_toggle_perimeter)
        t.start()
    elif self.a_held:
      self.a_release_queue.put(True)
    else:
      while not self.a_release_queue.empty():
        self.a_release_queue.get()

  def check_toggle_perimeter(self):
    secs_left = 5
    while self.a_release_queue.empty():
      if secs_left == 0:
        self.get_logger().info("Toggling Perimeter")
        toggle_perimeter_msg = Empty()
        self.toggle_perimeter_pub.publish(toggle_perimeter_msg)
        self.a_held = False
        return
      self.get_logger().info("Toggling Perimeter in {} seconds".format(secs_left))
      time.sleep(1)
      secs_left -= 1
    self.a_release_queue.get()
    self.get_logger().info("Cancel Toggling Perimeter")
    self.a_held = False



  def toggle_manual(self, msg):
    self.get_logger().info("Toggling from {} to {}".format(
      "MANUAL" if self.manual_control else "AUTO"
      , "AUTO" if self.manual_control else "MANUAL"
      ))
    self.manual_control = not self.manual_control

def main(args=None):
    rclpy.init(args=args)

    manual_control = ManualControl()
    
    rclpy.spin(manual_control)

    manual_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()