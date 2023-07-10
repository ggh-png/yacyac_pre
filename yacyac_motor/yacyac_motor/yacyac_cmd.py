
import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 멀티 스레드
from rclpy.executors import MultiThreadedExecutor
class cmd_vel_2_rpm(Node):
    def __init__(self):
        super().__init__('cmd_vel_2_rpm')
        self.subscription = self.create_subscription(Twist, '/yacyac/cmd_vel', self.cmd_vel_callback, 10)
        self.subscription  # prevent unused variable warning
        self.wheel_radius = 0.0679  # 바퀴 반지름 (단위: m/)
        self.wheel_base = 0.31477  # 바퀴 베이스 (단위: m)/

    def cmd_vel_callback(self, msg):
        linear_vel = msg.linear.x  # 선속도(m/s)
        angular_vel = msg.angular.z  # 각속도(rad/s)

        # 좌측, 우측 바퀴의 선속도 계산
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_base / 2) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_base / 2) / self.wheel_radius

        # 좌측 바퀴 RPM 값을 발행
        left_wheel_rpm = (left_wheel_vel * 60 / (2 * 3.14159))
        # 우측 바퀴 RPM 값을 발행
        right_wheel_rpm = (right_wheel_vel * 60 / (2 * 3.14159))
        print("left_wheel_rpm:", left_wheel_rpm, "right_wheel_rpm:", right_wheel_rpm)


def main(args=None):
    rp.init(args=args)
    cmd_vel_sub= cmd_vel_2_rpm()

    executor = MultiThreadedExecutor()
    executor.add_node(cmd_vel_sub)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        cmd_vel_sub.destroy_node()
        rp.shutdown()
if __name__ == '__main__':
    main()
