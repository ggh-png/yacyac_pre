
import math
import serial
import rclpy as rp
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Twist, Vector3
from time import sleep
# 멀티 스레드
from rclpy.executors import MultiThreadedExecutor

# 1의 보수를 구하는 함수
def ones_complement(number, bit_length):
    complement = ~number & ((1 << bit_length) - 1)
    return complement


class EncoderOdom(Node):
    def __init__(self):
        super().__init__('yacyac_serial')
        # 오돔 값을 발행하기 위한 publisher
        self.publisher = self.create_publisher(Odometry, '/yacyac/odometry', 10)
        self.serial_port = '/dev/ttyACM0'  # 시리얼 포트 경로
        self.baud_rate = 115200  # 시리얼 통신 속도

        self.wheel_radius = 0.0679  # 바퀴 반지름 (단위: m/)
        self.wheel_base = 0.31477  # 바퀴 베이스 (단위: m)/

        # 초기 위치와 방향
        self.position_x = 0.0
        self.position_y = 0.0
        self.orientation_z = 0.0

        # 타이머 설정
        self.timer_period = 0.0001  # 주기 (단위: 초)
        self.timer = self.create_timer(self.timer_period, self.read_encoder)


    def read_encoder(self):
        ser = serial.Serial(self.serial_port, self.baud_rate)

        # 시리얼 포트로부터 엔코더 값을 읽어옴
        received_data = ser.readline().decode().strip()
        if received_data:
            received_data = int(received_data)

            data1 = (received_data >> 16) & 0xFFFF
            data2 = received_data & 0xFFFF

            if len(bin(data1)[2:]) == 16:
                data1 = ones_complement(data1, 16) * -1

            if len(bin(data2)[2:]) == 16:
                data2 = ones_complement(data2, 16) * -1

            # print("Motor 1:", data1, "Motor 2:", data2)
            left_encoder_value = data1
            right_encoder_value = data2
            # 엔코더 값을 가지고 현재 rpm을 계산합니다.
            left_rpm = left_encoder_value * 60000 / 2652 / 10
            right_rpm = right_encoder_value * 60000 / 2652 / 10
            # 좌측, 우측 바퀴의 선속도 계산
            left_wheel_vel = (2 * math.pi * self.wheel_radius * left_rpm) / 60.0
            right_wheel_vel = (2 * math.pi * self.wheel_radius * right_rpm) / 60.0
            # 이동 거리 계산
            distance = (left_wheel_vel + right_wheel_vel) / 2.0
            # 방향 갱신
            self.orientation_z += (right_wheel_vel - left_wheel_vel) / self.wheel_base

            # 위치 업데이트
            self.position_x += distance * math.cos(self.orientation_z)
            self.position_y += distance * math.sin(self.orientation_z)
            # odom 메시지 생성
            # 오도메트리 메시지 생성
            odometry = Odometry()
            odometry.header = Header()
            odometry.header.stamp = self.get_clock().now().to_msg()
            odometry.header.frame_id = 'odom'
            odometry.child_frame_id = 'base_link'
            odometry.pose.pose.position.x = self.position_x
            odometry.pose.pose.position.y = self.position_y
            odometry.pose.pose.orientation.z = math.sin(self.orientation_z / 2.0)
            odometry.pose.pose.orientation.w = math.cos(self.orientation_z / 2.0)

            # 오도메트리 메시지 발행
            self.publisher.publish(odometry)



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
    encoder_pub = EncoderOdom()
    encoder_pub.read_encoder()
    cmd_vel_sub= cmd_vel_2_rpm()

    executor = MultiThreadedExecutor()
    executor.add_node(encoder_pub)
    # executor.add_node(cmd_vel_sub)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        encoder_pub.destroy_node()
        # cmd_vel_sub.destroy_node()
        rp.shutdown()
if __name__ == '__main__':
    main()
