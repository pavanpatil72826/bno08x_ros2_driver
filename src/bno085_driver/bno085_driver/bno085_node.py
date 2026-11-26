import math
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu, MagneticField


def parse_line(line: str):
    """Parse '$IMU,...*CS'. Returns list of 10 floats (qx qy qz qw gx gy gz ax ay az),
    or 13 floats if the line also carries mag (mx my mz, in uT), or None."""
    if not line.startswith('$') or '*' not in line:
        return None
    body, _, cs = line[1:].rpartition('*')
    calc = 0
    for ch in body.encode():
        calc ^= ch
    try:
        if calc != int(cs.strip(), 16):
            return None
        f = body.split(',')
        if f[0] != 'IMU' or len(f) not in (12, 15):
            return None
        return [float(x) for x in f[2:]]
    except ValueError:
        return None


class Bno085Node(Node):
    def __init__(self):
        super().__init__('bno085_node')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 921600).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        topic = self.declare_parameter('topic', 'imu/data').value
        o_sd = self.declare_parameter('orientation_stddev', 0.01).value
        g_sd = self.declare_parameter('angular_velocity_stddev', 0.005).value
        a_sd = self.declare_parameter('linear_acceleration_stddev', 0.05).value

        self.pub = self.create_publisher(Imu, topic, 10)
        rpy_topic = self.declare_parameter('rpy_topic', 'imu/rpy').value
        self.rpy_deg = self.declare_parameter('rpy_degrees', False).value
        self.rpy_pub = self.create_publisher(Vector3Stamped, rpy_topic, 10)
        mag_topic = self.declare_parameter('mag_topic', 'imu/mag').value
        # Firmware reports uT; sensor_msgs/MagneticField wants Tesla.
        self.mag_scale = self.declare_parameter('mag_scale', 1e-6).value
        self.mag_pub = self.create_publisher(MagneticField, mag_topic, 10)
        self.o_cov = [o_sd ** 2 if i % 4 == 0 else 0.0 for i in range(9)]
        self.g_cov = [g_sd ** 2 if i % 4 == 0 else 0.0 for i in range(9)]
        self.a_cov = [a_sd ** 2 if i % 4 == 0 else 0.0 for i in range(9)]

        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    def publish(self, v):
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame_id
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = v[0:4]
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = v[4:7]
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = v[7:10]
        m.orientation_covariance = self.o_cov
        m.angular_velocity_covariance = self.g_cov
        m.linear_acceleration_covariance = self.a_cov
        self.pub.publish(m)

        # Roll/pitch/yaw (ZYX, REP-103) from the quaternion; x=roll, y=pitch, z=yaw.
        qx, qy, qz, qw = v[0:4]
        r = Vector3Stamped()
        r.header = m.header
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        k = 180.0 / math.pi if self.rpy_deg else 1.0
        r.vector.x, r.vector.y, r.vector.z = roll * k, pitch * k, yaw * k
        self.rpy_pub.publish(r)

        if len(v) >= 13:
            mm = MagneticField()
            mm.header = m.header
            k = self.mag_scale
            mm.magnetic_field.x, mm.magnetic_field.y, mm.magnetic_field.z = (c * k for c in v[10:13])
            self.mag_pub.publish(mm)

    def read_loop(self):
        bad = 0
        while self.running and rclpy.ok():
            try:
                with serial.Serial(self.port, self.baud, timeout=1.0) as ser:
                    self.get_logger().info(f'Opened {self.port} @ {self.baud}')
                    ser.reset_input_buffer()
                    while self.running and rclpy.ok():
                        raw = ser.readline()
                        if not raw:
                            self.get_logger().warn('No data from ESP32 (1 s)', throttle_duration_sec=5.0)
                            continue
                        line = raw.decode('ascii', errors='ignore').strip()
                        if line.startswith('#'):
                            self.get_logger().info(f'ESP32: {line[1:]}')
                            continue
                        v = parse_line(line)
                        if v is None:
                            bad += 1
                            if bad % 50 == 1:
                                self.get_logger().warn(f'Dropped malformed line(s): {bad} total')
                            continue
                        self.publish(v)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial error: {e}; retrying in 2 s', throttle_duration_sec=10.0)
                time.sleep(2.0)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main():
    rclpy.init()
    node = Bno085Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
