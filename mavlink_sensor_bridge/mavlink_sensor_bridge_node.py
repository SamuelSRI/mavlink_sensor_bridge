#!/usr/bin/env python3

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from pymavlink import mavutil


class MavlinkSensorBridgeNode(Node):
    """Read Cube Orange MAVLink data and publish ROS 2 sensor topics."""

    def __init__(self):
        super().__init__("mavlink_sensor_bridge_node")

        self.declare_parameter("port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)

        self.declare_parameter("imu_frame_id", "imu_link")
        self.declare_parameter("gps_frame_id", "gps_link")

        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("heartbeat_rate_hz", 1.0)

        self.declare_parameter("attitude_rate_hz", 50.0)
        self.declare_parameter("raw_imu_rate_hz", 10.0)
        self.declare_parameter("gps_rate_hz", 10.0)

        self.declare_parameter("debug", False)

        self.port = str(self.get_parameter("port").value)
        self.baudrate = int(self.get_parameter("baudrate").value)

        self.imu_frame_id = str(self.get_parameter("imu_frame_id").value)
        self.gps_frame_id = str(self.get_parameter("gps_frame_id").value)

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.heartbeat_rate_hz = float(
            self.get_parameter("heartbeat_rate_hz").value
        )

        self.attitude_rate_hz = float(
            self.get_parameter("attitude_rate_hz").value
        )
        self.raw_imu_rate_hz = float(
            self.get_parameter("raw_imu_rate_hz").value
        )
        self.gps_rate_hz = float(
            self.get_parameter("gps_rate_hz").value
        )

        self.debug = bool(self.get_parameter("debug").value)

        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.gps_status_pub = self.create_publisher(
            DiagnosticStatus,
            "/gps/status",
            10,
        )

        self.master = None

        self.last_quaternion = None
        self.last_angular_velocity = None
        self.last_linear_acceleration = None

        self.last_gps_time = 0.0
        self.last_heartbeat_time = 0.0

        self.connect_mavlink()

        timer_period = 1.0 / self.publish_rate_hz
        self.timer = self.create_timer(timer_period, self.loop)

        self.get_logger().info("MAVLink UART sensor bridge started.")
        self.get_logger().info(f"UART port: {self.port}")
        self.get_logger().info(f"Baudrate: {self.baudrate}")
        self.get_logger().info("Publishing IMU on /imu/data")
        self.get_logger().info("Publishing GPS on /gps/fix")

    def connect_mavlink(self):
        while rclpy.ok():
            try:
                self.get_logger().info(
                    "Connecting to Cube Orange through UART..."
                )

                self.master = mavutil.mavlink_connection(
                    self.port,
                    baud=self.baudrate,
                    source_system=255,
                    source_component=0,
                )

                self.get_logger().info("Waiting for MAVLink heartbeat...")
                self.master.wait_heartbeat(timeout=10)

                self.get_logger().info(
                    f"Heartbeat received from system "
                    f"{self.master.target_system}, component "
                    f"{self.master.target_component}"
                )

                self.request_message_streams()
                return

            except Exception as exc:
                self.get_logger().warn(
                    f"MAVLink UART connection failed: {exc}"
                )
                self.get_logger().warn("Retrying in 2 seconds...")
                time.sleep(2.0)

    def set_message_interval(
        self,
        message_id: int,
        frequency_hz: float,
    ):
        if self.master is None or frequency_hz <= 0.0:
            return

        interval_us = int(1_000_000.0 / frequency_hz)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            float(message_id),
            float(interval_us),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )

    def request_message_streams(self):
        if self.master is None:
            return

        try:
            self.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,
                self.attitude_rate_hz,
            )

            self.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU,
                self.raw_imu_rate_hz,
            )

            self.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
                self.gps_rate_hz,
            )

            self.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                self.gps_rate_hz,
            )

            self.get_logger().info(
                "Requested ATTITUDE_QUATERNION, RAW_IMU and GPS rates."
            )

        except Exception as exc:
            self.get_logger().warn(
                f"Could not request MAVLink message intervals: {exc}"
            )

            try:
                self.master.mav.request_data_stream_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_DATA_STREAM_ALL,
                    10,
                    1,
                )

                self.get_logger().warn(
                    "Falling back to MAV_DATA_STREAM_ALL at 10 Hz."
                )

            except Exception as fallback_exc:
                self.get_logger().warn(
                    f"Fallback stream request failed: {fallback_exc}"
                )

    def send_heartbeat(self):
        if self.master is None:
            return

        now = time.time()

        if self.heartbeat_rate_hz <= 0.0:
            return

        if now - self.last_heartbeat_time < (
            1.0 / self.heartbeat_rate_hz
        ):
            return

        self.last_heartbeat_time = now

        try:
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

        except Exception as exc:
            self.get_logger().warn(
                f"Failed to send heartbeat: {exc}"
            )

    def loop(self):
        if self.master is None:
            self.connect_mavlink()
            return

        self.send_heartbeat()

        imu_should_publish = False

        try:
            msg = self.master.recv_match(blocking=False)

            while msg is not None:
                if self.handle_mavlink_message(msg):
                    imu_should_publish = True

                msg = self.master.recv_match(blocking=False)

            if imu_should_publish:
                self.publish_imu()

        except Exception as exc:
            self.get_logger().warn(
                f"MAVLink read error: {exc}"
            )
            self.master = None

    def handle_mavlink_message(self, msg) -> bool:
        msg_type = msg.get_type()

        if msg_type == "BAD_DATA":
            return False

        if self.debug:
            self.get_logger().info(
                f"Received MAVLink message: {msg_type}"
            )

        if msg_type == "ATTITUDE_QUATERNION":
            self.handle_attitude_quaternion(msg)

            # ATTITUDE_QUATERNION arrives around 48 Hz and drives
            # publication of /imu/data.
            return True

        if msg_type == "RAW_IMU":
            self.handle_raw_imu(msg)

            # RAW_IMU updates acceleration, but does not control the
            # publication frequency.
            return False

        if msg_type == "GPS_RAW_INT":
            self.handle_gps_raw_int(msg)
            return False

        if msg_type == "GLOBAL_POSITION_INT":
            self.handle_global_position_int(msg)
            return False

        return False

    @staticmethod
    def quaternion_to_yaw(
        qw: float,
        qx: float,
        qy: float,
        qz: float,
    ) -> float:
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)

        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def yaw_to_quaternion(yaw: float):
        half_yaw = 0.5 * yaw

        return (
            math.cos(half_yaw),
            0.0,
            0.0,
            math.sin(half_yaw),
        )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return math.atan2(
            math.sin(angle),
            math.cos(angle),
        )

    def handle_attitude_quaternion(self, msg):
        # MAVLink attitude uses NED/FRD.
        #
        # For this 2D rover:
        # - MAVLink NED yaw: zero toward North, clockwise positive
        # - ROS ENU yaw: zero toward East, counter-clockwise positive
        yaw_ned = self.quaternion_to_yaw(
            float(msg.q1),
            float(msg.q2),
            float(msg.q3),
            float(msg.q4),
        )

        yaw_enu = self.normalize_angle(
            (math.pi / 2.0) - yaw_ned
        )

        self.last_quaternion = self.yaw_to_quaternion(
            yaw_enu
        )

        # Body frame conversion:
        # MAVLink FRD -> ROS FLU
        # x unchanged, y inverted, z inverted
        self.last_angular_velocity = (
            float(msg.rollspeed),
            -float(msg.pitchspeed),
            -float(msg.yawspeed),
        )

    def handle_raw_imu(self, msg):
        # ArduPilot RAW_IMU commonly provides:
        # - acceleration in milli-g
        # - angular velocity in millirad/s
        #
        # RAW_IMU is used here only as an acceleration fallback.
        ax = float(msg.xacc) * 9.80665 / 1000.0
        ay = -float(msg.yacc) * 9.80665 / 1000.0
        az = -float(msg.zacc) * 9.80665 / 1000.0

        self.last_linear_acceleration = (
            ax,
            ay,
            az,
        )

        # Only use RAW_IMU gyro before ATTITUDE_QUATERNION has supplied
        # angular velocity.
        if self.last_angular_velocity is None:
            gx = float(msg.xgyro) / 1000.0
            gy = -float(msg.ygyro) / 1000.0
            gz = -float(msg.zgyro) / 1000.0

            self.last_angular_velocity = (
                gx,
                gy,
                gz,
            )

    def publish_imu(self):
        imu_msg = Imu()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.imu_frame_id

        if self.last_quaternion is not None:
            qw, qx, qy, qz = self.last_quaternion

            imu_msg.orientation.w = qw
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz

            imu_msg.orientation_covariance[0] = 0.05
            imu_msg.orientation_covariance[4] = 0.05
            imu_msg.orientation_covariance[8] = 0.05

        else:
            imu_msg.orientation_covariance[0] = -1.0

        if self.last_angular_velocity is not None:
            gx, gy, gz = self.last_angular_velocity

            imu_msg.angular_velocity.x = gx
            imu_msg.angular_velocity.y = gy
            imu_msg.angular_velocity.z = gz

            imu_msg.angular_velocity_covariance[0] = 0.02
            imu_msg.angular_velocity_covariance[4] = 0.02
            imu_msg.angular_velocity_covariance[8] = 0.02

        else:
            imu_msg.angular_velocity_covariance[0] = -1.0

        if self.last_linear_acceleration is not None:
            ax, ay, az = self.last_linear_acceleration

            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az

            imu_msg.linear_acceleration_covariance[0] = 0.2
            imu_msg.linear_acceleration_covariance[4] = 0.2
            imu_msg.linear_acceleration_covariance[8] = 0.2

        else:
            imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

    def handle_gps_raw_int(self, msg):
        gps_msg = NavSatFix()

        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = self.gps_frame_id

        gps_msg.latitude = float(msg.lat) / 1e7
        gps_msg.longitude = float(msg.lon) / 1e7
        gps_msg.altitude = float(msg.alt) / 1000.0

        fix_type = int(msg.fix_type)
        satellites_visible = int(msg.satellites_visible)

        if fix_type >= 3:
            gps_msg.status.status = NavSatStatus.STATUS_FIX
        else:
            gps_msg.status.status = NavSatStatus.STATUS_NO_FIX

        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        eph_m = self.safe_cm_to_m(msg.eph)
        epv_m = self.safe_cm_to_m(msg.epv)

        if eph_m is not None and epv_m is not None:
            gps_msg.position_covariance[0] = eph_m * eph_m
            gps_msg.position_covariance[4] = eph_m * eph_m
            gps_msg.position_covariance[8] = epv_m * epv_m

            gps_msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            )

        else:
            gps_msg.position_covariance_type = (
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            )

        self.gps_pub.publish(gps_msg)

        self.publish_gps_status(
            fix_type,
            satellites_visible,
            eph_m,
            epv_m,
        )

        self.last_gps_time = time.time()

    def handle_global_position_int(self, msg):
        now = time.time()

        # Prefer GPS_RAW_INT when it was received recently.
        if now - self.last_gps_time < 1.0:
            return

        gps_msg = NavSatFix()

        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.header.frame_id = self.gps_frame_id

        gps_msg.latitude = float(msg.lat) / 1e7
        gps_msg.longitude = float(msg.lon) / 1e7
        gps_msg.altitude = float(msg.alt) / 1000.0

        gps_msg.status.status = NavSatStatus.STATUS_FIX
        gps_msg.status.service = NavSatStatus.SERVICE_GPS

        gps_msg.position_covariance_type = (
            NavSatFix.COVARIANCE_TYPE_UNKNOWN
        )

        self.gps_pub.publish(gps_msg)
        self.last_gps_time = now

    def publish_gps_status(
        self,
        fix_type: int,
        satellites_visible: int,
        eph_m: Optional[float],
        epv_m: Optional[float],
    ):
        status_msg = DiagnosticStatus()

        status_msg.name = "gps"
        status_msg.hardware_id = "cube_orange"

        if fix_type >= 3:
            status_msg.level = DiagnosticStatus.OK
            status_msg.message = "GPS fix"

        elif fix_type == 2:
            status_msg.level = DiagnosticStatus.WARN
            status_msg.message = "2D GPS fix"

        else:
            status_msg.level = DiagnosticStatus.ERROR
            status_msg.message = "No GPS fix"

        status_msg.values.append(
            KeyValue(
                key="fix_type",
                value=str(fix_type),
            )
        )

        status_msg.values.append(
            KeyValue(
                key="satellites_visible",
                value=str(satellites_visible),
            )
        )

        if eph_m is not None:
            status_msg.values.append(
                KeyValue(
                    key="eph_m",
                    value=f"{eph_m:.3f}",
                )
            )

        if epv_m is not None:
            status_msg.values.append(
                KeyValue(
                    key="epv_m",
                    value=f"{epv_m:.3f}",
                )
            )

        self.gps_status_pub.publish(status_msg)

    @staticmethod
    def safe_cm_to_m(value):
        value = int(value)

        if value <= 0:
            return None

        if value >= 65535:
            return None

        return float(value) / 100.0


def main(args=None):
    rclpy.init(args=args)

    node = MavlinkSensorBridgeNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
