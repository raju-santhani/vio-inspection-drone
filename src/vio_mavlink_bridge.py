#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from pymavlink import mavutil


class VioMavlinkBridge(Node):
    def __init__(self):
        super().__init__('vio_mavlink_bridge')

        # CubeOrange+ serial connection
        self.port = (
            '/dev/serial/by-id/'
            'usb-CubePilot_CubeOrange+_3F0035000651333232393436-if00'
        )
        self.baud = 115200

        self.get_logger().info(
            f'Connecting to CubeOrange+: {self.port} @ {self.baud}'
        )

        self.mav = mavutil.mavlink_connection(
            self.port,
            baud=self.baud,
            source_system=245,
            source_component=191
        )

        self.get_logger().info('Waiting for ArduPilot heartbeat...')

        while rclpy.ok():
            msg = self.mav.recv_match(
                type='HEARTBEAT',
                blocking=True,
                timeout=1
            )

            if msg is None:
                continue

            if (
                msg.get_srcSystem() == 1
                and msg.get_srcComponent() == 1
                and msg.autopilot ==
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA
            ):
                break

        self.target_system = 1
        self.target_component = 1

        self.get_logger().info(
            'ArduPilot heartbeat received '
            '(system=1, component=1)'
        )

        self.count = 0
        self.last_report = time.monotonic()

        self.subscription = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Subscribed to /visual_slam/tracking/odometry'
        )

    def odom_callback(self, msg):
        # ==========================================================
        # ISAAC ROS VSLAM -> ARDUPILOT FRAME CONVERSION
        #
        # Assumption for this setup:
        #   Camera +X = drone Forward
        #   Camera +Y = drone Left
        #   Camera +Z = drone Up
        #
        # Isaac ROS source convention: FLU
        # ArduPilot target convention:
        #   local  = FRD
        #   body   = FRD
        #
        # Therefore:
        #   X_FRD =  X_FLU
        #   Y_FRD = -Y_FLU
        #   Z_FRD = -Z_FLU
        # ==========================================================

        # -------------------------
        # Position [m]
        # -------------------------
        x_ros = float(msg.pose.pose.position.x)
        y_ros = float(msg.pose.pose.position.y)
        z_ros = float(msg.pose.pose.position.z)

        x = x_ros
        y = -y_ros
        z = -z_ros

        # -------------------------
        # Linear velocity [m/s]
        # -------------------------
        vx_ros = float(msg.twist.twist.linear.x)
        vy_ros = float(msg.twist.twist.linear.y)
        vz_ros = float(msg.twist.twist.linear.z)

        vx = vx_ros
        vy = -vy_ros
        vz = -vz_ros

        # -------------------------
        # Angular velocity [rad/s]
        #
        # Same FLU -> FRD axis transformation:
        #   wx =  wx
        #   wy = -wy
        #   wz = -wz
        # -------------------------
        wx_ros = float(msg.twist.twist.angular.x)
        wy_ros = float(msg.twist.twist.angular.y)
        wz_ros = float(msg.twist.twist.angular.z)

        rollspeed = wx_ros
        pitchspeed = -wy_ros
        yawspeed = -wz_ros

        # -------------------------
        # Quaternion
        #
        # ROS message order:
        #   x, y, z, w
        #
        # MAVLink order:
        #   w, x, y, z
        #
        # FLU -> FRD:
        #   w' = w
        #   x' = x
        #   y' = -y
        #   z' = -z
        # -------------------------
        q_ros = msg.pose.pose.orientation

        q = [
            float(q_ros.w),
            float(q_ros.x),
            -float(q_ros.y),
            -float(q_ros.z)
        ]

        # -------------------------
        # Use Isaac ROS timestamp
        # -------------------------
        time_usec = (
            int(msg.header.stamp.sec) * 1_000_000
            + int(msg.header.stamp.nanosec) // 1_000
        )

        # -------------------------
        # MAVLink frames
        # -------------------------
        frame_id = mavutil.mavlink.MAV_FRAME_LOCAL_FRD
        child_frame_id = mavutil.mavlink.MAV_FRAME_BODY_FRD

        # ==========================================================
        # Covariance
        #
        # ROS Odometry covariance:
        #   6x6 = 36 elements
        #
        # MAVLink ODOMETRY covariance:
        #   21 elements = upper triangular portion
        #
        # We first transform the covariance from FLU to FRD.
        # For this sign-change transform:
        #
        #   S = diag(1, -1, -1, 1, -1, -1)
        #
        # for [x,y,z,roll,pitch,yaw].
        # ==========================================================

        def transform_covariance_6x6(cov_ros):
            # Convert flat ROS covariance to 6x6 matrix
            C = [
                [float(cov_ros[r * 6 + c]) for c in range(6)]
                for r in range(6)
            ]

            # FLU -> FRD sign transformation
            signs = [1.0, -1.0, -1.0, 1.0, -1.0, -1.0]

            C_out = [
                [
                    signs[r] * C[r][c] * signs[c]
                    for c in range(6)
                ]
                for r in range(6)
            ]

            return C_out

        def upper_triangle_21(C):
            # MAVLink ODOMETRY covariance layout:
            # row-major upper triangular:
            #
            #  0  1  2  3  4  5
            #     6  7  8  9 10
            #       11 12 13 14
            #          15 16 17
            #             18 19
            #                20
            #
            return [
                C[0][0],
                C[0][1],
                C[0][2],
                C[0][3],
                C[0][4],
                C[0][5],
                C[1][1],
                C[1][2],
                C[1][3],
                C[1][4],
                C[1][5],
                C[2][2],
                C[2][3],
                C[2][4],
                C[2][5],
                C[3][3],
                C[3][4],
                C[3][5],
                C[4][4],
                C[4][5],
                C[5][5]
            ]

        pose_covariance = upper_triangle_21(
            transform_covariance_6x6(msg.pose.covariance)
        )

        # Twist covariance has the same [vx,vy,vz,wx,wy,wz]
        # axis ordering and therefore receives the same FLU -> FRD
        # sign transformation.
        velocity_covariance = upper_triangle_21(
            transform_covariance_6x6(msg.twist.covariance)
        )

        estimator_type = mavutil.mavlink.MAV_ESTIMATOR_TYPE_VIO

        # ==========================================================
        # SEND MAVLink ODOMETRY
        # ==========================================================
        self.mav.mav.odometry_send(
            time_usec,
            frame_id,
            child_frame_id,

            x,
            y,
            z,

            q,

            vx,
            vy,
            vz,

            rollspeed,
            pitchspeed,
            yawspeed,

            pose_covariance,
            velocity_covariance,

            0,              # reset_counter
            estimator_type,
            100,            # quality

            False           # q_estimated
        )

        self.count += 1

        now = time.monotonic()

        if now - self.last_report >= 1.0:
            rate = self.count / (now - self.last_report)

            self.get_logger().info(
                f'ODOMETRY sent: {rate:.1f} Hz | '
                f'NED x={x:.3f} y={y:.3f} z={z:.3f} | '
                f'V={vx:.3f},{vy:.3f},{vz:.3f} | '
                f'Q={q[0]:.4f},{q[1]:.4f},{q[2]:.4f},{q[3]:.4f}'
            )

            self.count = 0
            self.last_report = now


def main(args=None):
    rclpy.init(args=args)

    node = VioMavlinkBridge()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.mav.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
