#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)
from rclpy.parameter import Parameter
from fleet_common.params import declare


ODOM_FRAME = "odom"
ROOT_FRAME = "vrtk_link"
IMU_FRAME = "microstrain_imu_link"


# ----------------------------------------------------------
# Quaternion helpers
# ----------------------------------------------------------

def multiply(a, b):
    """Multiply two quaternions (x, y, z, w)."""

    ax, ay, az, aw = a
    bx, by, bz, bw = b

    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def conjugate(q):
    return (-q[0], -q[1], -q[2], q[3])


def normalize(q):
    n = math.sqrt(sum(v * v for v in q))

    if not math.isfinite(n) or n < 1e-9:
        raise ValueError("Invalid quaternion")

    return tuple(v / n for v in q)


def rotate(q, v):
    """Rotate a 3D vector using a quaternion."""

    return multiply(
        multiply(q, (*v, 0.0)),
        conjugate(q)
    )[:3]


def vector3(v):
    return (v.x, v.y, v.z)


def quaternion(q):
    return (q.x, q.y, q.z, q.w)


# ----------------------------------------------------------
# FAST-LIO2 odometry adapter
# ----------------------------------------------------------

class FastLioOdomAdapter(Node):

    def __init__(self):
        super().__init__("fast_lio_odom_adapter")

        # TF listener: reads transforms published from the URDF.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self
        )

        # TF broadcaster: publishes odom -> vrtk_link.
        self._tf_broadcaster = TransformBroadcaster(self)

        # Publishes robot odometry for Nav2.
        self._odom_pub = self.create_publisher(
            Odometry,
            "odom",
            10
        )

        # Subscribes to FAST-LIO2's estimated IMU pose.
        self._sub = self.create_subscription(
            Odometry,
            "fast_lio/odometry",
            self._callback,
            qos_profile_sensor_data
        )

        # Fixed IMU mounting transform, loaded once from TF.
        self._mounting = None
        self._warned_missing_tf = False

        # Previous estimated pose for velocity calculation.
        self._previous = None

        # Filtered velocity estimate.
        self._filtered_twist = None

        self._alpha = float(
            declare(
                self,
                "velocity_smoothing_alpha",
                Parameter.Type.DOUBLE,
                "Weight of new velocity estimates in the exponential moving average; 1 disables smoothing",
                0.01,
                1.0,
            )
        )

        if not 0.0 < self._alpha <= 1.0:
            raise ValueError(
                "velocity_smoothing_alpha must be in (0, 1]"
            )

    # ------------------------------------------------------
    # Get the IMU mounting offset from URDF/TF
    # ------------------------------------------------------

    def _read_mounting(self):

        if self._mounting is not None:
            return self._mounting

        try:
            # Obtain the fixed transform:
            #
            # vrtk_link -> microstrain_imu_link
            #
            # This describes the IMU's position and
            # orientation relative to the robot root.

            tf = self._tf_buffer.lookup_transform(
                ROOT_FRAME,
                IMU_FRAME,
                Time()
            )

        except TransformException as exc:

            if not self._warned_missing_tf:
                self.get_logger().warning(
                    f"Waiting for {ROOT_FRAME} <- "
                    f"{IMU_FRAME}: {exc}"
                )

                self._warned_missing_tf = True

            return None

        t_root_imu = vector3(
            tf.transform.translation
        )

        q_root_imu = normalize(
            quaternion(tf.transform.rotation)
        )

        self._mounting = (
            t_root_imu,
            q_root_imu
        )

        self.get_logger().info(
            f"Loaded IMU mounting transform: {t_root_imu}"
        )

        return self._mounting

    # ------------------------------------------------------
    # Process FAST-LIO2 odometry
    # ------------------------------------------------------

    def _callback(self, source):

        # FAST-LIO2 estimates the IMU pose relative
        # to its initialization frame.
        #
        # camera_init -> body
        #
        # We adopt camera_init as the local odom frame.

        if (
            source.header.frame_id,
            source.child_frame_id
        ) != ("camera_init", "body"):

            self.get_logger().error(
                "Unexpected FAST-LIO2 frames"
            )

            return

        # Retrieve IMU mounting transform.
        mounting = self._read_mounting()

        if mounting is None:
            return

        t_root_imu, q_root_imu = mounting

        # ----------------------------------------------
        # 1. READ FAST-LIO2 ESTIMATED IMU POSE
        # ----------------------------------------------

        try:
            q_odom_imu = normalize(
                quaternion(
                    source.pose.pose.orientation
                )
            )
        except ValueError:
            self.get_logger().error(
                "Invalid FAST-LIO2 orientation"
            )
            return

        p_odom_imu = vector3(
            source.pose.pose.position
        )

        if not all(
            math.isfinite(v) for v in p_odom_imu
        ):
            return

        # ----------------------------------------------
        # 2. CONVERT IMU POSE TO ROBOT POSE
        # ----------------------------------------------

        # FAST-LIO2:
        # odom -> IMU
        #
        # URDF:
        # vrtk_link -> IMU
        #
        # Calculate:
        # odom -> vrtk_link

        # Robot orientation.
        q_odom_root = normalize(
            multiply(
                q_odom_imu,
                conjugate(q_root_imu)
            )
        )

        # Rotate the IMU mounting offset into odom.
        rotated_offset = rotate(
            q_odom_root,
            t_root_imu
        )

        # Subtract mounting offset from IMU position.
        p_odom_root = tuple(
            p_odom_imu[i] - rotated_offset[i]
            for i in range(3)
        )

        # ----------------------------------------------
        # 3. PUBLISH ROBOT TF
        # ----------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = source.header.stamp
        transform.header.frame_id = ODOM_FRAME

        transform.child_frame_id = ROOT_FRAME

        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ) = p_odom_root

        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        ) = q_odom_root

        self._tf_broadcaster.sendTransform(transform)

        # ----------------------------------------------
        # 4. CALCULATE VELOCITY
        # ----------------------------------------------

        # This FAST-LIO2 fork does not provide a usable
        # twist in its published Odometry message.
        #
        # Estimate velocity from successive robot poses.

        stamp = (
            source.header.stamp.sec
            + source.header.stamp.nanosec * 1e-9
        )

        if self._previous is None:

            self._previous = (
                stamp,
                p_odom_root,
                q_odom_root
            )

            return

        old_time, old_p, old_q = self._previous

        dt = stamp - old_time

        self._previous = (
            stamp,
            p_odom_root,
            q_odom_root
        )

        # Reject invalid or excessively delayed intervals.
        if not 0.0 < dt <= 0.5:
            self._filtered_twist = None
            return

        # Linear velocity in odom coordinates.
        v_world = tuple(
            (p_odom_root[i] - old_p[i]) / dt
            for i in range(3)
        )

        # Convert velocity into the robot frame.
        v_body = rotate(
            conjugate(q_odom_root),
            v_world
        )

        # Estimate angular velocity from rotation change.
        dq = normalize(
            multiply(
                conjugate(old_q),
                q_odom_root
            )
        )

        if dq[3] < 0.0:
            dq = tuple(-x for x in dq)

        xyz_norm = math.sqrt(
            sum(dq[i] ** 2 for i in range(3))
        )

        if xyz_norm < 1e-9:
            w_body = (0.0, 0.0, 0.0)
        else:
            w_body = tuple(
                dq[i]
                * 2.0
                * math.atan2(xyz_norm, dq[3])
                / (xyz_norm * dt)
                for i in range(3)
            )

        twist = (*v_body, *w_body)

        # Smooth the velocity estimates.
        if self._filtered_twist is not None:

            twist = tuple(
                self._alpha * new
                + (1.0 - self._alpha) * previous
                for new, previous in zip(
                    twist,
                    self._filtered_twist
                )
            )

        self._filtered_twist = twist

        # ----------------------------------------------
        # 5. PUBLISH /odom FOR NAV2
        # ----------------------------------------------

        odom = Odometry()

        odom.header = transform.header
        odom.child_frame_id = ROOT_FRAME

        # Estimated robot position.
        (
            odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z
        ) = p_odom_root

        # Estimated robot orientation.
        odom.pose.pose.orientation = (
            transform.transform.rotation
        )

        # Approximation: covariance is copied from
        # FAST-LIO2 without correcting for the IMU offset.
        odom.pose.covariance = list(
            source.pose.covariance
        )

        # Estimated linear and angular velocity.
        (
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            odom.twist.twist.angular.x,
            odom.twist.twist.angular.y,
            odom.twist.twist.angular.z
        ) = twist

        self._odom_pub.publish(odom)


def main():

    rclpy.init()

    node = FastLioOdomAdapter()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()