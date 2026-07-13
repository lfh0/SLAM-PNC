#!/usr/bin/env python3

import math

import rospy
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped


def make_transform(parent, child, xyz, rpy_rad):
    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id = parent
    transform.child_frame_id = child
    transform.transform.translation.x = xyz[0]
    transform.transform.translation.y = xyz[1]
    transform.transform.translation.z = xyz[2]

    # yaml 中 odom_robo 后三位为角度制 roll、pitch、yaw。
    quat = tf.transformations.quaternion_from_euler(
        rpy_rad[0], rpy_rad[1], rpy_rad[2]
    )
    transform.transform.rotation.x = quat[0]
    transform.transform.rotation.y = quat[1]
    transform.transform.rotation.z = quat[2]
    transform.transform.rotation.w = quat[3]
    return transform


def main():
    rospy.init_node("static_tf_from_params")

    odom_robo = rospy.get_param("/lio/extrinsic/odom_robo", None)
    if odom_robo is None or len(odom_robo) != 6:
        rospy.logerr("参数 /lio/extrinsic/odom_robo 缺失或长度不是 6")
        return

    xyz = [float(v) for v in odom_robo[:3]]
    rpy_rad = [math.radians(float(v)) for v in odom_robo[3:6]]

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    broadcaster.sendTransform([
        make_transform("robot", "imu", xyz, rpy_rad),
        make_transform("map", "world", [0.0, 0.0, 2.0], [0.0, 0.0, 0.0]),
    ])

    rospy.loginfo(
        "发布静态 TF: map -> world, robot -> imu; odom_robo=%s",
        odom_robo,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
