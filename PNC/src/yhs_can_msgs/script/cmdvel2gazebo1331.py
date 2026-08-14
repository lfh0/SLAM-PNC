#!/usr/bin/env python3

import math

import rospy
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd, steering_ctrl_cmd


class CmdVelToYhsCan:
    def __init__(self):
        rospy.init_node("cmdvel2gazebo1")

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.steering_cmd_topic = rospy.get_param("~steering_cmd_topic", "/steering_ctrl_cmd")
        self.ctrl_cmd_topic = rospy.get_param("~ctrl_cmd_topic", "/ctrl_cmd")

        self.steering_gear = rospy.get_param(
            "~steering_gear", rospy.get_param("~gear", 5)
        )
        self.spin_gear = rospy.get_param("~spin_gear", 6)
        self.max_speed = rospy.get_param("~max_speed", 1.2)
        self.max_steer_deg = rospy.get_param("~max_steer_deg", 25.0)
        self.max_steer_rate_deg = rospy.get_param("~max_steer_rate_deg", 35.0)
        self.steer_sign = self.normalized_sign(rospy.get_param("~steer_sign", 1.0))
        self.spin_sign = self.normalized_sign(rospy.get_param("~spin_sign", self.steer_sign))
        self.cmd_timeout = rospy.Duration(rospy.get_param("~cmd_timeout", 0.5))
        self.curvature_speed_limit = rospy.get_param("~curvature_speed_limit", True)
        self.spin_linear_epsilon = rospy.get_param("~spin_linear_epsilon", 0.03)
        self.spin_angular_epsilon = rospy.get_param("~spin_angular_epsilon", 0.05)
        self.spin_max_z_deg = rospy.get_param("~spin_max_z_deg", 22.0)

        self.last_cmd_time = rospy.Time(0)
        self.last_cmd = Twist()
        self.last_steer_deg = 0.0
        self.last_steer_time = rospy.Time.now()

        self.steering_pub = rospy.Publisher(
            self.steering_cmd_topic, steering_ctrl_cmd, queue_size=10
        )
        self.ctrl_pub = rospy.Publisher(self.ctrl_cmd_topic, ctrl_cmd, queue_size=10)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_callback, queue_size=1)

        rospy.loginfo(
            "cmdvel2gazebo1 real-car bridge ready: %s -> %s/%s, "
            "steering_gear=%s, spin_gear=%s, steer_sign=%s, spin_sign=%s",
            self.cmd_vel_topic,
            self.steering_cmd_topic,
            self.ctrl_cmd_topic,
            self.steering_gear,
            self.spin_gear,
            self.steer_sign,
            self.spin_sign,
        )

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def spin(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            legacy_gear = rospy.get_param("~gear", self.steering_gear)
            self.steering_gear = rospy.get_param("~steering_gear", legacy_gear)
            self.spin_gear = rospy.get_param("~spin_gear", self.spin_gear)
            self.max_steer_rate_deg = rospy.get_param(
                "~max_steer_rate_deg", self.max_steer_rate_deg
            )
            self.spin_max_z_deg = rospy.get_param(
                "~spin_max_z_deg", self.spin_max_z_deg
            )
            self.steer_sign = self.normalized_sign(
                rospy.get_param("~steer_sign", self.steer_sign)
            )
            self.spin_sign = self.normalized_sign(
                rospy.get_param("~spin_sign", self.spin_sign)
            )
            if rospy.Time.now() - self.last_cmd_time > self.cmd_timeout:
                self.publish_stop()
            else:
                self.publish_cmd(self.last_cmd)
            rate.sleep()

    def publish_cmd(self, twist):
        if self.should_spin_in_place(twist):
            self.publish_spin_cmd(twist)
        else:
            self.publish_steering_cmd(twist)

    def should_spin_in_place(self, twist):
        return (
            abs(twist.linear.x) <= self.spin_linear_epsilon
            and abs(twist.linear.y) <= self.spin_linear_epsilon
            and abs(twist.angular.z) >= self.spin_angular_epsilon
        )

    def publish_spin_cmd(self, twist):
        msg = ctrl_cmd()
        self.set_field(msg, ("ctrl_cmd_gear",), int(self.spin_gear))
        self.set_field(msg, ("ctrl_cmd_x_linear", "ctrl_cmd_linear"), 0.0)
        self.set_field(
            msg,
            ("ctrl_cmd_z_angular", "ctrl_cmd_angular"),
            self.limit(
                self.spin_sign * math.degrees(twist.angular.z),
                -self.spin_max_z_deg,
                self.spin_max_z_deg,
            ),
        )
        self.set_field(msg, ("ctrl_cmd_y_linear", "ctrl_cmd_slipangle"), 0.0)
        self.ctrl_pub.publish(msg)

    def publish_steering_cmd(self, twist):
        speed = self.limit(twist.linear.x, -self.max_speed, self.max_speed)
        steer_deg = self.steer_sign * math.degrees(twist.angular.z)
        steer_deg = self.limit(steer_deg, -self.max_steer_deg, self.max_steer_deg)
        steer_deg = self.limit_steer_rate(steer_deg)

        if self.curvature_speed_limit and abs(steer_deg) > 1e-3:
            ratio = math.tan(math.radians(self.max_steer_deg) + 0.1)
            ratio /= math.tan(math.radians(abs(steer_deg)) + 0.1)
            speed_limit = self.max_speed * math.sqrt(max(0.0, ratio))
            speed = self.limit(speed, -speed_limit, speed_limit)

        msg = steering_ctrl_cmd()
        self.set_field(msg, ("ctrl_cmd_gear",), int(self.steering_gear))
        self.set_field(msg, ("steering_ctrl_cmd_velocity",), speed)
        self.set_field(msg, ("steering_ctrl_cmd_steering",), steer_deg)
        self.set_field(msg, ("steering_ctrl_cmd_slipangle",), 0.0)
        self.steering_pub.publish(msg)

    def limit_steer_rate(self, target_deg):
        if self.max_steer_rate_deg <= 0.0:
            self.last_steer_deg = target_deg
            self.last_steer_time = rospy.Time.now()
            return target_deg

        now = rospy.Time.now()
        dt = (now - self.last_steer_time).to_sec()
        if dt <= 0.0 or dt > 1.0:
            self.last_steer_deg = target_deg
            self.last_steer_time = now
            return target_deg

        max_delta = self.max_steer_rate_deg * dt
        delta = self.limit(target_deg - self.last_steer_deg, -max_delta, max_delta)
        self.last_steer_deg += delta
        self.last_steer_time = now
        return self.last_steer_deg

    def publish_stop(self):
        steering = steering_ctrl_cmd()
        self.set_field(steering, ("ctrl_cmd_gear",), int(self.steering_gear))
        self.set_field(steering, ("steering_ctrl_cmd_velocity",), 0.0)
        self.set_field(steering, ("steering_ctrl_cmd_steering",), 0.0)
        self.set_field(steering, ("steering_ctrl_cmd_slipangle",), 0.0)
        self.steering_pub.publish(steering)

        ctrl = ctrl_cmd()
        self.set_field(ctrl, ("ctrl_cmd_gear",), int(self.spin_gear))
        self.set_field(ctrl, ("ctrl_cmd_x_linear", "ctrl_cmd_linear"), 0.0)
        self.set_field(ctrl, ("ctrl_cmd_z_angular", "ctrl_cmd_angular"), 0.0)
        self.set_field(ctrl, ("ctrl_cmd_y_linear", "ctrl_cmd_slipangle"), 0.0)
        self.ctrl_pub.publish(ctrl)

    @staticmethod
    def set_field(msg, names, value):
        for name in names:
            if hasattr(msg, name):
                setattr(msg, name, value)
                return

    @staticmethod
    def limit(value, lower, upper):
        return max(lower, min(upper, value))

    @staticmethod
    def normalized_sign(value):
        try:
            return -1.0 if float(value) < 0.0 else 1.0
        except (TypeError, ValueError):
            return 1.0


if __name__ == "__main__":
    try:
        CmdVelToYhsCan().spin()
    except rospy.ROSInterruptException:
        pass
