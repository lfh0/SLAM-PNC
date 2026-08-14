#!/usr/bin/env python3

import math
import os
import struct

import rospy
import yaml
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import OccupancyGrid
from tf.transformations import quaternion_from_euler


def read_pgm(path):
    with open(path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise ValueError("只支持 P2/P5 格式的 pgm: {}".format(path))

        tokens = []
        while len(tokens) < 3:
            line = f.readline()
            if not line:
                raise ValueError("pgm 文件头不完整: {}".format(path))
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())

        width = int(tokens[0])
        height = int(tokens[1])
        max_value = int(tokens[2])
        if max_value <= 0 or max_value > 65535:
            raise ValueError("pgm max_value 不合法: {}".format(max_value))

        pixel_count = width * height
        if magic == b"P5":
            if max_value < 256:
                data = list(f.read(pixel_count))
            else:
                raw = f.read(pixel_count * 2)
                data = list(struct.unpack(">{}H".format(pixel_count), raw))
        else:
            raw_tokens = []
            for line in f:
                line = line.split(b"#", 1)[0]
                raw_tokens.extend(line.split())
            data = [int(v) for v in raw_tokens[:pixel_count]]

        if len(data) != pixel_count:
            raise ValueError("pgm 像素数量不匹配，期望 {}，实际 {}".format(pixel_count, len(data)))

        if max_value != 255:
            data = [int(round(v * 255.0 / max_value)) for v in data]

        return width, height, data


def resolve_image_path(map_yaml_path, image_path):
    if os.path.isabs(image_path):
        return image_path
    return os.path.join(os.path.dirname(map_yaml_path), image_path)


def safe_yaw(origin):
    if len(origin) < 3:
        return 0.0
    yaw = float(origin[2])
    if math.isnan(yaw) or math.isinf(yaw):
        rospy.logwarn("地图 origin yaw 非法，已按 0.0 处理")
        return 0.0
    return yaw


def load_map_config():
    map_yaml_path = rospy.get_param("~map_yaml", "")
    config = {}

    if map_yaml_path:
        map_yaml_path = os.path.expanduser(map_yaml_path)
        with open(map_yaml_path, "r") as f:
            config = yaml.safe_load(f) or {}
        image_path = resolve_image_path(map_yaml_path, config["image"])
    else:
        image_path = rospy.get_param("~image")

    return {
        "image": os.path.expanduser(image_path),
        "resolution": float(rospy.get_param("~resolution", config.get("resolution", 0.05))),
        "origin": rospy.get_param("~origin", config.get("origin", [0.0, 0.0, 0.0])),
        "negate": int(rospy.get_param("~negate", config.get("negate", 0))),
        "occupied_thresh": float(rospy.get_param("~occupied_thresh", config.get("occupied_thresh", 0.65))),
    }


def build_obstacle_only_grid(config):
    width, height, pixels = read_pgm(config["image"])

    msg = OccupancyGrid()
    msg.header.frame_id = rospy.get_param("~frame_id", "map")
    msg.info.resolution = config["resolution"]
    msg.info.width = width
    msg.info.height = height

    origin = config["origin"]
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = 0.0
    quat = quaternion_from_euler(0.0, 0.0, safe_yaw(origin))
    msg.info.origin.orientation = Quaternion(*quat)

    data = []
    for y in range(height):
        image_y = height - y - 1
        row_start = image_y * width
        for x in range(width):
            pixel = pixels[row_start + x]
            if config["negate"]:
                occupied_prob = pixel / 255.0
            else:
                occupied_prob = (255 - pixel) / 255.0

            # 只保留障碍物；空闲和未知区域都发布成已知空闲 0。
            data.append(100 if occupied_prob > config["occupied_thresh"] else 0)

    msg.data = data
    return msg


def main():
    rospy.init_node("obstacle_only_map_publisher")
    publish_rate = float(rospy.get_param("~publish_rate", 1.0))
    latch = bool(rospy.get_param("~latch", True))

    config = load_map_config()
    grid = build_obstacle_only_grid(config)
    # 输出话题固定，仅输入话题允许由 launch 动态配置。
    topic = "/map"
    pub = rospy.Publisher("/map", OccupancyGrid, queue_size=1, latch=latch)

    rospy.loginfo(
        "发布障碍物地图: image=%s topic=%s size=%dx%d resolution=%.3f",
        config["image"],
        topic,
        grid.info.width,
        grid.info.height,
        grid.info.resolution,
    )

    if publish_rate <= 0.0:
        grid.header.stamp = rospy.Time.now()
        pub.publish(grid)
        rospy.spin()
        return

    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        grid.header.stamp = rospy.Time.now()
        pub.publish(grid)
        rate.sleep()


if __name__ == "__main__":
    main()
