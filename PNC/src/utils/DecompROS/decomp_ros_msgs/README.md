# decomp_ros_msgs

## 功能作用

`decomp_ros_msgs` 定义凸空间分解结果的 ROS 消息，用于在规划模块、数据记录工具和 RViz 插件之间传递椭球与凸多面体数组。本包仅生成消息类型，不包含节点、固定话题、服务或参数。

本工程的 `trajopt` 包会发布 `decomp_ros_msgs/PolyhedronArray`，`decomp_ros_utils` 则提供对应的 RViz 显示插件和 C++ 类型转换函数。

## 输入输出

| 方向 | 接口 | 说明 |
| --- | --- | --- |
| 数据输入 | 椭球中心/形状矩阵，或凸多面体各约束平面上的点和外法向量 | 由空间分解或轨迹优化模块填入 |
| 类型输出 | `decomp_ros_msgs/Ellipsoid` | 单个三维椭球 |
| 类型输出 | `decomp_ros_msgs/EllipsoidArray` | 带坐标系和时间戳的椭球数组 |
| 类型输出 | `decomp_ros_msgs/Polyhedron` | 由半空间交集表达的凸多面体 |
| 类型输出 | `decomp_ros_msgs/PolyhedronArray` | 带坐标系和时间戳的凸多面体数组 |

## 消息定义

### `Ellipsoid.msg`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `d` | `float64[3]` | 椭球中心坐标 |
| `E` | `float64[9]` | 按行展开的 `3 x 3` 形状矩阵 |

该工程中的几何库使用 `x = E * u + d`、`||u|| <= 1` 表示椭球。RViz 插件通过 `E` 的特征值和特征向量恢复椭球的轴长与方向，因此发布方应提供有限且符合该几何含义的矩阵。

### `EllipsoidArray.msg`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 数组的时间戳和坐标系 |
| `ellipsoids` | `Ellipsoid[]` | 椭球列表 |

### `Polyhedron.msg`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `points` | `geometry_msgs/Point[]` | 每个约束平面上的一个点 `p` |
| `normals` | `geometry_msgs/Point[]` | 对应平面的外法向量 `n` |

`points[i]` 与 `normals[i]` 必须一一对应。几何库采用 `n.dot(x - p) <= 0` 表示多面体内部，所有半空间的交集形成最终凸多面体。

### `PolyhedronArray.msg`

| 字段 | 类型 | 含义 |
| --- | --- | --- |
| `header` | `std_msgs/Header` | 数组的时间戳和坐标系 |
| `polyhedrons` | `Polyhedron[]` | 凸多面体列表 |

## 构建与检查

消息由 `catkin_simple()` 自动扫描 `msg/` 并生成：

```bash
catkin_make --pkg catkin_simple decomp_ros_msgs
source devel/setup.bash
rosmsg show decomp_ros_msgs/PolyhedronArray
rosmsg show decomp_ros_msgs/EllipsoidArray
```

本包不决定消息发布到哪个话题；话题名由使用这些消息的上层节点配置或定义。
