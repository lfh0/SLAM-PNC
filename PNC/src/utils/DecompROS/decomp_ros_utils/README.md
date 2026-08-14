# decomp_ros_utils

## 功能作用

`decomp_ros_utils` 提供二维/三维凸空间分解算法、几何数据结构、ROS 消息转换函数和 RViz 显示插件。它主要用于根据离散路径与障碍物点生成无碰撞凸安全走廊，并把安全走廊转换为 `decomp_ros_msgs` 供 ROS 节点发布和可视化。

本包是库和 RViz 插件，不提供独立 ROS 节点、launch 文件、固定话题、服务或运行参数。

## 算法输入输出

| 方向 | 数据/API | 说明 |
| --- | --- | --- |
| 输入 | `vec_Vecf<Dim>` 障碍物点 | 通过 `set_obs()` 设置，可选局部包围盒会先筛除远处障碍物 |
| 输入 | `vec_Vecf<Dim>` 离散路径 | 传给 `EllipsoidDecomp::dilate()` 或 `IterativeDecomp::dilate_iter()` |
| 输入 | 局部/全局包围盒、长轴偏移、迭代次数和降采样分辨率 | 控制走廊范围与路径简化过程 |
| 输出 | `get_ellipsoids()` | 沿各路径线段膨胀得到的椭球序列 |
| 输出 | `get_polyhedrons()` | 障碍物分离平面构成的凸安全走廊 |
| 输出 | `get_constraints()` | 与安全走廊等价的线性约束 `A*x <= b` |
| 输出 | `get_path()` | 实际用于分解的路径 |

主要算法类：

- `EllipsoidDecomp2D` / `EllipsoidDecomp3D`：逐路径段生成椭球和凸多面体。
- `IterativeDecomp2D` / `IterativeDecomp3D`：反复分解并移除冗余路径点。
- `SeedDecomp2D` / `SeedDecomp3D`：围绕单个种子点用给定半径膨胀。
- `Polyhedron`、`Ellipsoid`、`Hyperplane` 和 `LinearConstraint`：基础几何数据结构与包含关系计算。

## ROS 类型转换

头文件 `include/decomp_ros_utils/data_ros_utils.h` 提供以下输入输出转换：

| 输入 | 输出 | 函数 |
| --- | --- | --- |
| Eigen 二维/三维点序列 | `nav_msgs/Path` | `vec_to_path()` |
| 三维点序列 | `sensor_msgs/PointCloud` | `vec_to_cloud()` |
| `sensor_msgs/PointCloud` | 三维点序列 | `cloud_to_vec()` |
| `decomp_ros_msgs/Polyhedron` 或数组 | `Polyhedron3D` 或数组 | `ros_to_polyhedron()`、`ros_to_polyhedron_array()` |
| `Polyhedron2D/3D` 或数组 | `decomp_ros_msgs/Polyhedron` 或数组 | `polyhedron_to_ros()`、`polyhedron_array_to_ros()` |
| 椭球数组 | `decomp_ros_msgs/EllipsoidArray` | `ellipsoid_array_to_ros()` |

这些转换函数不会自动填写消息的 `header.stamp` 和 `header.frame_id`，发布方需要在发送前补齐。

二维几何转成三维 ROS 消息时有以下固定处理：

- `Polyhedron2D` 会额外增加位于 `z=0.01` 和 `z=-0.01` 的两个约束平面，使可视化结果成为总厚度 `0.02 m` 的薄片；
- 二维椭球的 `d.z` 设为 `0`，形状矩阵 `E` 的 Z 行和 Z 列设为 `0`。

## RViz 输入输出

编译后会生成共享库 `libdecomp_rviz_plugins`，并注册两个 RViz Display：

| 插件 | 订阅输入 | 可视化输出 |
| --- | --- | --- |
| `decomp_rviz_plugins/EllipsoidArray` | 用户在 RViz 中选择的话题，类型为 `decomp_ros_msgs/EllipsoidArray` | 椭球的位置、尺寸、方向、颜色和透明度 |
| `decomp_rviz_plugins/PolyhedronArray` | 用户在 RViz 中选择的话题，类型为 `decomp_ros_msgs/PolyhedronArray` | 凸多面体表面、边界或平面法向量 |

插件使用消息 `header.frame_id` 经 TF 转换到 RViz Fixed Frame，因此发布时必须设置有效坐标系，并保证对应 TF 可用。

使用步骤：

1. 启动 RViz，点击 **Add**。
2. 选择 `decomp_rviz_plugins/PolyhedronArray` 或 `decomp_rviz_plugins/EllipsoidArray`。
3. 在插件属性中选择对应消息话题并设置颜色、透明度及显示方式。

## 在本工程中的位置

`trajopt` 使用本包的多面体几何结构与 `polyhedron_array_to_ros()`，将轨迹安全走廊发布到 `/trajopt/polyhedrons`。`navi_map/rviz/navi_2d.rviz` 已配置 `PolyhedronArray` 显示项。

## 构建

```bash
catkin_make --pkg catkin_simple decomp_ros_msgs decomp_ros_utils
source devel/setup.bash
```

编译依赖包括 Eigen、ROS、RViz、Qt、`decomp_ros_msgs` 和 `catkin_simple`。
