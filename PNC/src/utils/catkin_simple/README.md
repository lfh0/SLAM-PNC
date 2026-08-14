# catkin_simple

## 功能作用

`catkin_simple` 是工程内置的 Catkin CMake 辅助包，用较少的样板代码完成依赖发现、消息生成、目标链接、安装和包导出。`decomp_ros_msgs` 与 `decomp_ros_utils` 的构建脚本使用了它。

该包属于构建期工具，不参与机器人运行时数据链路，因此没有 ROS 节点、话题、服务、动作或运行参数。

## 输入输出

| 方向 | 内容 | 说明 |
| --- | --- | --- |
| 输入 | 使用方的 `package.xml` | 读取构建依赖和运行依赖 |
| 输入 | `CMakeLists.txt` 中的 `catkin_simple()` 与 `cs_*` 宏 | 定义可执行文件、库、安装项和导出项 |
| 输入 | 约定目录 `include/`、`msg/`、`srv/`、`action/`、`cfg/` 等 | 自动发现头文件、消息/服务/动作和动态配置 |
| 输出 | Catkin CMake 配置 | 为使用方填充 include、依赖和导出信息 |
| 输出 | 消息/服务/动作及动态配置生成目标 | 在对应依赖存在时自动创建 |
| 输出 | 自动链接的可执行文件和库 | 默认链接 `${catkin_LIBRARIES}` 并添加生成目标依赖 |
| 输出 | 安装规则 | 安装目标、头文件、脚本及常用共享资源目录 |

## 提供的宏

| 宏 | 作用 |
| --- | --- |
| `catkin_simple()` | 初始化包，读取依赖，加入头文件路径，并扫描消息/服务/动作/动态配置 |
| `cs_add_executable()` | 创建可执行目标，默认自动链接 Catkin 库并添加依赖 |
| `cs_add_library()` | 创建并记录需要导出的库目标 |
| `cs_add_targets_to_package()` | 将已有目标加入包级聚合目标 |
| `cs_install()` | 安装已登记目标、头文件和常用资源目录 |
| `cs_install_scripts()` | 将脚本安装到包的可执行目录 |
| `cs_export()` | 调用 `catkin_package()` 导出 include、库和依赖 |

## 使用示例

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(example_package)

find_package(catkin_simple REQUIRED)
catkin_simple()

cs_add_library(example_library src/example_library.cpp)
cs_add_executable(example_node src/example_node.cpp)

cs_install()
cs_export()
```

依赖应写在使用方的 `package.xml` 中；`catkin_simple()` 会据此调用 `find_package()`。如需任一构建依赖缺失时立即报错，可使用 `catkin_simple(ALL_DEPS_REQUIRED)`；该选项不改变运行依赖在 `cs_export()` 中的静默查找行为。

## 构建

```bash
catkin_make --pkg catkin_simple
source devel/setup.bash
```

本目录是工程携带的第三方基础包。除非需要修复构建机制，不应在业务功能开发中修改其宏行为。
