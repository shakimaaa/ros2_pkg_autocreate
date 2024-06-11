# ros2_pkg_autocreate
# 自动创建 ROS 2 功能包脚本

**作者:** Shakima

**运行环境:** Ubuntu 22.04 / ROS2 Humble (未在其他环境下测试)

这是一个构建ros2功能包并自动创建基本格式的节点文件和附加文件，并配置ament_cmake_auto格式的cmake文件的脚本，这样可以快速设置和启动 ROS 2 项目。

本人在开始一个项目时，总是要重复的写一些固定的内容和配置，由于途径的限制没有找到一些简便的解决方法。所以自己动手写了一个脚本。我不清楚这是不是一个聪明的办法，但目前使用起来还不错。

**运行脚本后会在功能包内创建一个scripts 目录，并将生成附加文件的脚本文件移动到 scripts 目录。** 

## 使用方法

### 步骤 1: 准备环境
确保您的系统中已安装 ROS 2，并配置好相应的环境。

### 步骤 2: 创建工作空间
在本地机器上创建一个 ROS 工作空间（如果尚未存在）：

```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```

### 步骤 3: 克隆项目

```bash
git clone https://github.com/shakimaaa/ros2_pkg_autocreate.git
```

### 步骤 4: 赋予权限

```bash
chmod +x create_ros2_pkg.sh
```

### 步骤 5: 运行脚本
进入工作空间

```bash
~/create_ros2_pkg.sh <pkg_name> <example_node>
```
-- <pkg_name> 功能包名字
-- <example_node> 节点文件名字，只需修改'example'即可，会自动将类名命名为'ExampleNode'

### 步骤 6: 创建附加文件
如果需要创建节点文件之外的'hpp','cpp'文件,进入新创建的功能包运行下面的指令

```bash
cd ~/ros_ws/src/<pkg_name>
python3 scripts/create_additional_files.py $(pwd) <pkg_name> <header_file_name> <source_file_name> <class_name>
```

-- <pkg_name> 功能包名字
-- <header_file_name> 头文件名字
-- <source_file_name> 源文件名字
-- <class_name> 类的名字
