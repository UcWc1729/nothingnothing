# ROBOCON2026 武林探秘 - MuJoCo仿真场景

第二十五届全国大学生机器人大赛ROBOCON"武林探秘"竞技赛MuJoCo仿真环境。

[English Document](README.md)

<img src="assets/family.png" alt="场景预览" style="zoom:50%;" />


## 快速开始

### 环境要求
- Python >= 3.7
- MuJoCo物理引擎

### 安装依赖
```bash
pip install mujoco
```

### 启动仿真
```bash
# 请检查自己的mujoco版本
pip list | grep mujoco
# 如果是mujoco>=3.3.0 请使用以下的指令
python -m mujoco.viewer --mjcf models/mjcf/robocon2026.xml
# 如果是3.3.0之前的版本 则使用
python -m mujoco.viewer --mjcf models/mjcf/robocon2026_old.xml
```

## 激光雷达模拟

1. 请先安装 **MuJoCo-LiDAR**。PyPI 上的 `mujoco-lidar` **不包含** CPU/Taichi 后端实现（`core_cpu`、`core_ti`），**必须从源码安装**后才能运行本项目的 LiDAR 仿真：

   ```bash
   # 从源码安装（CPU 与 GPU 后端均可用）
   pip3 install mujoco numpy etils
   pip3 install taichi   # 若要用 GPU 后端
   git clone https://github.com/TATP-233/MuJoCo-LiDAR.git
   cd MuJoCo-LiDAR
   pip3 install -e ".[taichi]"
   cd -
   ```

   安装完成后默认用 CPU 后端即可运行。若要用 **GPU 后端**（更快），需配置 CUDA 环境：

   **LiDAR GPU 后端（CUDA）配置**

   - **1. 确认 NVIDIA 驱动与 libcuda**  
     终端执行：`ls /usr/lib/x86_64-linux-gnu/libcuda.so*` 或 `nvidia-smi`。若没有，需先安装 [NVIDIA 驱动](https://www.nvidia.com/Download/index.aspx)（或系统“附加驱动”里选专有驱动）。
   - **2. 安装支持 CUDA 且版本 ≥ 2.3 的 PyTorch**  
     Taichi 后端的 BVH 会用到 PyTorch，且需 2.3+ 才支持 `u32`。例如（按你本机 CUDA 版本选其一）：
     ```bash
     # 若本机为 CUDA 12.x（推荐）
     pip3 install torch --index-url https://download.pytorch.org/whl/cu121
     # 若为 CUDA 11.8
     pip3 install torch --index-url https://download.pytorch.org/whl/cu118
     ```
     安装后执行：`python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"`，应显示版本 ≥2.3 且 `True`。
   - **3. 运行 GPU 后端**  
     在本项目目录下：
     ```bash
     python3 src/lidar_sim_native.py --backend gpu
     python3 src/lidar_sim_ros2.py --backend gpu
     ```
     若仍报 `libcuda.so not found`，请确认驱动安装后重启，或检查 `LD_LIBRARY_PATH` 是否包含驱动库路径。

```bash
cd ROBOCON2026_Scene
# 默认 CPU 后端（无需 CUDA）
python3 src/lidar_sim_native.py
# 或显式指定
python3 src/lidar_sim_native.py --backend cpu
python3 src/lidar_sim_native.py --backend gpu   # 需已配置 CUDA + PyTorch>=2.3
```

<img src="./assets/lidar_sim_native.png" alt="image-lidar_sim" style="zoom:50%;" />


2. 集成 ROS2，需要先安装好 ROS2 环境，且 mujoco_lidar 需已从源码安装（见上文）。

```bash
cd ROBOCON2026_Scene
python3 src/lidar_sim_ros2.py
# 或 CPU 后端
python3 src/lidar_sim_ros2.py --backend cpu
```

<img src="./assets/lidar_sim_ros2.png" alt="image-lidar_sim" style="zoom:50%;" />

## 机器人运动控制模拟

我们基于强化学习训练的 ONNX 策略模型，提供了多款主流机器人的运动控制仿真，支持手柄实时交互控制和 ROS2 接口集成。目前支持的机器人包括：

- **宇树科技**：Go1 四足机器人、G1 人形机器人
- **Booster机器人**：T1 双足人形机器人
- **逐级动力**：Tron A1 双足机器人

### 环境准备

在运行任何机器人控制程序前，请先安装必要的依赖：

```bash
# 安装 ONNX 运行时和游戏手柄支持
pip install onnxruntime pygame etils

# （可选）安装 ROS2 支持，用于话题发布
# 请参考 ROS2 官方文档安装对应版本
```

### Unitree Go2 四足机器人

<!-- Go2 演示图片 -->
<img src="assets/go2_demo.png" alt="Go2 运动演示" style="zoom:50%;" />

```bash
# 手柄控制模式（推荐使用 Xbox 手柄）
python3 src/robots/play_go2_joystick.py

# ROS2 话题模式（需要先安装 ROS2）
python3 src/robots/play_go2_ros2.py

# 雷达默认 mid360；可选 --lidar airy（airy96）、hdl64、vlp32、os128
python3 src/robots/play_go2_ros2.py
python3 src/robots/play_go2_ros2.py --lidar airy
```

**控制说明**：
- `左摇杆`：前后左右移动
- `右摇杆`：原地旋转
- `Backspace`: 重置环境

### Unitree Go1 四足机器人

<!-- Go1 演示图片 -->
<img src="assets/go1_demo.png" alt="Go1 运动演示" style="zoom:50%;" />

```bash
# 手柄控制模式
python3 src/robots/play_go1_joystick.py

# ROS2 话题模式
python3 src/robots/play_go1_ros2.py
```

### Unitree G1 人形机器人

<!-- G1 演示图片 -->
<img src="assets/g1_demo.png" alt="G1 运动演示" style="zoom:50%;" />

```bash
# 手柄控制模式
python3 src/robots/play_g1_joystick.py

# ROS2 话题模式
python3 src/robots/play_g1_ros2.py
```

### Booster T1 双足人形机器人

<!-- T1 演示图片 -->
<img src="assets/t1_demo.png" alt="T1 运动演示" style="zoom:50%;" />

```bash
# 手柄控制模式
python3 src/robots/play_t1_joystick.py
```

### Tron 双足机器人

<!-- A1 演示图片 -->
<img src="assets/tron_demo.png" alt="Tron 运动演示" style="zoom:50%;" />

```bash
# 手柄控制模式
python3 src/robots/play_tron_joystick.py
```

玩的开心！

### 常见问题

**Q: 手柄无法识别？**  
A: 请确保手柄已连接并安装 `pygame`。运行 `python -m pygame.examples.joystick` 测试手柄连接。

**Q: 如何自定义机器人模型？**  
A: 修改 `models/mjcf/` 目录下对应的 XML 文件，并重新训练策略模型。

## 使用 FastlIO2 进行 SLAM

1. 默认使用 ROS2 humble，如版本不同，请参照 `build_fastlio2.sh` 中的设置进行修改

2. 赋予执行权限

   ```bash
   chmod +x build_fastlio2.sh
   ```

3. 运行编译脚本

   ```bash
   ./build_fastlio2.sh
   ```

4. 加载环境

   ```bash
   cd ros2_ws
   source install/setup.bash
   ```

5. 启动 Go2 仿真环境（在新终端）

   ```bash
   python src/robots/play_go2_ros2.py 
   ```

6. 开启键盘控制节点（在新终端）

   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard 
   ```

7. 最后运行 FastLIO2（在当前终端）

   ```bash
   ros2 launch fast_lio mapping.launch.py config_file:=go2_fastlio2.yaml
   ```

**一键启动（可选）**：若已按上述步骤完成环境配置，可直接用脚本一次性启动 Go2 仿真、键盘控制节点和 FastLIO2 建图，按 Ctrl+C 结束全部：

   ```bash
   chmod +x run_go2_fastlio2.sh
   ./run_go2_fastlio2.sh
   ```

<img src="./assets/image-20251119221104659.png" alt="image-20251119221104659" style="zoom:80%;" />

> [!note]
>
> 如果需要适配其它机器人，根据模型文件，在 `ros2_ws/src/FAST_LIO/config` 中的配置文件里，修改 imu 和 lidar 之间的外参即可
>
> ```yaml
> extrinsic_T: [0.32057, 0.0, -0.11732]
> extrinsic_R: [-0.9802,    0.,     0.1980,
>               0.,         1.,     0.,
>               -0.1980,    0.,     -0.9802]
> ```
>
> 此外，如需支持键盘遥控，参考 `src/robots/play_go2_ros2.py` 以修改相应的代码

**若 RViz/FastLIO 没有点云**：
1. 确认已用**最新配置**：`cp -r src/fastlio2_config/* ros2_ws/src/FAST_LIO/config/` 后重新 `colcon build --packages-select fast_lio`（或重跑 `./build_fastlio2.sh`），否则 FastLIO 仍用旧 `lidar_type` 会报 No Effective Points。
2. 在 Go2 仿真和 FastLIO 都启动后，另开终端执行：`source ros2_ws/install/setup.bash && ros2 topic hz /lidar_points`，若有输出（如 10–20 Hz）说明仿真在发点云；若无，检查 Go2 仿真窗口是否正常运行、是否有 “LiDAR 发布点数” 的日志。
3. RViz 中 Fixed Frame 选 `camera_init`（一键脚本会发布静态 TF `map`→`camera_init`，故下拉里必有此项）；或选 `lidar`。添加 PointCloud2，Topic 选 `/lidar_points` 或 FastLIO 的 `/Laser_map`。

**雷达位置与量程（Go2 mid360）**：
- **位置**（相对机身 base）：`pos="0.295 0 -0.12"` → 前 29.5 cm、向下 12 cm（已调低，便于打到地面/场景）。修改 `models/robots/unitree_go2/go2_mjx_fullcollisions.xml` 中 `<site name="lidar" ...>` 可改高度。
- **距离范围**：`lidar_min_range=0.01 m`、`lidar_max_range=100 m`（代码里可调）；射线最大距离 100 m（MuJoCo 后端 cutoff）。
- **角度范围**：由 `mid360.npy` 决定，约 360° 水平；垂直方向视 npy 而定。若觉得“雷达太高”打不到地，可把 lidar 的 `pos` 的 z 再调负（如 `-0.15`）。

## 文件结构

```
ROBOCON2026_Scene/
├── README.md                       # 英文说明文档
├── README_zh.md                    # 中文说明文档（本文件）
├── build_fastlio2.sh               # FastLIO2 一键编译脚本
├── run_go2_fastlio2.sh             # 一键启动 Go2 + 键盘控制 + FastLIO2 建图
├── check_env.sh                    # 环境与配置检查脚本
├── sync_fastlio2_config.sh         # 同步 FastLIO2 配置并重新编译
├── assets/                         # 资源文件
├── models/
│   ├── meshes/                     # 3D 模型文件
│   │   ├── kfs/                    # 武功秘籍模型
│   │   ├── robocon2026.obj         # 主场景模型
│   │   ├── robocon2026.mtl         # 材质文件
│   │   ├── parts/                  # 场景部件模型
│   │   └── visual/                 # 可视化资源
│   └── mjcf/                       # MuJoCo XML 场景文件
│       ├── robocon2026.xml         # 主场景（MuJoCo >= 3.3.0）
│       ├── robocon2026_old.xml     # 兼容旧版本 MuJoCo
│       ├── mocap_env.xml           # 激光雷达仿真场景
│       ├── kfs.xml                 # 武功秘籍场景
│       ├── kfs_dep.xml             # 武功秘籍资产依赖
│       ├── scene_go1.xml           # Go1 机器人场景
│       ├── scene_g1.xml            # G1 机器人场景
│       ├── scene_t1.xml            # T1 机器人场景
│       └── scene_a1.xml            # A1 机器人场景
├── src/
│   ├── fastlio2_config/            # FastLIO2 配置文件（如 go2_fastlio2.yaml）
│   ├── lidar_sim_native.py         # 激光雷达仿真（本地）
│   ├── lidar_sim_ros2.py           # 激光雷达仿真（ROS2）
│   ├── robots/                     # 机器人控制脚本
│   │   ├── play_go1_joystick.py    # Go1 手柄控制
│   │   ├── play_go1_ros2.py        # Go1 ROS2 接口
│   │   ├── play_g1_joystick.py     # G1 手柄控制
│   │   ├── play_g1_ros2.py         # G1 ROS2 接口
│   │   ├── play_t1_joystick.py     # T1 手柄控制
│   │   ├── play_tron_joystick.py   # Tron 手柄控制
│   │   ├── play_go2_joystick.py    # Go2 手柄控制
│   │   ├── play_go2_ros2.py        # Go2 ROS2 接口
│   │   ├── gamepad_reader.py       # 游戏手柄读取模块
│   │   ├── camera_utils.py         # 相机工具模块
│   │   └── onnx/                   # ONNX 策略模型文件
│   └── rviz_config/                # RViz 配置文件
│       ├── g1.rviz
│       ├── go1.rviz
│       └── lidar.rviz
├── ros2_ws/                        # ROS2 工作空间（Livox 驱动、FastLIO 等）
└── 第二十五届全国大学生机器人大赛ROBOCON_u201C武林探秘_u201D竞技赛规则V.1.pdf
```

## 相关链接
- [ROBOCON官网](http://robocon.org.cn/sys-index/)
- [MuJoCo文档](https://mujoco.readthedocs.io/)

## 引用

本仓库使用的技术栈基于我们的仿真器 [DISCOVERSE](https://air-discoverse.github.io/)。若本工作对您的研究有帮助，欢迎引用我们的论文：

```bibtex
@article{jia2025discoverse,
    title={DISCOVERSE: Efficient Robot Simulation in Complex High-Fidelity Environments},
    author={Yufei Jia and Guangyu Wang and Yuhang Dong and Junzhe Wu and Yupei Zeng and Haonan Lin and Zifan Wang and Haizhou Ge and Weibin Gu and Chuxuan Li and Ziming Wang and Yunjie Cheng and Wei Sui and Ruqi Huang and Guyue Zhou},
    journal={arXiv preprint arXiv:2507.21981},
    year={2025},
    url={https://arxiv.org/abs/2507.21981}
}
```

## 致谢

感谢以下项目和贡献者：

- 感谢重庆邮电大学开源的[场景 Blender 模型](https://rcbbs.top/t/topic/2261)
- 感谢 DeepMind [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground) 提供的机器人运动控制策略和实现参考
- 感谢香港大学 MARS 实验室开源的 [FAST_LIO](https://github.com/hku-mars/FAST_LIO) 激光雷达惯性里程计算法
- 感谢 Livox 团队提供的 [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) 和 [livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) 驱动支持

