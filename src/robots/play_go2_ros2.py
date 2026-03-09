# Copyright 2025 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
"""Deploy an MJX policy in ONNX format to C MuJoCo and play with it."""

import os
import sys
# 优先使用从源码安装的 mujoco_lidar（含 core_ti/core_cpu）
_script_dir = os.path.dirname(os.path.abspath(__file__))
_mujoco_lidar_build = os.path.join(_script_dir, "..", "..", ".mujoco_lidar_build")
if os.path.isdir(_mujoco_lidar_build):
    _build_path = os.path.abspath(_mujoco_lidar_build)
    if _build_path not in sys.path:
        sys.path.insert(0, _build_path)

from etils import epath
import mujoco
import mujoco.viewer as viewer
import argparse
import numpy as np
import threading
# import taichi as ti
from scipy.spatial.transform import Rotation

from camera_utils import camera2k, get_site_tmat
from play_go2_joystick import OnnxController

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import Image, CameraInfo, Imu, PointCloud2, PointField

from mujoco_lidar import MjLidarWrapper
from mujoco_lidar import scan_gen

_HERE = epath.Path(__file__).parent
_ONNX_DIR = _HERE / "onnx"
_MJCF_PATH = _HERE.parent.parent / "models" / "mjcf" / "scene_go2.xml"

_JOINT_NUM = 12
class OnnxControllerRos2(OnnxController, Node):
    """ONNX controller for the Go-2 robot."""

    def __init__(
        self,
        mj_model: mujoco.MjModel,
        policy_path: str,
        default_angles: np.ndarray,
        n_substeps: int,
        action_scale: float = 0.5,
        lidar_type: str = "mid360",
        lidar_backend: str = "cpu",
        #* add
        lidar_min_range: float = 0.2,
        lidar_max_range: float = 100.0,
        enable_camera: bool = True,
        tf_rate: float = 10.0,
        imu_rate: float = 200.0,
        image_rate: float = 20.0,
        lidar_rate: float = 20.0,
        caminfo_rate: float = 1.0,
    ):
        super().__init__(
            policy_path,
            default_angles,
            n_substeps,
            action_scale
        )
        Node.__init__(self, 'robocon_go2_node')

        self.camera_width = 640
        self.camera_height = 480
        self._camera_name = "head_camera"
        self._renderer = mujoco.Renderer(mj_model, height=self.camera_height, width=self.camera_width)
        
        self.enable_camera = enable_camera

        self.tf_rate = tf_rate
        self.imu_rate = imu_rate
        self.image_rate = image_rate
        self.lidar_rate = lidar_rate
        self.caminfo_rate = caminfo_rate

        self.init_topic_publisher(mj_model)

        # lidar
        self.lidar_min_range = lidar_min_range
        self.lidar_max_range = lidar_max_range
        self.dynamic_lidar = False
        if lidar_type == "airy":
            self.rays_theta, self.rays_phi = scan_gen.generate_airy96()
        elif lidar_type == "hdl64":
            self.rays_theta, self.rays_phi = scan_gen.generate_HDL64()
        elif lidar_type == "vlp32":
            self.rays_theta, self.rays_phi = scan_gen.generate_vlp32()
        elif lidar_type == "os128":
            self.rays_theta, self.rays_phi = scan_gen.generate_os128()
        elif lidar_type == "mid360":
            self.livox_generator = scan_gen.LivoxGenerator(lidar_type)
            self.rays_theta, self.rays_phi = self.livox_generator.sample_ray_angles()
            self.dynamic_lidar = True
        else:
            raise ValueError(f"Unknown lidar type: {lidar_type}")

        self.rays_theta = np.ascontiguousarray(self.rays_theta).astype(np.float32)[::3]
        self.rays_phi = np.ascontiguousarray(self.rays_phi).astype(np.float32)[::3]

        # 射线检测：只排除机身 base；排除整机会导致场景/地面未被命中而无点云，故保留仅排除 base
        geomgroup = np.ones((mujoco.mjNGROUP,), dtype=np.ubyte)
        try:
            base_body_id = mj_model.body("base").id
        except KeyError:
            base_body_id = -1
        self.lidar = MjLidarWrapper(mj_model, site_name="lidar", backend=lidar_backend, args={'bodyexclude': base_body_id, "geomgroup": geomgroup})

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.latest_cmd_vel = np.zeros(2)

    def init_topic_publisher(self, mj_model):
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.last_pub_time_tf = -1.
        self.odom_origin = None  # 首次发布时记录起点，使 odom 与 imu 起始重合、坐标变换更紧凑

        self.imu_puber = self.create_publisher(Imu, '/imu', 10)
        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu"
        self.last_pub_time_imu = -1.

        self.bridge = CvBridge()
        self.last_pub_time_image = -1.
        self.last_pub_time_caminfo = -1.

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.pub_staticc_tf_once = False

        if self.enable_camera:
            self.head_color_puber = self.create_publisher(Image, '/head_camera/color/image_raw', 2)
            self.head_color_info_puber = self.create_publisher(CameraInfo, '/head_camera/color/camera_info', 2)
            self.head_color_info = CameraInfo()
            self.head_color_info.width = self.camera_width
            self.head_color_info.height = self.camera_height
            self.head_color_info.k = camera2k(mj_model.camera("head_camera").fovy.item() * np.pi / 180., self.camera_width, self.camera_height).flatten().tolist()

            self.head_depth_puber  = self.create_publisher(Image, '/head_camera/aligned_depth_to_color/image_raw', 2)
            self.head_depth_info_puber  = self.create_publisher(CameraInfo, '/head_camera/aligned_depth_to_color/camera_info', 2)
            self.head_depth_info = CameraInfo()
            self.head_depth_info.width = self.camera_width
            self.head_depth_info.height = self.camera_height
            self.head_depth_info.k = camera2k(mj_model.camera("head_camera").fovy.item() * np.pi / 180., self.camera_width, self.camera_height).flatten().tolist()

        # TRANSIENT_LOCAL 让后启动的 RViz 也能收到最近一帧点云
        qos_lidar = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.lidar_puber = self.create_publisher(PointCloud2, '/lidar_points', qos_lidar)
        self.last_pub_time_lidar = -1.
        # 定义点云字段（含 intensity，供 FastLIO default_handler PointXYZI 解析）
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        pc_msg = PointCloud2()
        pc_msg.header.frame_id = "lidar"
        pc_msg.fields = fields
        pc_msg.is_bigendian = False
        pc_msg.point_step = 16  # 4 个 float32 (x,y,z,intensity)
        pc_msg.height = 1
        pc_msg.is_dense = True
        self.pc_msg = pc_msg

    def get_obs(self, model, data) -> np.ndarray:
        linvel = data.sensor("local_linvel").data
        gyro = data.sensor("gyro").data
        imu_xmat = data.site_xmat[model.site("imu").id].reshape(3, 3)
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        joint_angles = data.qpos[7:7+_JOINT_NUM] - self._default_angles
        joint_velocities = data.qvel[6:6+_JOINT_NUM]

        command = np.zeros(3, dtype=np.float32)
        command[0] = self.latest_cmd_vel[0]
        command[1] = 0.0
        command[2] = self.latest_cmd_vel[1]

        obs = np.hstack([
            linvel,
            gyro,
            gravity,
            joint_angles,
            joint_velocities,
            self._last_action,
            command
        ])
        return obs.astype(np.float32)

    def get_control(self, model: mujoco.MjModel, data: mujoco.MjData) -> None:
        super().get_control(model, data)
        self.update_ros2(data)

    def publish_static_transform(self, mj_data, header_frame_id, child_frame_id):
        stfs_msg = TransformStamped()
        stfs_msg.header.stamp = self.get_clock().now().to_msg()
        stfs_msg.header.frame_id = header_frame_id
        stfs_msg.child_frame_id = child_frame_id

        tmat_base = get_site_tmat(mj_data, header_frame_id)
        tmat_child = get_site_tmat(mj_data, child_frame_id)
        tmat_trans = np.linalg.inv(tmat_base) @ tmat_child

        stfs_msg.transform.translation.x = tmat_trans[0, 3]
        stfs_msg.transform.translation.y = tmat_trans[1, 3]
        stfs_msg.transform.translation.z = tmat_trans[2, 3]

        quat = Rotation.from_matrix(tmat_trans[:3, :3]).as_quat()
        stfs_msg.transform.rotation.x = quat[0]
        stfs_msg.transform.rotation.y = quat[1]
        stfs_msg.transform.rotation.z = quat[2]
        stfs_msg.transform.rotation.w = quat[3]

        self.static_broadcaster.sendTransform(stfs_msg)

    def update_ros2(self, mj_data: mujoco.MjData) -> None:
        time_stamp = self.get_clock().now().to_msg()
        if not self.pub_staticc_tf_once:
            self.pub_staticc_tf_once = True
            self.publish_static_transform(mj_data, 'imu', 'lidar')
        self.publish_camera_info(mj_data)
        self.publish_tf(mj_data, time_stamp)
        self.publish_imu(mj_data, time_stamp)
        self.publish_images(mj_data, time_stamp)
        self.publish_lidar(mj_data, time_stamp)

    def publish_camera_info(self, mj_data):
        if not self.enable_camera:
            return
            
        if self.last_pub_time_caminfo > mj_data.time:
            self.last_pub_time_caminfo = mj_data.time
            return
        # 使用配置的帧率
        if mj_data.time - self.last_pub_time_caminfo < 1.0 / self.caminfo_rate:
            return
        self.last_pub_time_caminfo = mj_data.time
        self.head_color_info_puber.publish(self.head_color_info)
        self.head_depth_info_puber.publish(self.head_depth_info)

        self.publish_static_transform(mj_data, "imu", "lidar")

    def publish_tf(self, mj_data, time_stamp):
        if self.last_pub_time_tf > mj_data.time:
            self.last_pub_time_tf = mj_data.time
            return
        # 使用配置的帧率
        if mj_data.time - self.last_pub_time_tf < 1.0 / self.tf_rate:
            return
        self.last_pub_time_tf = mj_data.time

        pos = mj_data.sensor("global_position").data
        if self.odom_origin is None:
            self.odom_origin = pos.copy()
        trans_msg = TransformStamped()
        trans_msg.header.stamp = time_stamp
        trans_msg.header.frame_id = "odom"
        trans_msg.child_frame_id = "imu"
        trans_msg.transform.translation.x = pos[0] - self.odom_origin[0]
        trans_msg.transform.translation.y = pos[1] - self.odom_origin[1]
        trans_msg.transform.translation.z = pos[2] - self.odom_origin[2]
        trans_msg.transform.rotation.w = mj_data.sensor("orientation").data[0]
        trans_msg.transform.rotation.x = mj_data.sensor("orientation").data[1]
        trans_msg.transform.rotation.y = mj_data.sensor("orientation").data[2]
        trans_msg.transform.rotation.z = mj_data.sensor("orientation").data[3]
        self.tf_broadcaster.sendTransform(trans_msg)

    def publish_imu(self, mj_data, time_stamp):
        if self.last_pub_time_imu > mj_data.time:
            self.last_pub_time_imu = mj_data.time
            return
        # 使用配置的帧率
        if mj_data.time - self.last_pub_time_imu < 1.0 / self.imu_rate: # TODO fps Bug
            return
        self.last_pub_time_imu = mj_data.time

        self.imu_msg.header.stamp = time_stamp
        self.imu_msg.orientation.w = mj_data.sensor("orientation").data[0]
        self.imu_msg.orientation.x = mj_data.sensor("orientation").data[1]
        self.imu_msg.orientation.y = mj_data.sensor("orientation").data[2]
        self.imu_msg.orientation.z = mj_data.sensor("orientation").data[3]
        self.imu_msg.angular_velocity.x = mj_data.sensor("gyro").data[0]
        self.imu_msg.angular_velocity.y = mj_data.sensor("gyro").data[1]
        self.imu_msg.angular_velocity.z = mj_data.sensor("gyro").data[2]
        self.imu_msg.linear_acceleration.x = mj_data.sensor("accelerometer").data[0]
        self.imu_msg.linear_acceleration.y = mj_data.sensor("accelerometer").data[1]
        self.imu_msg.linear_acceleration.z = mj_data.sensor("accelerometer").data[2]
        self.imu_puber.publish(self.imu_msg)

    def publish_images(self, mj_data, time_stamp):
        if not self.enable_camera:
            return
            
        if self.last_pub_time_image > mj_data.time:
            self.last_pub_time_image = mj_data.time
            return
        if mj_data.time - self.last_pub_time_image < 1.0 / self.image_rate: # TODO fps Bug
            return
        self.last_pub_time_image = mj_data.time

        self._renderer.disable_depth_rendering()
        self._renderer.update_scene(mj_data, self._camera_name)
        head_color_img_msg = self.bridge.cv2_to_imgmsg(self._renderer.render(), encoding="rgb8")
        head_color_img_msg.header.stamp = time_stamp
        head_color_img_msg.header.frame_id = "head_camera"
        self.head_color_puber.publish(head_color_img_msg)

        self._renderer.enable_depth_rendering()
        self._renderer.update_scene(mj_data, self._camera_name)
        head_depth_img = np.array(np.clip(self._renderer.render()*1e3, 0, 65535), dtype=np.uint16)
        head_depth_img_msg = self.bridge.cv2_to_imgmsg(head_depth_img, encoding="mono16")
        head_depth_img_msg.header.stamp = time_stamp
        head_depth_img_msg.header.frame_id = "head_camera"
        self.head_depth_puber.publish(head_depth_img_msg)

    def publish_lidar(self, mj_data, time_stamp):
        if self.last_pub_time_lidar > mj_data.time:
            self.last_pub_time_lidar = mj_data.time
            return
        if mj_data.time - self.last_pub_time_lidar < 1. / self.lidar_rate:
            return
        self.last_pub_time_lidar = mj_data.time

        if self.dynamic_lidar:
            self.rays_theta, self.rays_phi = self.livox_generator.sample_ray_angles()
        n_rays = self.rays_theta.shape[0]
        # 首次或长期无点时打印射线数，确认“有没有射线”
        if not getattr(self, "_lidar_rays_logged", False):
            self._lidar_rays_logged = True
            self.get_logger().info("LiDAR 射线数: %d（每帧 trace_rays）" % n_rays)
        if n_rays == 0:
            self.get_logger().warn("LiDAR 射线数为 0，请检查 mid360 scan_mode 或 sample_ray_angles")
            return
        self.lidar.trace_rays(mj_data, self.rays_theta, self.rays_phi)
        points = self.lidar.get_hit_points()

        if points is None or points.size == 0:
            if not getattr(self, "_lidar_empty_warned", False):
                self._lidar_empty_warned = True
                self.get_logger().warn("LiDAR 射线数 %d 但 get_hit_points 为空，请检查场景/geomgroup/bodyexclude 或 flg_static" % n_rays)
            # 每约 10 秒提醒一次，便于确认仿真在跑
            _t = getattr(self, "_lidar_empty_last_log", 0.0)
            if mj_data.time - _t > 10.0:
                self._lidar_empty_last_log = mj_data.time
                self.get_logger().warn("LiDAR 仍无点云（仿真时间 %.0fs），请确认 MuJoCo 窗口在前台且场景已加载" % mj_data.time)
            return

        distances = np.sqrt(np.sum(points**2, axis=1))
        # 排除未命中射线（后端返回 dist=0 即 (0,0,0)）和过近噪声
        hit_mask = distances > 0.01
        n_rays = points.shape[0]
        n_hit = int(hit_mask.sum())
        points = points[hit_mask]
        distances = distances[hit_mask]
        if points.size == 0:
            if not getattr(self, "_lidar_empty_warned", False):
                self._lidar_empty_warned = True
                self.get_logger().warn("LiDAR 无有效命中（所有射线未击中），请检查 bodyexclude 与场景 geom group")
            return
        range_mask = (distances >= self.lidar_min_range) & (distances <= self.lidar_max_range)
        filtered_points = points[range_mask]

        if filtered_points.shape[0] == 0:
            d_min, d_max = float(distances.min()), float(distances.max())
            if not getattr(self, "_lidar_empty_warned", False):
                self._lidar_empty_warned = True
                self.get_logger().warn("LiDAR 命中距离范围 %.2f～%.2f m，在 [%.1f, %.1f] 内为空（可据此调整 lidar_min_range/lidar_max_range）" % (d_min, d_max, self.lidar_min_range, self.lidar_max_range))
            return
        # 首次发布时打印射线/命中/发布点数，便于排查“没有点云”
        if not getattr(self, "_lidar_count_logged", False):
            self._lidar_count_logged = True
            self.get_logger().info("LiDAR 发布点数: %d (命中 %d / 射线 %d, min_range=%.1f max_range=%.1f)" % (
                filtered_points.shape[0], n_hit, n_rays, self.lidar_min_range, self.lidar_max_range))
        # 拼接 intensity 列（0），满足 FastLIO default_handler PointXYZI；强制 float32 保证 data 长度 = width*point_step
        points_xyzi = np.hstack([filtered_points, np.zeros((filtered_points.shape[0], 1), dtype=np.float32)])
        points_xyzi = np.ascontiguousarray(points_xyzi.astype(np.float32))
        n_pts = points_xyzi.shape[0]
        self.pc_msg.header.stamp = time_stamp
        self.pc_msg.row_step = self.pc_msg.point_step * n_pts
        self.pc_msg.width = n_pts
        self.pc_msg.data = points_xyzi.tobytes()
        self.lidar_puber.publish(self.pc_msg)

    def cmd_vel_callback(self, msg):
        self.latest_cmd_vel[0] = msg.linear.x
        self.latest_cmd_vel[1] = msg.angular.z

def load_callback(model=None, data=None):
    global args
    mujoco.set_mjcb_control(None)

    model = mujoco.MjModel.from_xml_path(
        _MJCF_PATH.as_posix()
    )
    data = mujoco.MjData(model)

    mujoco.mj_resetDataKeyframe(model, data, 0)

    ctrl_dt = 0.02
    sim_dt = 0.001
    n_substeps = int(round(ctrl_dt / sim_dt))
    model.opt.timestep = sim_dt

    policy = OnnxControllerRos2(
        model,
        policy_path=(_ONNX_DIR / "go2_policy.onnx").as_posix(),
        default_angles=np.array(model.keyframe("home").qpos[7:7+_JOINT_NUM]),
        n_substeps=n_substeps,
        action_scale=0.5,
        lidar_type=args.lidar,
        lidar_backend=getattr(args, "backend", "cpu"),
        enable_camera=False,
    )

    spin_thread = threading.Thread(target=lambda:rclpy.spin(policy), daemon=True)
    spin_thread.start()

    mujoco.set_mjcb_control(policy.get_control)

    return model, data

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='MuJoCo LiDAR可视化与ROS2集成')
    parser.add_argument(
        "--lidar",
        type=str,
        default="mid360",
        help="LiDAR型号，默认 mid360；可选 airy, mid360, hdl64, vlp32, os128",
        choices=["airy", "mid360", "hdl64", "vlp32", "os128"],
    )
    parser.add_argument(
        "--rviz",
        action="store_true",
        help="尝试自动启动 rviz2（因 Qt/cv2 冲突常失败，建议不选此项，改在另一终端手动开 rviz2）",
    )
    args = parser.parse_args()

    rclpy.init()

    print("=" * 60)
    folder_path = os.path.dirname(os.path.abspath(__file__))
    rviz_config = os.path.join(folder_path, "../rviz_config/go2.rviz")
    import subprocess
    import shutil

    rviz_proc = None
    if getattr(args, "rviz", False):
        rviz_exec = shutil.which("rviz2")
        if rviz_exec:
            try:
                cmd = ["bash", "-c", "unset QT_PLUGIN_PATH; export QT_QPA_PLATFORM=xcb; exec \"$0\" -d \"$1\"", rviz_exec, rviz_config]
                rviz_proc = subprocess.Popen(cmd, env=os.environ)
                print(f"启动 rviz2 (pid={rviz_proc.pid})，使用配置: {rviz_config}")
            except Exception as e:
                print(f"自动启动 rviz2 失败: {e}")
        else:
            print("rviz2 未找到。")
    # 默认不自动开 rviz（从 Python 子进程开 rviz 易因 cv2 的 Qt 插件冲突失败）
    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    setup_bash = os.path.join(repo_root, "ros2_ws", "install", "setup.bash")
    print("查看点云：在【另一终端】运行（避免 Qt 插件冲突）：")
    print("  unset QT_PLUGIN_PATH; export QT_QPA_PLATFORM=xcb")
    print("  source", setup_bash)
    print("  rviz2 -d", rviz_config)
    print("=" * 60)

    try:
        viewer.launch(loader=load_callback)
    finally:
        # 仿真退出时尝试优雅关闭 rviz2
        if rviz_proc is not None:
            try:
                if rviz_proc.poll() is None:
                    rviz_proc.terminate()
                    rviz_proc.wait(timeout=2)
            except Exception:
                try:
                    rviz_proc.kill()
                except Exception:
                    pass
