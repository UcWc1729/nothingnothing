import mujoco
import numpy as np
from typing import Optional, Union, List, Set

class MjLidarCPU:
    def __init__(self,
                 mj_model: mujoco.MjModel, cutoff_dist: float = 100.0,
                 geomgroup: Optional[np.ndarray] = None,
                 bodyexclude: Union[int, List[int], Set[int]] = -1) -> None:

        self.mj_model = mj_model
        self.cutoff_dist = cutoff_dist
        self.geomgroup = geomgroup
        # 支持单个 body ID 或多个 body ID 列表（排除整机时用）
        if isinstance(bodyexclude, (list, set)):
            self._bodyexclude_set = set(bodyexclude)
            self._bodyexclude_single = -1
        else:
            self._bodyexclude_set = None
            self._bodyexclude_single = int(bodyexclude) if bodyexclude >= 0 else -1

        self._dist: Optional[np.ndarray] = None
        self._hit_points: Optional[np.ndarray] = None

    def update(self, mj_data: mujoco.MjData) -> None:
        self.mj_data = mj_data

    def trace_rays(self,
                   pose_4x4: np.ndarray,
                   ray_theta: np.ndarray,
                   ray_phi: np.ndarray) -> None:

        if ray_phi.shape[0] != ray_theta.shape[0]:
            raise ValueError("ray_phi and ray_theta must have the same shape")

        _nray = ray_phi.shape[0]

        # Initialize
        self._dist = np.full(_nray, self.cutoff_dist, dtype=np.float64)
        _geomid = np.full(_nray, 0, dtype=np.int32)

        site_pos, site_mat = pose_4x4[:3, 3], pose_4x4[:3, :3]
        pnt = np.array([site_pos]).T
        x = np.cos(ray_phi) * np.cos(ray_theta)
        y = np.cos(ray_phi) * np.sin(ray_theta)
        z = np.sin(ray_phi)
        local_vecs = np.stack((x, y, z), axis=-1)
        world_vecs = local_vecs @ site_mat.T
        world_vecs /= np.linalg.norm(world_vecs, axis=1, keepdims=True)
        world_vecs_flat = world_vecs.flatten()

        # 多 body 排除时先不传 bodyexclude，射完再按 geom 所属 body 过滤
        bodyexclude_arg = -1 if self._bodyexclude_set is not None else self._bodyexclude_single

        # flg_static=1 包含静态物体（地面 plane、场景 mesh），否则地面扫不到
        mujoco.mj_multiRay(
            m=self.mj_model,
            d=self.mj_data,
            pnt=pnt,
            vec=world_vecs_flat,
            geomgroup=self.geomgroup,
            flg_static=1,
            bodyexclude=bodyexclude_arg,
            geomid=_geomid,
            dist=self._dist,
            nray=_nray,
            cutoff=self.cutoff_dist,
        )

        # 多 body 排除：把命中“排除 body”的射线当作未命中（dist=0，后续会滤掉）
        if self._bodyexclude_set is not None and self.mj_model.ngeom > 0:
            for i in range(_nray):
                gid = int(_geomid[i])
                if gid >= 0 and gid < self.mj_model.ngeom:
                    if self.mj_model.geom_bodyid[gid] in self._bodyexclude_set:
                        self._dist[i] = 0.0

        self._dist[_geomid == -1] = 0
        self._hit_points = local_vecs * self._dist[:, np.newaxis]

    def get_hit_points(self) -> Optional[np.ndarray]:
        return self._hit_points

    def get_distances(self) -> Optional[np.ndarray]:
        return self._dist