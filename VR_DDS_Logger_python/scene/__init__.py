from omni.isaac.core import World
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.objects import cuboid, sphere
from omni.isaac.core.utils.types import ArticulationAction
from typing import Dict, List

# Third Party
import numpy as np
from matplotlib import cm
from omni.isaac.core import World
from omni.isaac.core.materials import OmniPBR
from omni.isaac.core.objects import cuboid
from omni.isaac.core.robots import Robot
from pxr import UsdPhysics

class PCDManager:
    def __init__(
        self,
        num_points: int = 5000,
        size: float = 0.02,
        color: List[float] = [1, 1, 1],
        prefix_path: str = "/World/pointcloud/point_",
        material_path: str = "/World/looks/v_",
    ) -> None:
        self.cuboid_list = []
        self.cuboid_material_list = []
        self.disable_idx = num_points
        for i in range(num_points):
            target_material = OmniPBR("/World/looks/v_" + str(i), color=np.ravel(color))
            s=sphere.VisualSphere(
                prefix_path +str(i),
                position=np.array([0, 0, -10]),
                radius=size,
                visual_material=target_material,
                )
            self.cuboid_list.append(s)
            self.cuboid_material_list.append(target_material)
            s.set_visibility(True)

    def update_points(self, point_position: np.ndarray, color_axis: int = 0):
        max_index = min(point_position.shape[0], len(self.cuboid_list))

        jet = cm.get_cmap("hot")  # .reversed()
        z_val = point_position[:, 0]

        jet_colors = jet(z_val)

        for i in range(max_index):
            self.cuboid_list[i].set_visibility(True)

            self.cuboid_list[i].set_local_pose(translation=point_position[i])
            self.cuboid_material_list[i].set_color(jet_colors[i][:3])

        for i in range(max_index, len(self.cuboid_list)):
            self.cuboid_list[i].set_local_pose(translation=np.ravel([0, 0, -10.0]))

            # self.cuboid_list[i].set_visibility(False)

    def clear(self):
        for i in range(len(self.cuboid_list)):
            self.cuboid_list[i].set_local_pose(translation=np.ravel([0, 0, -10.0]))
            