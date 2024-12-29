# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#


class ScenarioTemplate:
    def __init__(self):
        pass

    def setup_scenario(self):
        pass

    def teardown_scenario(self):
        pass

    def update_scenario(self):
        pass

import numpy as np
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid, GroundPlane
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils import distance_metrics
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats, quats_to_rot_matrices, rot_matrices_to_quats
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.viewports import set_camera_view
from omni.isaac.motion_generation import ArticulationMotionPolicy, RmpFlow
from omni.isaac.motion_generation.interface_config_loader import load_supported_motion_policy_config
from omni.isaac.nucleus import get_assets_root_path
from .dds.PoseMsg import Pose6D
from .dds.FlexivState import FlexivStateMsg
from .dds.telemetry import Subscriber

"""
This scenario takes in a robot Articulation and makes it move through its joint DOFs.
Additionally, it adds a cuboid prim to the stage that moves in a circle around the robot.

The particular framework under which this scenario operates should not be taken as a direct
recomendation to the user about how to structure their code.  In the simple example put together
in this template, this particular structure served to improve code readability and separate
the logic that runs the example from the UI design.
"""
import os

class FlexivJointMimicScenario(ScenarioTemplate):
    def __init__(self):
        self._object = None
        self._articulation = None
        self._rmpflow = None
        self.articulation_motion_policy = None
        self._running_scenario = False
        self.flexiv_q0 = np.array([0.001961823320016265,
                                    -0.20705948770046234,
                                    0.21981844305992126,
                                    2.069606304168701,
                                    -0.0594966784119606,
                                    0.6976734399795532,
                                    0.07441084086894989,])
        self.assets_path = os.path.join(os.path.abspath(os.path.dirname(__file__)), 'assets')
        self._robot_description_file = os.path.join(self.assets_path, 'flexiv-lula-description.yaml')
        self._robot_urdf_file = os.path.join(self.assets_path, 'flexiv_rizon10s_kinematics.urdf')
        self._rmpflow_config_yaml = os.path.join(self.assets_path, 'rmp_flow_cfg.yaml')
        self._offset = 0.49362
        self._x_limit = [0.3,  0.7]
        self._y_limit = [-0.40, 0.4]
        self._z_limit = [0.04,  0.2]

    def setup_scenario(self, articulation, object_prim, get_vr_pose_fn):
        self._articulation = articulation
        self._object = object_prim
        self.get_vr_pose_fn = get_vr_pose_fn
        self._initial_object_position = self._object.get_world_pose()[0]
        self._running_scenario = True
        self._lower_joint_limits = articulation.dof_properties["lower"]
        self._upper_joint_limits = articulation.dof_properties["upper"]
        articulation.set_joint_positions(self.flexiv_q0)

         # Loading RMPflow can be done quickly for supported robots

        rmpflow_config_dict = {
            "end_effector_frame_name": 'flange',
            "maximum_substep_size": 0.0034,
            "ignore_robot_state_updates": False,
            "robot_description_path": self._robot_description_file,
            "urdf_path": self._robot_urdf_file,
            "rmpflow_config_path": self._rmpflow_config_yaml,
        }
        action = ArticulationAction(
                joint_positions =self.flexiv_q0,
                joint_velocities  = np.zeros_like(self.flexiv_q0),
            )
        self._articulation.apply_action(action)
        self._articulation.set_joint_positions(self.flexiv_q0)
        # Initialize an RmpFlow object
        self._rmpflow = RmpFlow(**rmpflow_config_dict)
        self.articulation_motion_policy = ArticulationMotionPolicy(self._articulation, self._rmpflow, 1 / 60.0)
        ee_trans, ee_rot = self._rmpflow.get_end_effector_pose(self.articulation_motion_policy.get_active_joints_subset().get_joint_positions())
        self._rmpflow.set_end_effector_target(ee_trans, np.array([0,0,1,0]))
        self.ee_z_init = ee_trans[2]
        try:
            self.flexiv_state_subscriber = Subscriber(FlexivStateMsg, 'isaac_flexiv_joint_cmds')
            self.joint_subsciber_enable = True
        except:
            print('Could not start the flexiv DDS joint subscriber')
            self.joint_subsciber_enable = False

    def teardown_scenario(self):
        self._time = 0.0
        self._object = None
        self._articulation = None
        self._rmpflow = None
        self.articulation_motion_policy = None
        self._running_scenario = False

    def update_scenario(self, step: float):
        if not self._running_scenario:
            return
        vr_state = self.get_vr_pose_fn()
        if self.joint_subsciber_enable:
            joint_cmd = self.flexiv_state_subscriber.getState()
            if joint_cmd is not None:
                print(joint_cmd.q)
        obj_pose = self._object.get_world_pose()
        obj_t = obj_pose[0]
        # if state['t'] is not None:
            # self._object.set_world_pose(np.array(state['t']))
        # ee_trans, ee_rot = self._rmpflow.get_end_effector_pose(
        #         self.articulation_motion_policy.get_active_joints_subset().get_joint_positions()
        #     )
        if  vr_state is not None:
            cmd = vr_state['right_t'].copy()
            cmd[0] = np.clip(cmd[0], self._x_limit[0], self._x_limit[1])
            cmd[1] = np.clip(cmd[1], self._y_limit[0], self._y_limit[1])
            cmd[2] = np.clip(cmd[2], self._z_limit[0], self._z_limit[1])
            print(cmd)
            # ee_trans_error = cmd - ee_trans
            # clipped_error = np.clip(ee_trans_error, -0.04, 0.04)
            # ee_des = ee_trans + clipped_error
            # self._rmpflow.set_end_effector_target(obj_t, np.array([0,0,1,0]))
            # print(ee_des)
        # else:
        #     # self._rmpflow.set_end_effector_target(np.array([0.6, 0. , 0.8]), 
        #                                         # np.array([0,0,1,0]))
        #     self._rmpflow.set_end_effector_target(obj_t, np.array([0,0,1,0]))

        # print(ee_trans)
        # self._rmpflow.update_world()
        # action = self.articulation_motion_policy.get_next_articulation_action(1. / 60.0)
        # self._articulation.apply_action(action)


