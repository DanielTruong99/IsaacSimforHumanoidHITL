from typing import List, Optional # type: ignore

import isaacsim.core.api.tasks as tasks
import numpy as np
import carb

from isaacsim.robot.manipulators.examples.franka.tasks import FollowTarget # type: ignore
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleXFormPrim
from isaacsim.core.utils.prims import is_prim_path_valid
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import get_stage_units
from isaacsim.core.utils.string import find_unique_string_name
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.api.robots.robot import Robot
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.stage import add_reference_to_stage, get_stage_units
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.storage.native import get_assets_root_path

import isaacsim.robot_motion.motion_generation.interface_config_loader as interface_config_loader
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot_motion.motion_generation.articulation_kinematics_solver import ArticulationKinematicsSolver
from isaacsim.robot_motion.motion_generation.lula.kinematics import LulaKinematicsSolver

class LegKinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: SingleArticulation, end_effector_frame_name: Optional[str] = None) -> None:
        robot_description_path = "/home/humanoid/DanielWorkspace/IsaacSimDemo/leg_robot.yaml"
        urdf_path = "/home/humanoid/DanielWorkspace/RLHumanoid_Ver2/source/leg_robot/leg_robot/assets/urdf/leg00/leg00.urdf"
        self._kinematics = LulaKinematicsSolver(robot_description_path=robot_description_path, urdf_path=urdf_path)

        end_effector_frame_name = "L_toe"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)

        return

class LegRobot(Robot):
    def __init__(
        self,
        prim_path: str,
        name: str = "leg_robot",
        usd_path: Optional[str] = None,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None,
        end_effector_prim_name: Optional[str] = None,
    ) -> None:
        prim = get_prim_at_path(prim_path)
        self._end_effector = None
        self._end_effector_prim_name = end_effector_prim_name
        if not prim.IsValid():
            # add the leg robot to the stage
            add_reference_to_stage(usd_path=usd_path, prim_path=prim_path)

            # set the end effector path "World/LegRobot/L_toe"
            self._end_effector_prim_path = end_effector_prim_name

        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None
        )
        return

    @property
    def end_effector(self) -> SingleRigidPrim:
        """[summary]

        Returns:
            SingleRigidPrim: [description]
        """
        return self._end_effector

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]"""
        super().initialize(physics_sim_view)
        self._end_effector = SingleRigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self._end_effector.initialize(physics_sim_view)
        return

class LegFollowTarget(FollowTarget):
    def __init__(self, name="leg_follow_target", **kwargs):
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=kwargs.get("target_prim_path"),
            target_name=kwargs.get("target_name"),
            target_position=kwargs.get("target_position"),
            target_orientation=kwargs.get("target_orientation"),
            offset=kwargs.get("offset"),
        )
        self._leg_prim_path = kwargs.get("leg_prim_path")
        self._leg_robot_name = kwargs.get("leg_robot_name")
        self._ee_prim_path = kwargs.get("ee_prim_path")
        self._leg_usd_path = kwargs.get("leg_usd_path")
        return

    def get_observations(self) -> dict:
        """[summary]

        Returns:
            dict: [description]
        """
        joints_state = self._robot.get_joints_state()
        target_position, target_orientation = self._target.get_local_pose()
        robot_position, robot_orientation = self._robot.get_local_pose()
        target_position = target_position - robot_position

        return {
            self._robot.name: {
                "joint_positions": np.array(joints_state.positions),
                "joint_velocities": np.array(joints_state.velocities),
            },
            self._target.name: {"position": np.array(target_position), "orientation": None},
        }

    def set_robot(self) -> LegRobot:
        """[summary]

        Returns:
            LegRobot: [description]
        """
        if self._leg_prim_path is None:
            self._leg_prim_path = find_unique_string_name(
                initial_name="/World/LegRobot", is_unique_fn=lambda x: not is_prim_path_valid(x)
            )
        if self._leg_robot_name is None:
            self._leg_robot_name = find_unique_string_name(
                initial_name="leg_robot", is_unique_fn=lambda x: not self.scene.object_exists(x)
            )
        return LegRobot(prim_path=self._leg_prim_path, name=self._leg_robot_name, end_effector_prim_name=self._ee_prim_path, usd_path=self._leg_usd_path)
    

