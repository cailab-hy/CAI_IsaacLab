# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import os

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

ASSET_DIR = f"{ISAACLAB_NUCLEUS_DIR}/Factory"
AUTOMATE_ASSET_DIR = os.getcwd() + f"/source/isaaclab_assets/data/automate_usd"

tmp_mass = 0.019

@configclass
class FixedAssetCfg:
    usd_path: str = ""
    diameter: float = 0.0
    height: float = 0.0
    base_height: float = 0.0  # Used to compute held asset CoM.
    friction: float = 0.75
    mass: float = 0.05


@configclass
class HeldAssetCfg:
    usd_path: str = ""
    diameter: float = 0.0  # Used for gripper width.
    height: float = 0.0
    friction: float = 0.75
    mass: float = 0.05


@configclass
class RobotCfg:
    robot_usd: str = ""
    franka_fingerpad_length: float = 0.017608
    friction: float = 0.75


@configclass
# class AutomateTask:
class AutomateTaskDisassembly:
    robot_cfg: RobotCfg = RobotCfg()
    name: str = ""
    duration_s = 5.0

    fixed_asset_cfg: FixedAssetCfg = FixedAssetCfg()
    held_asset_cfg: HeldAssetCfg = HeldAssetCfg()
    asset_size: float = 0.0

    # Robot
    hand_init_pos: list = [0.0, 0.0, 0.015]  # Relative to fixed asset tip.
    hand_init_pos_noise: list = [0.02, 0.02, 0.01]
    hand_init_orn: list = [3.1416, 0, 2.356]
    hand_init_orn_noise: list = [0.0, 0.0, 1.57]

    # Action
    unidirectional_rot: bool = False

    # Fixed Asset (applies to all tasks)
    fixed_asset_init_pos_noise: list = [0.05, 0.05, 0.05]
    fixed_asset_init_orn_deg: float = 0.0
    fixed_asset_init_orn_range_deg: float = 360.0

    # Held Asset (applies to all tasks)
    held_asset_pos_noise: list = [0.0, 0.006, 0.003]  # noise level of the held asset in gripper
    held_asset_rot_init: float = -90.0

    # Reward
    ee_success_yaw: float = 0.0  # nut_thread task only.
    action_penalty_scale: float = 0.0
    action_grad_penalty_scale: float = 0.0
    # Reward function details can be found in Appendix B of https://arxiv.org/pdf/2408.04587.
    # Multi-scale keypoints are used to capture different phases of the task.
    # Each reward passes the keypoint distance, x, through a squashing function:
    #     r(x) = 1/(exp(-ax) + b + exp(ax)).
    # Each list defines [a, b] which control the slope and maximum of the squashing function.
    num_keypoints: int = 4
    keypoint_scale: float = 0.15
    keypoint_coef_baseline: list = [5, 4]  # General movement towards fixed object.
    keypoint_coef_coarse: list = [50, 2]  # Movement to align the assets.
    keypoint_coef_fine: list = [100, 0]  # Smaller distances for threading or last-inch insertion.
    # Fixed-asset height fraction for which different bonuses are rewarded (see individual tasks).
    success_threshold: float = 0.04
    engage_threshold: float = 0.9

################################### Template ##############################################################

@configclass
class Plug_00000(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00004_plug/00004_plug.usd"

    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.007986

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.050

    # Not Used?
    mass = 0.019


@configclass
class Socket_00000(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00004_socket/00004_socket.usd"

    # Not Used?
    diameter = 0.0081

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.025
    base_height = 0.0

############################################################################################################

@configclass
class Plug_00004(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00004_plug/00004_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00690

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.07522

    # Not Used?
    mass = 0.019

@configclass
class Socket_00004(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00004_socket/00004_socket.usd"

    # Not Used?
    diameter = 0.00744

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.02219
    base_height = 0.0

@configclass
class Plug_00007(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00007_plug/00007_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.02148

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.01980

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00007(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00007_socket/00007_socket.usd"

    # Not Used?
    diameter = 0.01836

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.04032
    base_height = 0.0

@configclass
class Plug_00014(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00014_plug/00014_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00437

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.04217

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00014(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00014_socket/00014_socket.usd"

    # Not Used?
    diameter = 0.00472

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.00948
    base_height = 0.0

@configclass
class Plug_00015(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00015_plug/00015_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00614

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.04191 - 0.01

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00015(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00015_socket/00015_socket.usd"

    # Not Used?
    diameter = 0.00726

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.01419
    base_height = 0.0

@configclass
class Plug_00016(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00016_plug/00016_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00793

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.05024 - 0.005

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00016(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00016_socket/00016_socket.usd"

    # Not Used?
    diameter = 0.00905

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.01984
    base_height = 0.0

@configclass
class Plug_00021(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00021_plug/00021_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00783

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.04333 - 0.01

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00021(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00021_socket/00021_socket.usd"

    # Not Used?
    diameter = 0.00918

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.00791
    base_height = 0.0

@configclass
class Plug_00028(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00028_plug/00028_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00921

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.05895 - 0.01

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00028(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00028_socket/00028_socket.usd"

    # Not Used?
    diameter = 0.00993

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.02187
    base_height = 0.0

@configclass
class Plug_00030(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00030_plug/00030_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00649

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.05283 - 0.01

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00030(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00030_socket/00030_socket.usd"

    # Not Used?
    diameter = 0.00730

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.00780
    base_height = 0.0

@configclass
class Plug_00032(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00032_plug/00032_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00496

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.02000

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00032(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00032_socket/00032_socket.usd"

    # Not Used?
    diameter = 0.00260

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.02528
    base_height = 0.0

@configclass
class Plug_00042(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00042_plug/00042_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00544

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.01926 - 0.005

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00042(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00042_socket/00042_socket.usd"

    # Not Used?
    diameter = 0.00649

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.00538
    base_height = 0.0

@configclass
class Plug_00062(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00062_plug/00062_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.01247

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.05642

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00062(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00062_socket/00062_socket.usd"

    # Not Used?
    diameter = 0.00875

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.04798
    base_height = 0.0

@configclass
class Plug_00074(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00074_plug/00074_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00465

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.04207 + 0.055

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00074(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00074_socket/00074_socket.usd"

    # Not Used?
    diameter = 0.00580

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.08010
    base_height = 0.0

@configclass
class Plug_00077(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00077_plug/00077_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.01995

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    # height = 0.01025 # by measure
    height = 0.03389 - 0.01025 + 0.015 # by js

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00077(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00077_socket/00077_socket.usd"

    # Not Used?
    diameter = 0.01022

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.03389
    base_height = 0.0

@configclass
class Plug_00078(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00078_plug/00078_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00732

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.05851 - 0.005

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00078(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00078_socket/00078_socket.usd"

    # Not Used?
    diameter = 0.00878

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.04119
    base_height = 0.0

@configclass
class Plug_00081(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00081_plug/00081_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.03496

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    # height = 0.01825 # by measure
    height = 0.03701 - 0.01825 + 0.005 # by js

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00081(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00081_socket/00081_socket.usd"

    # Not Used?
    diameter = 0.02364

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.03701
    base_height = 0.0

@configclass
class Plug_00083(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00083_plug/00083_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00847

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.03429 - 0.001

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00083(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00083_socket/00083_socket.usd"

    # Not Used?
    diameter = 0.00379

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.01633
    base_height = 0.0

@configclass
class Plug_00103(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00103_plug/00103_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00483

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.08260 - 0.01

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00103(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00103_socket/00103_socket.usd"

    # Not Used?
    diameter = 0.00623

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.00660
    base_height = 0.0

@configclass
class Plug_00110(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00110_plug/00110_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00749

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.07563 - 0.01

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00110(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00110_socket/00110_socket.usd"

    # Not Used?
    diameter = 0.00551

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.02960
    base_height = 0.0

@configclass
class Plug_00117(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00117_plug/00117_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00425

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.10634 - 0.005

    # Not Used?
    mass = 0.019

@configclass
class Socket_00117(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00117_socket/00117_socket.usd"

    # Not Used?
    diameter = 0.03863

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.06866
    base_height = 0.0

@configclass
class Plug_00133(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00133_plug/00133_plug.usd"
    
    # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    diameter = 0.00882

    # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    height = 0.04334 - 0.005

    # Not Used?
    mass = tmp_mass

@configclass
class Socket_00133(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00133_socket/00133_socket.usd"

    # Not Used?
    diameter = 0.00720

    # Used for automate_env.py's def _init_tensors() & def randomize_initial_state() | unit: meter
    height = 0.02429
    base_height = 0.0

############################################################################################################

@configclass
class PlugInsert(AutomateTaskDisassembly):
    name = "plug_insert"
    fixed_asset_cfg = Socket_00133()
    held_asset_cfg = Plug_00133()
    asset_size = 8.0
    duration_s = 10.0

    # Robot
    hand_init_pos: list = [0.0, 0.0, held_asset_cfg.height] # [0.0, 0.0, 0.047] # Relative to fixed asset tip. (edit by SH Yu)
    hand_init_pos_noise: list = [0.02, 0.02, 0.01]
    hand_init_orn: list = [3.1416, 0.0, 0.0]
    hand_init_orn_noise: list = [0.0, 0.0, 0.785]

    # Fixed Asset (applies to all tasks)
    fixed_asset_init_pos_noise: list = [0.05, 0.05, 0.05]
    fixed_asset_init_orn_deg: float = 0.0
    fixed_asset_init_orn_range_deg: float = 360.0

    # Held Asset (applies to all tasks)
    held_asset_pos_noise: list = [0.003, 0.0, 0.003]  # noise level of the held asset in gripper
    held_asset_rot_init: float = 0.0

    # Rewards
    keypoint_coef_baseline: list = [5, 4]
    keypoint_coef_coarse: list = [50, 2]
    keypoint_coef_fine: list = [100, 0]

    # Fraction of socket height.
    success_threshold: float = 0.04
    engage_threshold: float = 0.9

    fixed_asset: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/FixedAsset",
        spawn=sim_utils.UsdFileCfg(
            usd_path=fixed_asset_cfg.usd_path,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=False,
                max_depenetration_velocity=5.0,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=3666.0,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=192,
                solver_velocity_iteration_count=1,
                max_contact_impulse=1e32,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=fixed_asset_cfg.mass),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.6, 0.0, 0.05), rot=(1.0, 0.0, 0.0, 0.0), joint_pos={}, joint_vel={}
        ),
        actuators={},
    )
    held_asset: ArticulationCfg = ArticulationCfg(
        prim_path="/World/envs/env_.*/HeldAsset",
        spawn=sim_utils.UsdFileCfg(
            usd_path=held_asset_cfg.usd_path,
            activate_contact_sensors=True,
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
                linear_damping=0.0,
                angular_damping=0.0,
                max_linear_velocity=1000.0,
                max_angular_velocity=3666.0,
                enable_gyroscopic_forces=True,
                solver_position_iteration_count=192,
                solver_velocity_iteration_count=1,
                max_contact_impulse=1e32,
            ),
            mass_props=sim_utils.MassPropertiesCfg(mass=held_asset_cfg.mass),
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            pos=(0.0, 0.4, 0.1), rot=(1.0, 0.0, 0.0, 0.0), joint_pos={}, joint_vel={}
            # pos=(0.4, 0.0, 0.1), rot=(1.0, 0.0, 0.0, 0.0), joint_pos={}, joint_vel={}
        ),
        actuators={},
    )
