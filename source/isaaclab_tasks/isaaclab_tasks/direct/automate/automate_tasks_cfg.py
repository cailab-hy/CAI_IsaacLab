# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

#################################################################################
#################################################################################
# TODO: Link the asset_ID in args
ASSET_ID = "00117"
#################################################################################
#################################################################################

# === Load library for AutoMate (Edit by CAI-Lab) ===============================
import os

ASSET_DIR = f"{ISAACLAB_NUCLEUS_DIR}/Factory"
AUTOMATE_ASSET_DIR = os.getcwd() + f"/source/isaaclab_assets/data/automate_usd"
# ===============================================================================

@configclass
class FixedAssetCfg:
    usd_path: str = ""
    diameter: float = 0.0       # Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    height: float = 0.0
    base_height: float = 0.0    # Used to compute held asset CoM.
    friction: float = 0.75
    mass: float = 0.05

@configclass
class HeldAssetCfg:
    asset_ID: str = ""
    usd_path: str = ""
    diameter: float = 0.0       # [Gripper Width] Used for automate_env.py's def _set_franka_to_default_pose() | unit: meter
    height: float = 0.0         # Used for automate_env.py's def get_handheld_asset_relative_pose() | unit: meter
    friction: float = 0.75
    mass: float = 0.05

@configclass
class RobotCfg:
    robot_usd: str = ""
    franka_fingerpad_length: float = 0.017608
    friction: float = 0.75

@configclass
class AutomateTask:
    robot_cfg: RobotCfg = RobotCfg()
    asset_ID: str = ""
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

@configclass
class Plug(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/{ASSET_ID}_plug/{ASSET_ID}_plug.usd"
    mass = 0.019

    # === JS Shin   (00004~00133) =====
    if ASSET_ID == "00004":
        diameter = 0.00690
        height = 0.07522
    elif ASSET_ID == "00007":
        diameter = 0.02148
        height = 0.01980
    elif ASSET_ID == "00014":
        diameter = 0.00437
        height = 0.04217
    elif ASSET_ID == "00015":
        diameter = 0.00614
        height = 0.04191 - 0.01
    elif ASSET_ID == "00016":
        diameter = 0.00793
        height = 0.05024 - 0.005
    elif ASSET_ID == "00021":
        diameter = 0.00783
        height = 0.04333 - 0.01
    elif ASSET_ID == "00028":
        diameter = 0.00921
        height = 0.05895 - 0.01
    elif ASSET_ID == "00030":
        diameter = 0.00649
        height = 0.05283 - 0.01
    elif ASSET_ID == "00032":
        diameter = 0.00496
        height = 0.02000
    elif ASSET_ID == "00042":
        diameter = 0.00544
        height = 0.01926 - 0.005
    elif ASSET_ID == "00062":
        diameter = 0.01247
        height = 0.05642
    elif ASSET_ID == "00074":
        diameter = 0.00465
        height = 0.04207 + 0.055
    elif ASSET_ID == "00077":
        diameter = 0.01995
        height = 0.03389 - 0.01025 + 0.015
    elif ASSET_ID == "00078":
        diameter = 0.00732
        height = 0.05851 - 0.005
    elif ASSET_ID == "00081":
        diameter = 0.03496
        height = 0.03701 - 0.01825 + 0.005
    elif ASSET_ID == "00083":
        diameter = 0.00847
        height = 0.03429 - 0.001
    elif ASSET_ID == "00103":
        diameter = 0.00483
        height = 0.08260 - 0.01
    elif ASSET_ID == "00110":
        diameter = 0.00749
        height = 0.07563 - 0.01
    elif ASSET_ID == "00117":
        diameter = 0.00425
        height = 0.10634 - 0.005
    elif ASSET_ID == "00133":
        diameter = 0.00882
        height = 0.04334 - 0.005

    # === HG Park   (00138~00318) =====
    # === WW Park   (00319~00499) =====
    # === BC Kim    (00506~00731) =====
    # === SY Hong   (00741~01136) =====

@configclass
class Socket(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/{ASSET_ID}_socket/{ASSET_ID}_socket.usd"
    base_height = 0.0

    # === JS Shin   (00004~00133) =====
    if ASSET_ID == "00004":
        diameter = 0.00744
        height = 0.02219
    elif ASSET_ID == "00007":
        diameter = 0.01836
        height = 0.04032
    elif ASSET_ID == "00014":
        diameter = 0.00472
        height = 0.00948
    elif ASSET_ID == "00015":
        diameter = 0.00726
        height = 0.01419
    elif ASSET_ID == "00016":
        diameter = 0.00905
        height = 0.01984
    elif ASSET_ID == "00021":
        diameter = 0.00918
        height = 0.00791
    elif ASSET_ID == "00028":
        diameter = 0.00993
        height = 0.02187
    elif ASSET_ID == "00030":
        diameter = 0.00730
        height = 0.00780
    elif ASSET_ID == "00032":
        diameter = 0.00260
        height = 0.02528
    elif ASSET_ID == "00042":
        diameter = 0.00649
        height = 0.00538
    elif ASSET_ID == "00062":
        diameter = 0.00875
        height = 0.04798
    elif ASSET_ID == "00074":
        diameter = 0.00580
        height = 0.08010
    elif ASSET_ID == "00077":
        diameter = 0.01022
        height = 0.03389
    elif ASSET_ID == "00078":
        diameter = 0.00878
        height = 0.04119
    elif ASSET_ID == "00081":
        diameter = 0.02364
        height = 0.03701
    elif ASSET_ID == "00083":
        diameter = 0.00379
        height = 0.01633
    elif ASSET_ID == "00103":
        diameter = 0.00623
        height = 0.00660
    elif ASSET_ID == "00110":
        diameter = 0.00551
        height = 0.02960
    elif ASSET_ID == "00117":
        diameter = 0.03863
        height = 0.06866
    elif ASSET_ID == "00133":
        diameter = 0.00720
        height = 0.02429

    # === HG Park   (00138~00318) =====
    # === WW Park   (00319~00499) =====
    # === BC Kim    (00506~00731) =====
    # === SY Hong   (00741~01136) =====

@configclass
class PlugInsert(AutomateTask):
    name = "plug_insert"
    fixed_asset_cfg = Socket()
    held_asset_cfg = Plug()
    asset_size = 8.0
    duration_s = 10.0

    # Robot
    hand_init_pos: list = [0.0, 0.0, held_asset_cfg.height]     # Relative to fixed asset tip. (Edit by CAI-Lab)
    hand_init_pos_noise: list = [0.02, 0.02, 0.01]
    hand_init_orn: list = [3.1416, 0.0, 0.0]
    hand_init_orn_noise: list = [0.0, 0.0, 0.785]

    # Fixed Asset (applies to all tasks)
    fixed_asset_init_pos_noise: list = [0.05, 0.05, 0.05]
    fixed_asset_init_orn_deg: float = 0.0
    fixed_asset_init_orn_range_deg: float = 360.0

    # Held Asset (applies to all tasks)
    held_asset_pos_noise: list = [0.003, 0.0, 0.003]            # noise level of the held asset in gripper
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
        ),
        actuators={},
    )
