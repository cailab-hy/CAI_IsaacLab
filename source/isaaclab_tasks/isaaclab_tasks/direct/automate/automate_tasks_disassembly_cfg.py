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
class AutomateTaskDisassembly:
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
        height = 0.02000 + 0.005

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
    elif ASSET_ID == "00138":
        diameter = 0.00796
        height = 0.07522

    elif ASSET_ID == "00141":
        diameter = 0.00655
        height = 0.04289

    elif ASSET_ID == "00143":
        diameter = 0.00495
        height = 0.06383

    elif ASSET_ID == "00163":
        diameter = 0.00752
        height = 0.03498

    elif ASSET_ID == "00175":
        diameter = 0.00558
        height = 0.06076

    elif ASSET_ID == "00186":
        diameter = 0.02748
        height = 0.01266

    elif ASSET_ID == "00187":
        diameter = 0.00815
        height = 0.05950

    elif ASSET_ID == "00190":
        diameter = 0.01898
        height = 0.01266

    elif ASSET_ID == "00192":
        diameter = 0.00770
        height = 0.07552

    elif ASSET_ID == "00210":
        diameter = 0.00772
        height = 0.07200

    elif ASSET_ID == "00211":
        diameter = 0.00548
        height = 0.05878

    elif ASSET_ID == "00213":
        diameter = 0.00550
        height = 0.04990

    elif ASSET_ID == "00255":
        diameter = 0.01009
        height = 0.02312

    elif ASSET_ID == "00256":
        diameter = 0.00663
        height = 0.03481

    elif ASSET_ID == "00271":
        diameter = 0.00967
        height = 0.04170

    elif ASSET_ID == "00293":
        diameter = 0.00726
        height = 0.04948

    elif ASSET_ID == "00296":
        diameter = 0.04055
        height = 0.03964

    elif ASSET_ID == "00301":
        diameter = 0.00492
        height = 0.06812

    elif ASSET_ID == "00308":
        diameter = 0.02078
        height = 0.01285

    elif ASSET_ID == "00318":
        diameter = 0.00845
        height = 0.07447

    # === WW Park   (00319~00499) =====
    elif ASSET_ID == "00319":
        diameter = 0.01
        height = 0.07340

    elif ASSET_ID == "00320":
        diameter = 0.01
        height = 0.08

    elif ASSET_ID == "00329":
        diameter = 0.01400
        height = 0.03613

    elif ASSET_ID == "00340":
        diameter = 0.00985
        height = 0.02924

    elif ASSET_ID == "00345":
        diameter = 0.00855
        height = 0.05858

    elif ASSET_ID == "00360":
        diameter = 0.00501
        height = 0.05846

    elif ASSET_ID == "00388":
        diameter = 0.03252
        height = 0.06459

    elif ASSET_ID == "00410":
        diameter = 0.04413
        height = 0.04115

    elif ASSET_ID == "00417":
        diameter = 0.00765
        height = 0.03987

    elif ASSET_ID == "00422":
        diameter = 0.02066
        height = 0.01238

    elif ASSET_ID == "00426":
        diameter = 0.00641
        height = 0.02944

    elif ASSET_ID == "00437":
        diameter = 0.00582
        height = 0.04232

    elif ASSET_ID == "00444":
        diameter = 0.01108
        height = 0.01535

    elif ASSET_ID == "00446":
        diameter = 0.01076
        height = 0.05961

    elif ASSET_ID == "00470":
        diameter = 0.02173
        height = 0.01581

    elif ASSET_ID == "00471":
        diameter = 0.00831
        height = 0.03271

    elif ASSET_ID == "00480":
        diameter = 0.00751
        height = 0.03759

    elif ASSET_ID == "00486":
        diameter = 0.00942
        height = 0.05603

    elif ASSET_ID == "00499":
        diameter = 0.0085
        height = 0.065

    # === BC Kim    (00506~00731) =====
    elif ASSET_ID == "00506":
        diameter = 0.00623
        height = 0.0751

    elif ASSET_ID == "00514":
        diameter = 0.00693
        height = 0.05135

    elif ASSET_ID == "00537":
        diameter = 0.0071
        height = 0.07812

    elif ASSET_ID == "00553":
        diameter = 0.01779
        height = 0.00896

    elif ASSET_ID == "00559":
        diameter = 0.00787
        height = 0.0569

    elif ASSET_ID == "00581":
        diameter = 0.01283
        height = 0.00601

    elif ASSET_ID == "00597":
        diameter = 0.00777
        height = 0.04848

    elif ASSET_ID == "00614":
        diameter = 0.00790
        height = 0.07238

    elif ASSET_ID == "00615":
        diameter = 0.01792
        height = 0.02635

    elif ASSET_ID == "00638":
        diameter = 0.00763
        height = 0.06

    elif ASSET_ID == "00648":
        diameter = 0.00877
        height = 0.08989

    elif ASSET_ID == "00649":
        diameter = 0.00701
        height = 0.04249

    elif ASSET_ID == "00652":
        diameter = 0.01193
        height = 0.03305

    elif ASSET_ID == "00659":
        diameter = 0.00818
        height = 0.0408

    elif ASSET_ID == "00681":
        diameter = 0.00661
        height = 0.05726

    elif ASSET_ID == "00686":
        diameter = 0.00981
        height = 0.03366

    elif ASSET_ID == "00700":
        diameter = 0.00328
        height = 0.0456

    elif ASSET_ID == "00703":
        diameter = 0.00917
        height = 0.04722

    elif ASSET_ID == "00726":
        diameter = 0.02171
        height = 0.03157

    elif ASSET_ID == "00731":
        diameter = 0.00786
        height = 0.03962

    # === SY Hong   (00741~01136) =====
    elif ASSET_ID == "00741":
        diameter = 0.00862
        height = 0.00848

    elif ASSET_ID == "00755":
        diameter = 0.00849
        height = 0.05264

    elif ASSET_ID == "00768":
        diameter = 0.01219
        height = 0.01338

    elif ASSET_ID == "00783":
        diameter = 0.00506
        height = 0.05116

    elif ASSET_ID == "00831":
        diameter = 0.00417
        height = 0.05704

    elif ASSET_ID == "00855":
        diameter = 0.00893
        height = 0.05770

    elif ASSET_ID == "00860":
        diameter = 0.00795
        height = 0.04999

    elif ASSET_ID == "00863":
        diameter = 0.00972
        height = 0.01613

    elif ASSET_ID == "01026":
        diameter = 0.00737
        height = 0.02480

    elif ASSET_ID == "01029":
        diameter = 0.00824
        height = 0.05588

    elif ASSET_ID == "01036":
        diameter = 0.00369
        height = 0.04634

    elif ASSET_ID == "01041":
        diameter = 0.00664
        height = 0.03293

    elif ASSET_ID == "01053":
        diameter = 0.00742
        height = 0.04360

    elif ASSET_ID == "01079":
        diameter = 0.00526
        height = 0.03518

    elif ASSET_ID == "01092":
        diameter = 0.00766
        height = 0.03693

    elif ASSET_ID == "01102": ##사각형임.
        diameter = 0.00599
        height = 0.04458

    elif ASSET_ID == "01125":
        diameter = 0.00578
        height = 0.02856

    elif ASSET_ID == "01129":
        diameter = 0.00980
        height = 0.05012

    elif ASSET_ID == "01132":
        diameter = 0.00884
        height = 0.09614

    elif ASSET_ID == "01136":
        diameter = 0.01260
        height = 0.05294

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
    elif ASSET_ID == "00138":
        diameter = 0.01272
        height = 0.00856

    elif ASSET_ID == "00141":
        diameter = 0.01114
        height = 0.02584

    elif ASSET_ID == "00143":
        diameter = 0.02673
        height = 0.01120

    elif ASSET_ID == "00163":
        diameter = 0.01682
        height = 0.00891

    elif ASSET_ID == "00175":
        diameter = 0.01682
        height = 0.00891

    elif ASSET_ID == "00186":
        diameter = 0.01645
        height = 0.02987

    elif ASSET_ID == "00187":
        diameter = 0.01926
        height = 0.01000

    elif ASSET_ID == "00190":
        diameter = 0.01206
        height = 0.07708

    elif ASSET_ID == "00192":
        diameter = 0.00846
        height = 0.05399

    elif ASSET_ID == "00210":
        diameter = 0.00838
        height = 0.04680

    elif ASSET_ID == "00211":
        diameter = 0.00653
        height = 0.01737

    elif ASSET_ID == "00213":
        diameter = 0.00553
        height = 0.00439

    elif ASSET_ID == "00255":
        diameter = 0.00623
        height = 0.04187

    elif ASSET_ID == "00256":
        diameter = 0.00771
        height = 0.01327

    elif ASSET_ID == "00271":
        diameter = 0.01091
        height = 0.02428

    elif ASSET_ID == "00293":
        diameter = 0.00843
        height = 0.01394

    elif ASSET_ID == "00296":
        diameter = 0.01472
        height = 0.02586

    elif ASSET_ID == "00301":
        diameter = 0.00622
        height = 0.02259

    elif ASSET_ID == "00308":
        diameter = 0.01950
        height = 0.04931

    elif ASSET_ID == "00318":
        diameter = 0.00980
        height = 0.03975

    # === WW Park   (00319~00499) =====
    elif ASSET_ID == "00319":
        socket_height = 0.03710
        socket_diameter = 0.00916

    elif ASSET_ID == "00320":
        socket_height = 0.01299
        socket_diameter = 0.0093

    elif ASSET_ID == "00329":
        socket_height = 0.02188
        socket_diameter = 0.01511

    elif ASSET_ID == "00340":
        socket_height = 0.02315
        socket_diameter = 0.00824

    elif ASSET_ID == "00345":
        socket_height = 0.03178
        socket_diameter = 0.00954

    elif ASSET_ID == "00360":
        socket_height = 0.02193
        socket_diameter = 0.00601

    elif ASSET_ID == "00388":
        socket_height = 0.01474
        socket_diameter = 0.01126

    elif ASSET_ID == "00410":
        socket_height = 0.01941
        socket_diameter = 0.01011

    elif ASSET_ID == "00417":
        socket_height = 0.00881
        socket_diameter = 0.01393

    elif ASSET_ID == "00422":
        socket_height = 0.02920
        socket_diameter = 0.01591

    elif ASSET_ID == "00426":
        socket_height = 0.00266
        socket_diameter = 0.00751

    elif ASSET_ID == "00437":
        socket_height = 0.01426
        socket_diameter = None

    elif ASSET_ID == "00444":
        socket_height = 0.01556
        socket_diameter = 0  # Need to check None?

    elif ASSET_ID == "00446":
        socket_height = 0.0605
        socket_diameter = 0.01226

    elif ASSET_ID == "00470":
        socket_height = 0.01440
        socket_diameter = None

    elif ASSET_ID == "00471":
        socket_height = 0.00798
        socket_diameter = 0.00797

    elif ASSET_ID == "00480":
        socket_height = 0.01271
        socket_diameter = 0.00859

    elif ASSET_ID == "00486":
        socket_height = 0.03627
        socket_diameter = 0.01457

    elif ASSET_ID == "00499":
        socket_height = 0.02979

    # === BC Kim    (00506~00731) =====
    elif ASSET_ID == "00506":
        diameter = 0.00616
        height = 0.04767

    elif ASSET_ID == "00514":
        diameter = 0.00747
        height = 0.01473

    elif ASSET_ID == "00537":
        diameter = 0.0081
        height = 0.06479

    elif ASSET_ID == "00553":
        diameter = 0.02188
        height = 0.03466

    elif ASSET_ID == "00559":
        diameter = 0.00829
        height = 0.01021

    elif ASSET_ID == "00581":
        diameter = 0.01044
        height = 0.03097

    elif ASSET_ID == "00597":
        diameter = 0.00880
        height = 0.02534

    elif ASSET_ID == "00614":
        diameter = 0.00891
        height = 0.5066

    elif ASSET_ID == "00615":
        diameter = 0.01731
        height = 0.04699

    elif ASSET_ID == "00638":
        diameter = 0.0079
        height = 0.02646

    elif ASSET_ID == "00648":
        diameter = 0.0089
        height = 0.06792

    elif ASSET_ID == "00649":
        diameter = 0.01068
        height = 0.00601

    elif ASSET_ID == "00652":
        diameter = 0.01292
        height = 0.02266

    elif ASSET_ID == "00659":
        diameter = 0.0082
        height = 0.00828

    elif ASSET_ID == "00681":
        diameter = 0.00876
        height = 0.04194

    elif ASSET_ID == "00686":
        diameter = 0.00999
        height = 0.00747

    elif ASSET_ID == "00700":
        diameter = 0.01189
        height = 0.00517

    elif ASSET_ID == "00703":
        diameter = 0   #None
        height = 0.03538

    elif ASSET_ID == "00726":
        diameter = 0   #None
        height = 0.0318

    elif ASSET_ID == "00731":
        diameter = 0.00902
        height = 0.01907

    # === SY Hong   (00741~01136) =====
    elif ASSET_ID == "00741":
        diameter = 0.00802
        height = 0.02708

    elif ASSET_ID == "00755":
        diameter = 0.01044
        height = 0.00748

    elif ASSET_ID == "00768":
        diameter = 0.01049
        height = 0.05776

    elif ASSET_ID == "00783":
        diameter = 0.00610
        height = 0.01232

    elif ASSET_ID == "00831":
        diameter = 0.00549
        height = 0.00545

    elif ASSET_ID == "00855":
        diameter = 0.00993
        height = 0.03065

    elif ASSET_ID == "00860":
        diameter = 0.00905
        height = 0.00966

    elif ASSET_ID == "00863":
        diameter = 0.00826
        height = 0.04797

    elif ASSET_ID == "01026":
        diameter = 0.00661
        height = 0.04150

    elif ASSET_ID == "01029":
        diameter = 0.00928
        height = 0.03040

    elif ASSET_ID == "01036":
        diameter = 0.00739
        height = 0.01027

    elif ASSET_ID == "01041":
        diameter = 0.00765
        height = 0.00900

    elif ASSET_ID == "01053":
        diameter = 0.00615
        height = 0.03530

    elif ASSET_ID == "01079":
        diameter = 0.00628
        height = 0.01989

    elif ASSET_ID == "01092":
        diameter = 0.00889
        height = 0.01589

    elif ASSET_ID == "01102": ##사각형임.
        diameter = 0.00735
        height = 0.01454

    elif ASSET_ID == "01125":
        diameter = 0.00707
        height = 0.00709

    elif ASSET_ID == "01129":
        diameter = 0.01073
        height = 0.01503

    elif ASSET_ID == "01132":
        diameter = 0.00980
        height = 0.05674

    elif ASSET_ID == "01136":
        diameter = 0.01376
        height = 0.01121

@configclass
class PlugInsert(AutomateTaskDisassembly):
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
