# automate_tasks_disassembly_cfg.py

import os
from hydra.core.config_store import ConfigStore
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR
from typing import Optional

# 예시 경로 (실제 프로젝트 환경에 맞게 수정)
ASSET_DIR = f"{ISAACLAB_NUCLEUS_DIR}/Factory"
AUTOMATE_ASSET_DIR = os.getcwd() + "/source/isaaclab_assets/data/automate_usd"

# 1. 기본 자산 구성 클래스 정의
@configclass
class FixedAssetCfg:
    usd_path: str = ""
    diameter: float = 0.0
    height: float = 0.0
    base_height: float = 0.0  # Held asset의 Center of Mass 계산에 사용됨.
    friction: float = 0.75
    mass: float = 0.05

@configclass
class HeldAssetCfg:
    usd_path: str = ""
    diameter: float = 0.0  # 그리퍼 너비 계산에 사용됨.
    height: float = 0.0
    friction: float = 0.75
    mass: float = 0.05

# (필요시 추가로 RobotCfg나 기타 구성 클래스 정의 가능)
@configclass
class RobotCfg:
    robot_usd: str = ""
    franka_fingerpad_length: float = 0.017608
    friction: float = 0.75

# 2. 기본 작업(Disassembly) 구성 클래스 정의
@configclass
class AutomateTaskDisassembly:
    robot_cfg: RobotCfg = RobotCfg()
    name: str = ""
    duration_s: float = 5.0
    # 추가된 asset 관련 설정
    asset_id: str = "00004"  # 기본 asset_id; CLI 오버라이드로 변경 가능
    data_dir: str = "source/isaaclab_tasks/isaaclab_tasks/direct/automate/data"
    disassembly_dist_file: str = "disassembly_dist.json"
    # save_data는 asset_id 기반으로 생성하도록 할 수도 있고, 직접 지정할 수도 있음
    save_data: str = ""

    # 자산 관련 기본 구성은 나중에 __post_init__에서 채워짐.
    fixed_asset_cfg: FixedAssetCfg = FixedAssetCfg()
    held_asset_cfg: HeldAssetCfg = HeldAssetCfg()
    asset_size: float = 0.0

    # Robot 관련 파라미터
    hand_init_pos: list = [0.0, 0.0, 0.015]  # fixed asset의 tip 기준 상대 위치.
    hand_init_pos_noise: list = [0.02, 0.02, 0.01]
    hand_init_orn: list = [3.1416, 0, 2.356]
    hand_init_orn_noise: list = [0.0, 0.0, 1.57]

    # Action 관련
    unidirectional_rot: bool = False

    # Fixed Asset (모든 작업에 공통)
    fixed_asset_init_pos_noise: list = [0.05, 0.05, 0.05]
    fixed_asset_init_orn_deg: float = 0.0
    fixed_asset_init_orn_range_deg: float = 360.0

    # Held Asset (모든 작업에 공통)
    held_asset_pos_noise: list = [0.0, 0.006, 0.003]  # 그리퍼 내 held asset 노이즈
    held_asset_rot_init: float = -90.0

    # Reward 관련
    ee_success_yaw: float = 0.0  # nut_thread task에만 적용.
    action_penalty_scale: float = 0.0
    action_grad_penalty_scale: float = 0.0
    num_keypoints: int = 4
    keypoint_scale: float = 0.15
    keypoint_coef_baseline: list = [5, 4]
    keypoint_coef_coarse: list = [50, 2]
    keypoint_coef_fine: list = [100, 0]
    success_threshold: float = 0.04
    engage_threshold: float = 0.9

# 3. ----------------------------- 자산별 세부 구성 클래스 ------------------------------------------------------
@configclass
class Plug_00004(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00004_plug/00004_plug.usd"
    diameter = 0.00690
    height = 0.07522
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00004(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00004_socket/00004_socket.usd"
    diameter = 0.00744
    height = 0.02219
    base_height = 0.0

@configclass
class Plug_00138(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00138_plug/00138_plug.usd"
    diameter = 0.00796
    height = 0.07522
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00138(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00138_socket/00138_socket.usd"
    diameter = 0.01272
    height = 0.00856
    base_height = 0.0

@configclass
class Plug_00141(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00141_plug/00141_plug.usd"
    diameter = 0.00655
    height = 0.04289
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00141(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00141_socket/00141_socket.usd"
    diameter = 0.01114
    height = 0.02584
    base_height = 0.0

@configclass
class Plug_00143(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00143_plug/00143_plug.usd"
    diameter = 0.00495
    height = 0.06383
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00143(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00143_socket/00143_socket.usd"
    diameter = 0.02673
    height = 0.01120
    base_height = 0.0

@configclass
class Plug_00163(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00163_plug/00163_plug.usd"
    diameter = 0.00752
    height = 0.03498
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00163(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00163_socket/00163_socket.usd"
    diameter = 0.01682
    height = 0.00891
    base_height = 0.0

@configclass
class Plug_00175(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00175_plug/00175_plug.usd"
    diameter = 0.00558
    height = 0.06076
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00175(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00175_socket/00175_socket.usd"
    diameter = 0.00757
    height = 0.03995
    base_height = 0.0

@configclass
class Socket_00175(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00175_socket/00175_socket.usd"
    diameter = 0.01682
    height = 0.00891
    base_height = 0.0

@configclass
class Plug_00175(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00175_plug/00175_plug.usd"
    diameter = 0.00558
    height = 0.06076
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00186(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00186_socket/00186_socket.usd"
    diameter = 0.01645
    height = 0.02987
    base_height = 0.0

@configclass
class Plug_00186(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00186_plug/00186_plug.usd"
    diameter = 0.02748
    height = 0.01266
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00187(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00187_socket/00187_socket.usd"
    diameter = 0.01926
    height = 0.01000
    base_height = 0.0

@configclass
class Plug_00187(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00187_plug/00187_plug.usd"
    diameter = 0.00815
    height = 0.05950
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00190(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00190_socket/00190_socket.usd"
    diameter = 0.01206
    height = 0.07708
    base_height = 0.0

@configclass
class Plug_00190(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00190_plug/00190_plug.usd"
    diameter = 0.01898
    height = 0.01266
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00192(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00192_socket/00192_socket.usd"
    diameter = 0.00846
    height = 0.05399
    base_height = 0.0

@configclass
class Plug_00192(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00192_plug/00192_plug.usd"
    diameter = 0.00770
    height = 0.07552
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00210(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00210_socket/00210_socket.usd"
    diameter = 0.00838
    height = 0.04680
    base_height = 0.0

@configclass
class Plug_00210(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00210_plug/00210_plug.usd"
    diameter = 0.00772
    height = 0.07200
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00211(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00211_socket/00211_socket.usd"
    diameter = 0.00653
    height = 0.01737
    base_height = 0.0

@configclass
class Plug_00211(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00211_plug/00211_plug.usd"
    diameter = 0.00548
    height = 0.05878
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00213(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00213_socket/00213_socket.usd"
    diameter = 0.00553
    height = 0.00439
    base_height = 0.0

@configclass
class Plug_00213(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00213_plug/00213_plug.usd"
    diameter = 0.00550
    height = 0.04990
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00255(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00255_socket/00255_socket.usd"
    diameter = 0.00623
    height = 0.04187
    base_height = 0.0

@configclass
class Plug_00255(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00255_plug/00255_plug.usd"
    diameter = 0.01009
    height = 0.02312
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00256(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00256_socket/00256_socket.usd"
    diameter = 0.00771
    height = 0.01327
    base_height = 0.0

@configclass
class Plug_00256(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00256_plug/00256_plug.usd"
    diameter = 0.00663
    height = 0.03481
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00271(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00271_socket/00271_socket.usd"
    diameter = 0.01091
    height = 0.02428
    base_height = 0.0

@configclass
class Plug_00271(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00271_plug/00271_plug.usd"
    diameter = 0.00967
    height = 0.04170
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00293(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00293_socket/00293_socket.usd"
    diameter = 0.00843
    height = 0.01394
    base_height = 0.0

@configclass
class Plug_00293(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00293_plug/00293_plug.usd"
    diameter = 0.00726
    height = 0.04948
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00296(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00296_socket/00296_socket.usd"
    diameter = 0.01472
    height = 0.02586
    base_height = 0.0

@configclass
class Plug_00296(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00296_plug/00296_plug.usd"
    diameter = 0.04055
    height = 0.03964
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00301(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00301_socket/00301_socket.usd"
    diameter = 0.00622
    height = 0.02259
    base_height = 0.0

@configclass
class Plug_00301(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00301_plug/00301_plug.usd"
    diameter = 0.00492
    height = 0.06812
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00308(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00308_socket/00308_socket.usd"
    diameter = 0.01950
    height = 0.04931
    base_height = 0.0

@configclass
class Plug_00308(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00308_plug/00308_plug.usd"
    diameter = 0.02078
    height = 0.01285
    mass = 0.019
    base_height = 0.0

@configclass
class Socket_00318(FixedAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00318_socket/00318_socket.usd"
    diameter = 0.00980
    height = 0.03975
    base_height = 0.0

@configclass
class Plug_00318(HeldAssetCfg):
    usd_path = f"{AUTOMATE_ASSET_DIR}/00318_plug/00318_plug.usd"
    diameter = 0.00845
    height = 0.07447
    mass = 0.019
    base_height = 0.0

# 4. AssetPairCfg: 플러그와 소켓 자산을 하나의 구성으로 묶기 위한 클래스  
@configclass
class AssetPairCfg:
    fixed_asset_cfg: FixedAssetCfg = FixedAssetCfg()
    held_asset_cfg: HeldAssetCfg = HeldAssetCfg()

# 5. PlugInsert 작업 구성  
@configclass
class PlugInsert(AutomateTaskDisassembly):
    name: str = "plug_insert"
    # asset_id: 새 asset pair를 선택할 수 있도록 문자열로 설정 (CLI에서 오버라이드 가능)
    asset_id: str = "00004"
    # asset_pair: 선택된 asset_id에 해당하는 자산 쌍을 저장할 필드
    asset_pair: AssetPairCfg = AssetPairCfg()

    asset_size: float = 8.0
    duration_s: float = 10.0

    # 작업 관련 추가 필드
    hand_init_pos: list = [0.0, 0.0, 0.0]  # __post_init__에서 asset_pair의 held_asset_cfg.height로 재설정
    hand_init_pos_noise: list = [0.02, 0.02, 0.01]
    hand_init_orn: list = [3.1416, 0.0, 0.0]
    hand_init_orn_noise: list = [0.0, 0.0, 0.785]

    fixed_asset_init_pos_noise: list = [0.05, 0.05, 0.05]
    fixed_asset_init_orn_deg: float = 0.0
    fixed_asset_init_orn_range_deg: float = 360.0

    held_asset_pos_noise: list = [0.003, 0.0, 0.003]
    held_asset_rot_init: float = 0.0

    keypoint_coef_baseline: list = [5, 4]
    keypoint_coef_coarse: list = [50, 2]
    keypoint_coef_fine: list = [100, 0]
    success_threshold: float = 0.04
    engage_threshold: float = 0.9

    # ArticulationCfg 객체는 __post_init__에서 동적으로 생성함
    fixed_asset: Optional[ArticulationCfg] = None
    held_asset: Optional[ArticulationCfg] = None

    def __post_init__(self):
        # asset_id에 따라 save_data 동적 생성
        #asset_id = f"{self.asset_id}"
        self.save_data = f"asset_{self.asset_id}_disassembly_traj.json"
        # 별도의 매핑 딕셔너리를 사용하여 Plug/Socket 클래스를 선택
        from asset_mapping import asset_mapping
        mapping = asset_mapping.get(self.asset_id)
        if mapping is None:
            raise ValueError(f"Unknown asset_id: {self.asset_id}")
        self.asset_pair.fixed_asset_cfg = mapping["socket"]()
        self.asset_pair.held_asset_cfg = mapping["plug"]()

        # 작업 구성 내 자산 구성 필드를 asset_pair에서 가져옵니다.
        self.fixed_asset_cfg = self.asset_pair.fixed_asset_cfg
        self.held_asset_cfg  = self.asset_pair.held_asset_cfg

        # 예시: hand_init_pos의 z값을 held_asset_cfg.height로 설정
        self.hand_init_pos = [0.0, 0.0, self.held_asset_cfg.height]

        # ArticulationCfg 객체들을 동적으로 생성합니다.
        self.fixed_asset = ArticulationCfg(
            prim_path="/World/envs/env_.*/FixedAsset",
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.fixed_asset_cfg.usd_path,
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
                mass_props=sim_utils.MassPropertiesCfg(mass=self.fixed_asset_cfg.mass),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.6, 0.0, 0.05), 
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={}, 
                joint_vel={}
            ),
            actuators={}
        )

        self.held_asset = ArticulationCfg(
            prim_path="/World/envs/env_.*/HeldAsset",
            spawn=sim_utils.UsdFileCfg(
                usd_path=self.held_asset_cfg.usd_path,
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
                mass_props=sim_utils.MassPropertiesCfg(mass=self.held_asset_cfg.mass),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.005, rest_offset=0.0),
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=(0.0, 0.4, 0.1),
                rot=(1.0, 0.0, 0.0, 0.0),
                joint_pos={},
                joint_vel={}
            ),
            actuators={}
        )

# 6. ConfigStore에 수동 등록 (자동 등록 기능은 제외)
cs = ConfigStore.instance()
# 작업 구성(PlugInsert)을 "task" 그룹에 등록
cs.store(name="PlugInsert", group="task", node=PlugInsert)
# 자산 쌍을 "asset_id" 그룹에 등록 (여기서는 "00004"만 예시로 등록)
asset_pair_00004 = AssetPairCfg(
    fixed_asset_cfg=Socket_00004(),
    held_asset_cfg=Plug_00004()
)
cs.store(name="00004", group="asset_id", node=asset_pair_00004)

asset_pair_00138 = AssetPairCfg(
    fixed_asset_cfg=Socket_00138(),
    held_asset_cfg=Plug_00138()
)
cs.store(name="00138", group="asset_id", node=asset_pair_00138)