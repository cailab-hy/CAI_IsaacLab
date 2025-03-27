"""
Omniverse Kit API를 사용하여 00004_plug URDF 파일을 USD로 변환하고 후처리하는 스크립트 예제

- URDF 파일: /home/geun/CAI_IsaacLab/source/isaaclab_assets/data/automate_urdf/urdf/00004_plug.urdf
- 출력 USD 파일: /home/geun/CAI_IsaacLab/source/isaaclab_assets/data/automate_usd/00004_plug/00004_plug.usd

후처리 작업:
  1. 변환된 로봇의 루트 prim에 Articulation Root 적용
  2. OmniPBR 재질을 생성하여 plug 색상 (R: 0.96471, G: 0.93333, B: 0.89804) 적용
  3. Collision prim의 "physics:approximation" 속성을 "SDF Mesh"로 변경
"""

import os
import omni.kit.commands
from pxr import Usd, UsdPhysics, UsdShade, UsdGeom, Gf
from omni.isaac.asset_importer import urdf

# 1. 파일 경로 설정 (절대경로 사용)
urdf_file = "/home/geun/CAI_IsaacLab/source/isaaclab_assets/data/automate_urdf/urdf/00004_plug.urdf"
usd_output = "/home/geun/CAI_IsaacLab/source/isaaclab_assets/data/automate_usd/00004_plug/00004_plug.usd"

# 2. 새로운 Stage 생성 (기존 스테이지를 초기화)
omni.usd.get_context().new_stage()

# 3. URDF Import 설정 구성
# (여기서는 plug 파일의 경우 --fix-base 옵션은 사용하지 않습니다)
import_config = urdf.ImportConfig()
import_config.fix_base = False          # 플러그는 고정 베이스가 아니므로 False (socket에는 --fix-base 적용)
import_config.merge_fixed_joints = True   # 고정 조인트 병합 옵션 (필요시)

# 4. URDF 파일을 USD로 임포트 (URDFParseAndImportFile 명령 사용)
result, prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=urdf_file,
    import_config=import_config,
    dest_path=usd_output
)
print("URDF 임포트 결과, 루트 prim 경로:", prim_path)

# 5. USD Stage 및 로봇 prim 획득
stage = omni.usd.get_context().get_stage()
robot_prim = stage.GetPrimAtPath(prim_path)
if not robot_prim:
    raise RuntimeError("URDF 임포트 실패: 로봇 prim을 찾을 수 없습니다.")

# 6. Articulation Root 적용 (물리 시뮬레이션에 필수)
UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
print("Articulation Root 적용 완료")

# 7. 재질(Material) 생성 및 모든 Mesh prim에 바인딩
# 플러그의 색상: R=0.96471, G=0.93333, B=0.89804
material_path = str(robot_prim.GetPath()) + "/Looks/OmniPBR"
material = UsdShade.Material.Define(stage, material_path)
shader = UsdShade.Shader.Define(stage, material_path + "/Shader")
shader.CreateIdAttr("UsdPreviewSurface")
# Albedo Color (diffuseColor) 값 설정
shader.CreateInput("diffuseColor", UsdShade.Tokens.Color3f).Set(Gf.Vec3f(0.96471, 0.93333, 0.89804))
shader.CreateInput("roughness", UsdShade.Tokens.Float).Set(0.3)  # roughness 값은 필요에 따라 조정
material.CreateSurfaceOutput().ConnectToSource(shader, "surface")
# Stage 내의 모든 Mesh prim에 대해 재질 바인딩
for prim in stage.Traverse():
    if prim.IsA(UsdGeom.Mesh):
        UsdShade.MaterialBindingAPI(prim).Bind(material)
print("재질(OmniPBR) 적용 완료")

# 8. Collision 설정 변경: 모든 충돌체 prim의 physics:approximation 값을 "SDF Mesh"로 변경
for prim in stage.Traverse():
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        approx_attr = prim.GetAttribute("physics:approximation")
        if approx_attr:
            approx_attr.Set("SDF Mesh")
print("Collision 설정 변경 완료 (SDF Mesh 적용)")

# 9. 변경 사항 저장
stage.GetRootLayer().Save()
print("최종 USD 저장 완료:", usd_output)
