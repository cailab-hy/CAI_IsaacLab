# 예: asset_mapping.py

from automate_tasks_disassembly_cfg import Socket_00004, Plug_00004, Socket_00138,Plug_00138
# (필요한 다른 자산 클래스들도 임포트)

# asset_id를 key로, 각 자산에 해당하는 클래스들을 딕셔너리 값으로 저장
asset_mapping = {
    "00004": {"socket": Socket_00004, "plug": Plug_00004},
    "00138": {"socket": Socket_00138, "plug": Plug_00138},
    # 추가 자산은 여기에 계속 추가합니다.
}