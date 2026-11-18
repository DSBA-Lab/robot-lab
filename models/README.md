# 3D 모델

배치도 페이지에서 로봇을 선택하면 오른쪽에 뜨는 3D 모델입니다. 각 로봇의 공개 URDF와 메시를
`tools/build_models.py`로 변환해 만들었습니다 (메시 단순화, 법선 재계산, 관절 구조를 유지한 GLB로 묶기).
관절 축과 가동 범위는 각 관절 노드의 `extras`에 들어 있습니다.

| 파일 | 원본 | 라이선스 | 비고 |
|---|---|---|---|
| `openarm2.glb` | [enactic/openarm_description](https://github.com/enactic/openarm_description) `assets/robot/openarm_v2.0/urdf/example/v2.urdf` | Apache-2.0 | 양팔 + 핀치 그리퍼. 카메라 마운트는 제외 |
| `xarm7.glb` | [xArm-Developer/xarm_ros2](https://github.com/xArm-Developer/xarm_ros2) `xarm_description` (xarm7) | BSD-3-Clause | 그리퍼 없음 |
| `so101.glb` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) `Simulation/SO101/so101_new_calib.urdf` | Apache-2.0 | 팔로워 암 |

원본 라이선스 전문은 `licenses/` 폴더에 있습니다. 원본 메시를 단순화하고 형식을 바꾼 것 외에 형상은 수정하지 않았습니다.

## 다시 만들기

```bash
pip install numpy fast-simplification
python3 tools/build_models.py <원본을 내려받은 폴더> models
```

`<원본 폴더>` 아래에 `so101/`, `xarm7/`, `openarm/`으로 각 저장소의 해당 경로를 그대로 두면 됩니다.
