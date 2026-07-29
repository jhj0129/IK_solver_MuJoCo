# DROK ARM MuJoCo — Stateful Multi-Box Pick and Place

> README 초안입니다. 사용자가 최종 편집한 뒤 저장소의 README.md에 반영하는 용도입니다.

## 1. 현재 목표

MuJoCo에 배치된 4개의 상자 중 원기둥의 현재 위치를 관리하고,
사용자가 목표 상자 번호만 지정하면 연속적인 Pick and Place를 수행합니다.

동작 흐름:

HOME
→ 현재 상자의 PICK_PREGRASP
→ PICK_APPROACH
→ GRASP
→ PICK_LIFT
→ TRANSFER
→ PLACE_DESCEND
→ RELEASE
→ PLACE_RETREAT
→ 안전 자세로 접기
→ HOME 복귀
→ 원기둥 현재 위치 자동 갱신

## 2. 상자 번호와 위치

ROS/MuJoCo 좌표계 기준:

- +X: 로봇 전방
- +Y: 로봇 왼쪽
- +Z: 위쪽

| BOX | 위치 (x, y, z) [m] | MuJoCo geom |
|---|---:|---|
| 1 | (0.450, +0.180, 0.116) | pickup_pedestal |
| 2 | (0.450, -0.180, 0.116) | place_pedestal |
| 3 | (0.350, -0.450, 0.116) | right_pick_pedestal |
| 4 | (0.350, +0.450, 0.116) | left_place_pedestal |

초기 MuJoCo 상태의 원기둥 위치는 BOX 1입니다.

## 3. 현재 검증 완료된 이동

2026-07-29 MuJoCo 기준:

| 이동 | 결과 | 선택 후보 | 최소 관절 제한 여유 | 최대 관절 점프 | 최종 HOME 오차 |
|---|---|---:|---:|---:|---:|
| BOX 1 → BOX 2 | PASS | 21 | 11.337° | 0.585° | 별도 HOME 복귀 PASS |
| BOX 2 → BOX 3 | PASS | 13 | 7.167° | 0.577° | 0.279° |
| BOX 3 → BOX 2 | PASS | 13 | 31.842° | 0.613° | 0.279° |
| BOX 2 → BOX 1 | PASS | 13 | 경로 생성 PASS | 경로 생성 PASS | 0.278° |

검증된 공통 동작:

- 그리퍼 초기 완전 개방
- HOME 상태 검사
- HOME → PICK_PREGRASP Poly5 진입
- PICK_APPROACH
- 원기둥 파지
- PICK_LIFT
- TRANSFER
- PLACE_DESCEND
- 부분 해제
- 완전 개방
- PLACE_RETREAT
- JOINT1을 유지한 상태에서 JOINT2~6 접기
- 접힌 상태에서 JOINT1을 HOME으로 복귀
- 성공한 경우에만 원기둥 위치 상태 갱신

## 4. 주요 파일

### 설정

- `src/drok_arm_control/config/box_layout.yaml`
  - BOX 1~4 좌표 및 원기둥/받침대 형상 설정

- `src/drok_arm_control/config/cylinder_gripper_grasp.yaml`
  - 그리퍼 개방 및 파지 위치 설정

### 경로 생성

- `src/drok_arm_control/scripts/generate_box_to_box_path.py`
  - `--from-box`, `--to-box`로 이동별 경로 생성
  - grasp candidate 생성
  - Hybrid IK
  - PLACE_RETREAT target 생성
  - PLACE_RETREAT IK
  - 전체 Cartesian path 조립
  - Poly5 시간 파라미터화

### 실행

- `src/drok_arm_control/scripts/execute_box_move.py`
  - 지정된 BOX 간 이동을 실제 MuJoCo에서 실행
  - 선택된 `timed_joint_path.yaml`을 읽어 블록별 실행
  - 경로의 마지막 PLACE_RETREAT 관절값을 자동으로 사용
  - 안전 HOME 복귀 포함

- `src/drok_arm_control/scripts/execute_box_target.py`
  - 저장된 현재 원기둥 위치를 기준으로 목표 BOX만 지정
  - 이동 성공 시에만 현재 BOX 상태 갱신
  - MuJoCo Reset 후 상태 초기화 지원

### 상태 파일

- `runtime_state/cylinder_box_state.yaml`
  - 현재 원기둥 BOX 번호 저장
  - Git 추적 대상에서 제외

## 5. 실행 환경

- Ubuntu 22.04
- ROS 2 Humble
- MuJoCo 3.6
- ros2_control
- Python 3
- PyYAML

저장소 루트:

```bash
~/IK_solver_MuJoCo
```

## 6. MuJoCo 실행

### Terminal 1

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch drok_arm_mujoco drok_arm_mujoco.launch.py
```

## 7. BOX 배치 검증

```bash
cd ~/IK_solver_MuJoCo

python3   src/drok_arm_control/scripts/check_box_layout.py
```

정상 결과:

```text
BOX LAYOUT VALIDATION: PASS
```

## 8. BOX 간 경로 생성

예시: BOX 3에서 BOX 2로 이동하는 경로 생성

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash
source install/setup.bash

python3   src/drok_arm_control/scripts/generate_box_to_box_path.py   --from-box 3   --to-box 2   --candidate-index 13   --overwrite
```

특정 후보가 실패하면 모든 grasp candidate를 검사합니다.

```bash
python3   src/drok_arm_control/scripts/generate_box_to_box_path.py   --from-box 3   --to-box 2   --overwrite
```

생성 결과:

```text
generated_box_paths/
└── box_3_to_2/
    ├── generation_report.yaml
    ├── grasp_candidates.yaml
    ├── grasp_geometry.yaml
    ├── planning_scene.xml
    ├── hybrid_path.yaml
    ├── retreat_target.yaml
    ├── retreat_path.yaml
    ├── full_cartesian_path.yaml
    └── timed_joint_path.yaml
```

## 9. 현재 원기둥 위치 확인

```bash
cd ~/IK_solver_MuJoCo

python3   src/drok_arm_control/scripts/execute_box_target.py   --show-state
```

## 10. 현재 위치 수동 지정

예시: 실제 원기둥이 BOX 3에 있는 경우

```bash
python3   src/drok_arm_control/scripts/execute_box_target.py   --set-current-box 3
```

## 11. MuJoCo Reset 후 상태 초기화

MuJoCo를 Reset하거나 재실행하면 원기둥은 초기 BOX 1로 돌아갑니다.
소프트웨어 상태도 반드시 함께 초기화해야 합니다.

```bash
python3   src/drok_arm_control/scripts/execute_box_target.py   --reset-state
```

## 12. 목표 BOX만 지정해 실행

현재 저장된 위치에서 BOX 2로 이동:

### Dry-run

```bash
python3   src/drok_arm_control/scripts/execute_box_target.py   --to-box 2
```

### 실제 MuJoCo 실행

```bash
python3   src/drok_arm_control/scripts/execute_box_target.py   --to-box 2   --speed-scale 0.5   --execute   --confirmation EXECUTE_MUJOCO_BOX_TARGET
```

성공하면 다음과 같이 상태가 자동 갱신됩니다.

```text
BOX MOVE RESULT: PASS
BOX TARGET STATE UPDATE: PASS
Previous state : BOX 3
Current state  : BOX 2
```

## 13. 직접 출발·도착 BOX를 지정해 실행

자동 상태 관리를 사용하지 않고 직접 실행:

```bash
python3   src/drok_arm_control/scripts/execute_box_move.py   --from-box 2   --to-box 3   --speed-scale 0.5   --execute   --confirmation EXECUTE_MUJOCO_BOX_MOVE
```

## 14. JOINT1 설정

MuJoCo 전용 JOINT1 범위:

```text
-6.5 rad ~ +6.5 rad
```

적용 파일:

- `src/drok_arm_mujoco/urdf/drok_arm_mujoco.urdf`
- `src/drok_arm_mujoco/config/mujoco_inputs.xml`
- `src/drok_arm_kinematics/config/robot_geometry.yaml`

주의:

- 이 범위는 MuJoCo 시뮬레이션 전용입니다.
- 실제 하드웨어에는 케이블, 커넥터, 기구 간섭 검증 없이 적용하면 안 됩니다.
- 현재 장면에서 약 147° 부근까지 회전했을 때 받침대와 충돌한 이력이 있습니다.
- 실제 Pick and Place는 필요한 작업 범위 안에서만 수행합니다.

## 15. 현재 제한사항

- 4개 BOX 간 총 12개 방향 경로가 모두 생성된 상태는 아닙니다.
- 현재 검증된 신규 경로는 2→3, 3→2, 2→1입니다.
- `runtime_state`는 비전 기반 위치 추정이 아니라 성공한 명령을 기준으로 저장됩니다.
- MuJoCo Reset 또는 재실행 후 `--reset-state`가 필요합니다.
- 실제 원기둥이 미끄러지거나 낙하했을 때 자동 감지하지 않습니다.
- BOX 3↔4처럼 긴 좌우 이동은 별도 충돌 검증이 필요합니다.
- 실제 로봇 하드웨어에서는 아직 검증하지 않았습니다.
- 현재 충돌 판단은 MuJoCo 시각 확인과 궤적 검증을 함께 사용합니다.

## 16. 다음 개발 항목

1. 남은 BOX 이동 방향 경로 생성 및 MuJoCo 검증
2. 모든 이동 경로의 충돌 검사 자동화
3. BOX 3↔4 안전 clearance 경로 검증
4. 연속 목표 목록 실행
5. 원기둥 위치를 MuJoCo body pose 또는 카메라로 자동 확인
6. 상태 파일과 실제 물체 위치 불일치 감지
7. 실제 하드웨어용 JOINT1 제한 및 케이블 안전 범위 별도 구성

## 17. 안전 주의사항

이 프로젝트의 현재 결과는 MuJoCo 시뮬레이션 기준입니다.

실제 로봇에서 실행하기 전에 반드시 다음을 별도로 검증해야 합니다.

- JOINT1 케이블 꼬임 범위
- 관절별 실제 소프트웨어 및 하드웨어 제한
- 모터 토크와 속도
- 그리퍼 파지력
- 비상 정지
- 자기 충돌
- 받침대 및 주변 물체 충돌
- 원기둥 낙하
