# DROK ARM MuJoCo Pick-and-Place Validation

검증 날짜: 2026-07-24

## 검증 상태

현재 코드는 MuJoCo 시뮬레이션 환경에서 80 mm 큐브를 대상으로 다음 동작을 완료했다.

1. Home 자세에서 PICK pre-grasp 이동
2. Cartesian 접근
3. 그리퍼 닫기 및 preload
4. 큐브 수직 상승
5. 안전 위치로 이동
6. 중앙 위치에서 wrist IK branch 전환
7. PLACE 상공 이동
8. 수직 하강
9. 그리퍼 열기
10. 수직 상승
11. 역방향 wrist branch 복귀
12. Home 자세 복귀

## 검증된 작업 설정

- Pickup cube center: `[0.450, 0.180, 0.272]`
- Place cube center: `[0.450, -0.180, 0.272]`
- Cube size: `80 x 80 x 80 mm`
- Cube mass: 약 `0.256 kg`
- Safe transfer height: `z = 0.360 m`
- Wrist branch-switch center: `[0.280, 0.000, 0.400]`
- Wrist branch trajectory: Poly5
- Branch-switch samples: `121`
- Minimum software joint-limit margin: `0.050 rad`

## MuJoCo 충돌 및 파지 설정

URDF-to-MJCF 변환 결과를 패치하기 위해 다음 구조를 사용한다.

- Raw MJCF topic: `/mujoco_robot_description_raw`
- Patched MJCF topic: `/mujoco_robot_description`
- Patch script:
  `src/drok_arm_mujoco/scripts/patch_mjcf_gripper_collision.py`

그리퍼 패드에는 단순 box collision을 사용하며, 큐브와의 명시적인 contact pair 및 마찰 파라미터를 적용했다.

## 주요 실행 파일

- Task configuration:
  `src/drok_arm_control/config/pick_place_task.yaml`
- Task node:
  `src/drok_arm_control/src/pick_place_task_node.cpp`
- MJCF patcher:
  `src/drok_arm_mujoco/scripts/patch_mjcf_gripper_collision.py`
- Robot geometry:
  `src/drok_arm_kinematics/config/robot_geometry.yaml`

## 중요 제한 사항

현재 V4 경로는 **MuJoCo 시뮬레이션 검증 전용**이다.

중앙 wrist branch-switch 과정에서 대략 다음 관절 변화가 발생한다.

- JOINT4: 약 179도 회전
- JOINT6: 약 179도 회전
- JOINT5: 0 rad 부근의 wrist singularity 통과

DROK ARM 실제 기체는 손목 외부에 배선이 있으므로 이 동작을 그대로 실행하면 배선 꼬임, 장력 증가, 커넥터 손상이 발생할 수 있다.

따라서 현재 V4 코드는 실제 로봇 실행 승인을 받지 않은 상태다.

## 실제 로봇 적용 전 필요한 작업

1. JOINT4, JOINT5, JOINT6의 배선 중립 자세 측정
2. 배선 기준 양방향 hard limit 측정
3. 여유를 둔 cable soft limit 정의
4. Wrist branch flip 검출 및 금지
5. 동일 wrist branch를 유지하는 PICK/PLACE pose 탐색
6. Cable-aware IK 비용함수 추가
7. 실제 팔용 V5 planner 별도 구현

## 현재 판정

| 항목 | 상태 |
|---|---|
| MuJoCo IK 및 Cartesian planning | 통과 |
| MuJoCo 그리퍼 파지 | 통과 |
| MuJoCo 큐브 이동 및 배치 | 통과 |
| 배치 후 안전 복귀 | 통과 |
| 실제 로봇 배선 안전성 | 미검증 |
| 실제 로봇 실행 | 금지 |

이 버전은 MuJoCo 기능 검증과 향후 cable-aware planner 개발을 위한 기준 버전으로 보관한다.
