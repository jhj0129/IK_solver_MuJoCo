# Pick and Place 1 Way

DROK ARM의 MuJoCo 기반 단방향 원기둥 Pick-and-Place 데모입니다.

## 구현된 동작

1. HOME 상태 확인
2. 그리퍼 완전 개방
3. 원기둥 PICK 위치 접근
4. 원기둥 파지
5. 수직 50 mm 상승
6. 반대편 받침대 상공으로 이동
7. 수직 50 mm 하강
8. 그리퍼 단계적 개방
9. 원기둥으로부터 약 80 mm 후퇴

현재 버전은 `PLACE_RETREAT` 종료점에서 끝납니다.  
최종 HOME 복귀와 다방향 연속동작은 다음 개발 단계에서 추가합니다.

## 검증 환경

- 운영체제: Ubuntu 22.04
- ROS 2: Humble
- 시뮬레이터: MuJoCo
- 팔 제어기: `arm_controller`
- 그리퍼 제어기: `gripper_controller`
- 상태 broadcaster: `joint_state_broadcaster`
- 팔 관절: `JOINT1` ~ `JOINT6`
- 그리퍼 관절:
  - `JOINT7`
  - `GRIPPER_RIGHT_JOINT`
- 원기둥 지름: 60 mm
- 최종 그리퍼 파지 명령:
  - `JOINT7 = +0.0488 m`
  - `GRIPPER_RIGHT_JOINT = -0.0488 m`

> 현재 그리퍼 범위와 파지값은 MuJoCo 시뮬레이션에서만 검증되었습니다.  
> 실제 로봇 하드웨어에는 그대로 적용하지 않습니다.

## 재부팅 후 바로 실행

컴퓨터를 껐다 켠 뒤 터미널 하나를 열고 다음 명령만 실행합니다.

```bash
cd ~/IK_solver_MuJoCo
./demo_pick_and_place_1_way.sh --execute
```

실행 스크립트가 자동으로 다음 작업을 수행합니다.

```text
MuJoCo 실행
→ ROS 2 제어기 준비 대기
→ Pick-and-Place 전체 실행
→ PLACE_RETREAT 종료점에서 정지
```

동작이 끝난 뒤 MuJoCo는 계속 실행됩니다.  
종료하려면 실행 터미널에서 `Ctrl+C`를 누릅니다.

## 직접 실행

### Terminal 1 — MuJoCo 실행

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch   drok_arm_mujoco   drok_arm_mujoco.launch.py
```

### Terminal 2 — Pick-and-Place 실행

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

src/drok_arm_control/scripts/execute_cylinder_pick_place_full.sh   --execute   --confirmation EXECUTE_MUJOCO_CYLINDER_PICK_PLACE
```

## 터미널 출력

통합 실행기는 상세 로그 대신 현재 동작만 간단하게 표시합니다.

```text
[시작] MuJoCo 원기둥 Pick-and-Place
[시작 대기] 제어기 및 액션 서버 확인
[현재 동작] 그리퍼 초기 개방
[현재 동작] 홈 위치 및 개방 상태 확인
[현재 동작] 원기둥 파지 위치로 접근
[현재 동작] 원기둥 파지
[현재 동작] 원기둥 들어 올리기
[현재 동작] 원기둥 운반
[현재 동작] 원기둥 내려놓기
[현재 동작] 그리퍼 1차 해제
[현재 동작] 그리퍼 완전 개방
[현재 동작] 원기둥에서 후퇴
[종료] Pick-and-Place 완료
```

## 로그 위치

전체 Pick-and-Place 실행 로그:

```text
~/IK_solver_MuJoCo/run_logs/cylinder_pick_place_full_*.log
```

MuJoCo launch 로그:

```text
~/IK_solver_MuJoCo/run_logs/pick_place_1_way_mujoco_*.log
```

## 현재 완료 상태

- PICK 접근: PASS
- 원기둥 파지: PASS
- PICK LIFT: PASS
- TRANSFER: PASS
- PLACE DESCEND: PASS
- 그리퍼 해제: PASS
- PLACE RETREAT: PASS
- 전체 단방향 Pick-and-Place: PASS

## 다음 개발 단계

1. 현재 동작 종료 후 HOME 복귀 추가
2. 후반부 중복 그리퍼 개방 확인 단계 제거
3. 좌측·우측·후방 받침대 추가
4. JOINT1의 MuJoCo 전용 회전 범위 확장
5. 원기둥 하나를 이용한 연속 Pick-and-Place 구현
