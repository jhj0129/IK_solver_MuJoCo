# DROK ARM — Meta Quest 3 VR Target Pick-and-Place in MuJoCo

Meta Quest 3에서 받침대 위 목표점을 선택하면 Ubuntu의 ROS 2 노드가 목표 좌표를 수신하고, 자체 IK 및 궤적 생성 코드를 이용해 DROK 6축 로봇팔이 MuJoCo에서 원기둥을 집어 목표 위치로 옮기는 프로젝트입니다.

현재 브랜치 기준으로 다음 기능을 포함합니다.

- DROK ARM MuJoCo 시뮬레이션
- ROS 2 Humble 및 `ros2_control`
- Meta Quest 3 기반 목표 위치 선택
- Unity ROS-TCP-Connector와 ROS-TCP-Endpoint 통신
- 자체 Numerical IK
- Position-only / Full-pose / Upright IK
- 원기둥 둘레 Grasp candidate 생성 및 선택
- VR에서 선택한 정확한 목표 좌표로 Pick-and-Place
- Cubic spline + Poly5 기반 시간 파라미터화
- `FollowJointTrajectory` Action 기반 팔·그리퍼 실행
- `/joint_states` 기반 Unity 로봇 모델 동기화
- 실행 단계 기반 Unity 원기둥 시각화
- MuJoCo 재시작 시 논리 상태 초기화 지원

---

## 1. 시스템 구조

```text
Meta Quest 3
  └─ 오른쪽 컨트롤러 Ray로 목표점 선택
      ├─ Trigger: 목표점 선택
      └─ A 버튼: 목표 확정
              │
              │ /vr/place_target
              │ geometry_msgs/msg/PoseStamped
              ▼
Unity ROS-TCP-Connector
              │ Wi-Fi / TCP 10000
              ▼
ROS-TCP-Endpoint
              │
              ▼
vr_pick_place_fast_node.py
  ├─ 목표 BOX 및 안전 영역 판정
  ├─ 현재 원기둥 논리 위치 확인
  ├─ Grasp candidate 선택
  ├─ Cartesian 목표 생성
  ├─ Numerical IK
  ├─ 관절 경로 생성
  └─ Poly5 시간 파라미터화
              │
              │ FollowJointTrajectory
              ▼
arm_controller / gripper_controller
              │
              ▼
ros2_control + MuJoCo
              │
              └─ /joint_states 약 100 Hz
```

VR은 로봇의 모든 관절을 실시간으로 직접 조종하지 않습니다.

```text
VR에서 목표 Pose를 1회 전송
→ Ubuntu에서 IK와 전체 궤적 계산
→ 완성된 JointTrajectory를 MuJoCo controller에 전송
```

---

## 2. 실행 환경

검증 환경:

```text
Ubuntu 22.04
ROS 2 Humble
MuJoCo
Python 3
Meta Quest 3
Unity 6
Unity ROS-TCP-Connector
ROS-TCP-Endpoint
기본 TCP 포트: 10000
```

저장소 기본 위치:

```text
~/IK_solver_MuJoCo
```

---

## 3. 처음 Clone한 경우

```bash
cd ~

git clone \
  --branch feature/random-known-pose-pick \
  https://github.com/jhj0129/IK_solver_MuJoCo.git

cd ~/IK_solver_MuJoCo
```

ROS 의존성 설치:

```bash
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  -r \
  -y
```

빌드:

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash

colcon build --symlink-install
```

빌드 후:

```bash
source ~/IK_solver_MuJoCo/install/setup.bash
```

---

# 4. 컴퓨터를 껐다 켠 뒤 실행

VR Pick-and-Place를 실행할 때는 아래 3개 Terminal을 사용합니다.

## Terminal 1 — MuJoCo 실행

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch \
  drok_arm_mujoco \
  drok_arm_mujoco.launch.py
```

정상 상태에서는 다음 controller가 `active`여야 합니다.

```text
arm_controller
gripper_controller
joint_state_broadcaster
```

확인:

```bash
ros2 control list_controllers
```

---

## Terminal 2 — ROS-TCP Endpoint 실행

현재 Ubuntu IP 확인:

```bash
hostname -I
```

예시:

```text
192.168.35.223
```

ROS-TCP Endpoint 실행:

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run \
  ros_tcp_endpoint \
  default_server_endpoint \
  --ros-args \
  -p ROS_IP:=192.168.35.223 \
  -p ROS_TCP_PORT:=10000
```

`192.168.35.223`은 현재 Ubuntu IP로 바꿉니다.

Quest Unity 앱의 ROS IP 설정도 동일한 주소여야 합니다.

```text
ROS IP   = Ubuntu IP
ROS Port = 10000
ROS Mode = ROS 2
```

주의:

```text
Unity Editor와 Quest 앱을 동시에 ROS-TCP Endpoint에 연결하지 않습니다.
Quest 또는 Unity Editor 중 하나만 연결합니다.
```

---

## Terminal 3 — VR FAST Pick-and-Place 실행

### MuJoCo까지 새로 실행한 직후

MuJoCo를 새로 실행하면 실제 원기둥은 BOX 1로 초기화되므로 `--fresh-mujoco`를 사용합니다.

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 \
  src/drok_arm_control/scripts/vr_pick_place_fast_node.py \
  --speed-scale 2.5 \
  --fresh-mujoco \
  --execute \
  --confirmation EXECUTE_MUJOCO_VR_PICK_PLACE
```

정상 시작 로그:

```text
Current BOX : 1
Center      : [0.450000, 0.180000, 0.272000]
LOGICAL_CYLINDER_STATE: BOX 1
```

### 같은 MuJoCo 세션에서 실행 노드만 다시 시작

원기둥이 이미 다른 BOX로 이동한 상태라면 `--fresh-mujoco`를 사용하지 않습니다.

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 \
  src/drok_arm_control/scripts/vr_pick_place_fast_node.py \
  --speed-scale 2.5 \
  --execute \
  --confirmation EXECUTE_MUJOCO_VR_PICK_PLACE
```

---

# 5. Quest 3 조작

```text
오른쪽 컨트롤러 Ray로 받침대 위를 조준
→ Trigger로 목표점 선택
→ A 버튼으로 목표 확정
```

A 버튼을 누르면 다음 토픽으로 목표가 한 번 전송됩니다.

```text
Topic: /vr/place_target
Type : geometry_msgs/msg/PoseStamped
```

VR에서 전송하는 핵심 정보:

```text
목표 원기둥 중심 x
목표 원기둥 중심 y
목표 원기둥 중심 z
```

그리퍼의 접근 자세는 VR이 정하지 않고 Ubuntu의 Grasp candidate 및 IK 코드가 계산합니다.

---

# 6. 좌표계

ROS/MuJoCo 좌표계:

```text
+X: 로봇 전방
+Y: 로봇 왼쪽
+Z: 위쪽
```

Unity local 좌표에서 ROS 좌표로 변환:

```text
ROS x =  Unity local z
ROS y = -Unity local x
ROS z =  Unity local y
```

---

# 7. BOX 위치

받침대 중심 위치:

| BOX | x [m] | y [m] | 받침대 중심 z [m] |
|---:|---:|---:|---:|
| BOX 1 | 0.450 | +0.180 | 0.116 |
| BOX 2 | 0.450 | -0.180 | 0.116 |
| BOX 3 | 0.350 | -0.450 | 0.116 |
| BOX 4 | 0.350 | +0.450 | 0.116 |

초기 원기둥 중심:

```text
BOX 1
[0.450, 0.180, 0.272] m
```

VR에서는 BOX 중심에만 놓는 것이 아니라 받침대 안전 영역 안의 정확한 선택 좌표에 원기둥을 배치할 수 있습니다.

---

# 8. 전체 Pick-and-Place 순서

```text
HOME
→ GRIPPER OPEN
→ PICK_PREGRASP
→ PICK_APPROACH
→ GRASP
→ PICK_LIFT
→ TRANSFER
→ PLACE_DESCEND
→ RELEASE
→ PLACE_RETREAT
→ HOME
```

FAST 실행 로그의 대표 단계:

```text
STARTED
GRIPPER_OPEN
ENTRY_TO_PICK_PREGRASP
PICK_APPROACH
GRASPED
PICK_LIFT
TRANSFER
PLACE_DESCEND
RELEASED
PLACE_RETREAT
RETURN_HOME
COMPLETE
```

---

# 9. IK 방식

현재 프로젝트는 MoveIt IK를 사용하지 않고 자체 C++ IK solver를 사용합니다.

정확한 방식:

```text
Numerical Jacobian
+ Damped Least-Squares
+ URDF joint limit
+ seed continuity
+ Upright null-space control
```

주요 파일:

```text
src/drok_arm_kinematics/src/inverse_kinematics.cpp
src/drok_arm_kinematics/src/solve_ik_pose.cpp
src/drok_arm_kinematics/src/solve_ik_upright.cpp
```

## 9.1 Damped Least-Squares

Pose 오차:

```text
위치 오차
ep = pd - p(q)

회전 오차
eR = log(Rd R(q)^T)
```

Damped pseudoinverse:

```text
J# = J^T (J J^T + λ²I)^-1
```

관절 변화:

```text
Δq = J# e
```

주요 기본값:

```text
damping λ          = 0.01
numerical delta    = 1e-6
maximum joint step = 0.05 rad
```

---

## 9.2 Numerical Jacobian

각 관절을 아주 조금씩 변화시킨 뒤 FK 변화를 이용해 Jacobian을 계산합니다.

```text
현재 FK 1회
JOINT1 perturbation FK
JOINT2 perturbation FK
JOINT3 perturbation FK
JOINT4 perturbation FK
JOINT5 perturbation FK
JOINT6 perturbation FK
```

6축 기준으로 IK iteration 1회에 대략 7회의 FK 계산이 수행됩니다.

---

## 9.3 Position-only IK

제약:

```text
TCP x, y, z만 맞춤
TCP 방향은 자유
```

용도:

```text
현재 자세에서 PICK_PREGRASP 위치 근처의 안정적인 seed 생성
```

초기 full-pose IK를 바로 계산하지 않고 먼저 position-only IK를 계산해 수렴 안정성을 높입니다.

---

## 9.4 Full-pose IK

제약:

```text
TCP x, y, z
TCP roll, pitch, yaw
```

사용 구간:

```text
PICK_PREGRASP 자세 정렬
PICK_APPROACH
PICK_LIFT
```

원기둥을 잡는 구간에서는 그리퍼의 접근 방향과 파지 방향을 정확히 유지합니다.

---

## 9.5 Upright 5-DoF IK

제약:

```text
TCP 위치 x, y, z
TCP local +Z = world +Z
TCP yaw는 자유
```

Task 차원:

```text
위치 3
+ upright 방향 2
= 5
```

사용 구간:

```text
TRANSFER
PLACE_DESCEND
```

6축 로봇에서 5차원 task를 사용하기 때문에 null space 1자유도가 남습니다.

Null-space 보조 목적:

```text
직전 waypoint 관절 자세 유지
관절 제한 근처 회피
```

---

# 10. Hybrid IK 구성

```text
현재 /joint_states
→ Position-only IK
→ Full-pose PICK_PREGRASP IK
→ Full-pose PICK_APPROACH IK
→ Full-pose PICK_LIFT IK
→ Upright TRANSFER IK
→ Upright PLACE_DESCEND IK
→ 최종 접근축 기준 PLACE_RETREAT IK
```

PLACE_RETREAT은 별도로 계산합니다.

이유:

```text
Upright IK에서는 yaw가 자유롭게 변할 수 있음
→ PLACE_GRASP 최종 TCP 방향 확인
→ 실제 접근축 반대 방향으로 Retreat 목표 생성
→ Retreat IK 계산
```

---

# 11. Grasp candidate

원기둥은 회전대칭이므로 여러 방향에서 파지할 수 있습니다.

표준 생성기는 원기둥 둘레를 36개 방향으로 나눕니다.

```text
360° / 36 = 10° 간격
```

각 candidate에는 다음 내용이 포함됩니다.

```text
파지 방위각
TCP 회전행렬
PICK_PREGRASP
PICK_GRASP
PICK_LIFT
PLACE_LIFT
PLACE_GRASP
PLACE_RETREAT
```

FAST 버전은 계산시간을 줄이기 위해 BOX별 shortlist를 먼저 검사합니다.

```text
BOX 1: 21, 20, 22
BOX 2: 13, 14, 15, 16
BOX 3: 13, 12, 14
BOX 4: 23, 22, 24, 21, 20, 18
```

앞 후보가 실패하면 다음 후보를 자동으로 시도합니다.

---

# 12. 표준 Cartesian IK와 FAST IK

## 표준 방식

기본 Cartesian step:

```text
0.003 m = 3 mm
```

구간 길이가 `L`일 때 IK waypoint 수:

```text
N = ceil(L / 0.003)
```

예:

```text
0.30 m 이동
→ 약 100개 Cartesian waypoint
→ 각 waypoint에서 IK 계산
```

장점:

```text
End-Effector가 Cartesian 직선 경로에 가깝게 이동
관절 branch 연속성 검증이 촘촘함
```

단점:

```text
IK 호출 횟수가 많아 계산시간이 김
```

## 현재 FAST 방식

현재 FAST 버전은 각 구간의 주요 Cartesian endpoint만 계산합니다.

Candidate 하나 기준 대략:

```text
Position-only seed          1회
Full-pose PICK_PREGRASP     1회
PICK_APPROACH endpoint      1회
PICK_LIFT endpoint          1회
TRANSFER endpoint           1회
PLACE_DESCEND endpoint      1회
PLACE_RETREAT endpoint      1회
```

즉 대략 7회의 IK 호출로 경로를 만듭니다.

정확한 분류:

```text
Cartesian endpoint 기반 IK
+ endpoint 사이 joint-space trajectory
```

FAST 방식은 표준 3 mm Cartesian 방식보다 빠르지만 중간 End-Effector 경로는 완전한 직선이 아닐 수 있습니다.

---

# 13. 궤적 생성

IK로 얻은 관절 waypoint:

```text
q0, q1, q2, ..., qN
```

관절 waypoint를 natural cubic spline으로 연결합니다.

```text
q = q(s)
```

그 후 진행률 `s`에 Poly5 시간법칙을 적용합니다.

```text
s(τ) = 10τ³ - 15τ⁴ + 6τ⁵
τ = t / T
```

최종 궤적:

```text
q(t) = q(s(t))
```

Poly5는 시작과 끝에서 다음 조건을 만족합니다.

```text
속도 = 0
가속도 = 0
```

시간 파라미터화 과정에서 다음을 확인합니다.

```text
관절 위치 제한
관절 속도 제한
관절 가속도 제한
관절 jerk 제한
```

---

# 14. MuJoCo로 전달하는 방식

관절 명령을 Python에서 10 ms마다 직접 Publish하지 않습니다.

완성된 `JointTrajectory` 전체를 하나의 Action Goal로 보냅니다.

팔:

```text
/arm_controller/follow_joint_trajectory
```

그리퍼:

```text
/gripper_controller/follow_joint_trajectory
```

메시지:

```text
control_msgs/action/FollowJointTrajectory
```

Trajectory 각 point에는 다음이 포함됩니다.

```text
positions
velocities
accelerations
time_from_start
```

Controller가 Goal을 받은 뒤 내부 주기에 맞춰 실행합니다.

---

# 15. 주기와 전송 속도

```text
ros2_control update rate : 약 100 Hz
JointTrajectory 기준     : 약 100 Hz
/joint_states            : 약 100 Hz
```

VR 목표 전송은 주기적 스트리밍이 아닙니다.

```text
A 버튼 1회
→ PoseStamped 메시지 1개
→ 전체 Pick-and-Place 1회
```

`/joint_states`는 Quest Unity의 로봇 모델 시각화에 사용합니다.

---

# 16. speed-scale

현재 FAST 권장값:

```text
2.5
```

개념적으로:

```text
시간         ÷ 2.5
속도         × 2.5
가속도       × 2.5²
```

FAST 실행은 MuJoCo 시뮬레이션 전용입니다.

실제 로봇에서는 속도·가속도·토크·케이블·충돌 검증 없이 그대로 사용하면 안 됩니다.

---

# 17. 원기둥 상태 관리

현재 VR Pick-and-Place는 성공한 명령을 기준으로 원기둥의 논리 위치를 저장합니다.

대표 상태 파일:

```text
runtime_state/vr_cylinder_state.yaml
```

저장 내용:

```text
현재 BOX
현재 원기둥 중심 x, y, z
마지막 성공 이동
갱신 시각
```

동작 성공 시에만 상태를 갱신합니다.

```text
FAILED
→ 상태 유지

COMPLETE
→ 목표 BOX와 정확한 목표 좌표로 갱신
```

---

# 18. MuJoCo 재시작 후 상태 초기화

MuJoCo를 재실행하면 실제 원기둥은 BOX 1로 돌아갑니다.

VR 논리 상태도 BOX 1로 맞춰야 합니다.

가장 권장되는 방법:

```bash
python3 \
  src/drok_arm_control/scripts/vr_pick_place_fast_node.py \
  --speed-scale 2.5 \
  --fresh-mujoco \
  --execute \
  --confirmation EXECUTE_MUJOCO_VR_PICK_PLACE
```

상태만 초기화:

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 \
  src/drok_arm_control/scripts/vr_pick_place_fast_node.py \
  --reset-state
```

---

# 19. Unity 시각화 동기화

## 로봇팔

Unity는 다음 토픽을 구독합니다.

```text
/joint_states
```

관절 매핑:

```text
JOINT1 → LINK1
JOINT2 → LINK2
JOINT3 → LINK3
JOINT4 → LINK4
JOINT5 → LINK5
JOINT6 → LINK6
JOINT7 → GRIPPER_LEFT
GRIPPER_RIGHT_JOINT → GRIPPER_RIGH
```

## 원기둥

Unity 원기둥은 실행 stage를 이용해 시각화합니다.

```text
GRASPED
→ 원기둥을 gripper_center에 parent

RELEASED
→ parent 해제

COMPLETE
→ 최종 논리 목표 좌표로 정렬
```

관련 토픽:

```text
/pick_place/execution_stage
/pick_place/cylinder_pose
/pick_place/status
```

현재 방식은 Unity 시각화용 논리 동기화이며 MuJoCo의 원기둥 free-body Pose를 직접 측정하는 방식은 아닙니다.

---

# 20. 표준 BOX 이동 테스트

VR 없이 MuJoCo에서 BOX 1 → BOX 2를 테스트합니다.

## Terminal 1 — MuJoCo

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch \
  drok_arm_mujoco \
  drok_arm_mujoco.launch.py
```

## Terminal 2 — 경로 생성 및 실행

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 \
  src/drok_arm_control/scripts/generate_box_to_box_path.py \
  --from-box 1 \
  --to-box 2 \
  --candidate-index 21 \
  --overwrite

python3 \
  src/drok_arm_control/scripts/execute_box_target.py \
  --reset-state

python3 \
  src/drok_arm_control/scripts/execute_box_target.py \
  --to-box 2 \
  --speed-scale 0.5 \
  --execute \
  --confirmation EXECUTE_MUJOCO_BOX_TARGET
```

BOX 2 → BOX 3:

```bash
python3 \
  src/drok_arm_control/scripts/generate_box_to_box_path.py \
  --from-box 2 \
  --to-box 3 \
  --candidate-index 13 \
  --overwrite

python3 \
  src/drok_arm_control/scripts/execute_box_target.py \
  --to-box 3 \
  --speed-scale 0.5 \
  --execute \
  --confirmation EXECUTE_MUJOCO_BOX_TARGET
```

BOX 1 → BOX 2가 끝난 뒤에는 `--reset-state`를 다시 사용하지 않습니다.

---

# 21. 주요 파일

```text
IK_solver_MuJoCo/
├─ src/
│  ├─ drok_arm_kinematics/
│  │  ├─ src/inverse_kinematics.cpp
│  │  ├─ src/solve_ik_pose.cpp
│  │  └─ src/solve_ik_upright.cpp
│  │
│  ├─ drok_arm_control/
│  │  ├─ config/
│  │  │  ├─ box_layout.yaml
│  │  │  ├─ cylinder_grasp_geometry.yaml
│  │  │  └─ cylinder_trajectory_timing_fast.yaml
│  │  │
│  │  └─ scripts/
│  │     ├─ vr_pick_place_fast_node.py
│  │     ├─ generate_fast_known_object_to_pose_path.py
│  │     ├─ generate_box_to_box_path_fast.py
│  │     ├─ dry_run_cylinder_hybrid_ik_fast.py
│  │     ├─ dry_run_cylinder_place_retreat_fast.py
│  │     ├─ assemble_cylinder_full_path.py
│  │     ├─ time_parameterize_cylinder_path.py
│  │     ├─ execute_box_move_fast.py
│  │     ├─ generate_box_to_box_path.py
│  │     ├─ execute_box_move.py
│  │     └─ execute_box_target.py
│  │
│  └─ drok_arm_mujoco/
│     ├─ launch/drok_arm_mujoco.launch.py
│     ├─ model/scene.xml
│     ├─ urdf/drok_arm_mujoco.urdf
│     └─ config/controllers.yaml
│
├─ generated_box_paths/
├─ generated_vr_target_paths/
├─ runtime_state/
└─ run_logs/
```

생성 경로와 실행 로그는 GitHub에 올리지 않는 것을 권장합니다.

```text
build/
install/
log/
run_logs/
generated_box_paths/
generated_vr_target_paths/
runtime_state/*.yaml
```

---

# 22. 자주 발생하는 문제

## `/joint_states`가 없음

```bash
ros2 topic hz /joint_states
```

확인:

```bash
ros2 control list_controllers
```

`joint_state_broadcaster`가 `active`인지 확인합니다.

---

## Action server가 없음

```bash
ros2 action list
```

정상적으로 다음 Action이 있어야 합니다.

```text
/arm_controller/follow_joint_trajectory
/gripper_controller/follow_joint_trajectory
```

---

## Quest가 ROS에 연결되지 않음

```text
Ubuntu IP 확인
Quest와 Ubuntu가 같은 Wi-Fi인지 확인
TCP 10000 포트 확인
Unity Editor와 Quest 중 하나만 연결
```

---

## 원기둥의 VR 위치와 MuJoCo 위치가 다름

MuJoCo를 재실행했다면 FAST 노드를 반드시 `--fresh-mujoco`로 시작합니다.

---

## `HYBRID IK FAIL (status=2)`

```text
초기 Position-only IK 실패
```

실제 원기둥 위치와 논리 상태가 일치하는지 확인합니다.

---

## `HYBRID IK FAIL (status=3)`

```text
초기 Full-pose IK 실패
```

현재 candidate의 파지 방향이 해당 원기둥 위치에서 수렴하지 않은 경우입니다. FAST shortlist의 다음 candidate를 시도합니다.

---

## `TIME PARAMETERIZATION FAIL`

IK waypoint는 생성됐지만 spline 경로가 다음 중 하나를 만족하지 못한 경우입니다.

```text
관절 제한
속도 제한
가속도 제한
jerk 제한
```

다른 candidate 또는 더 보수적인 속도를 사용합니다.

---

# 23. 현재 제한사항

- FAST 경로는 모든 중간 Cartesian waypoint를 계산하지 않습니다.
- 중간 End-Effector 경로가 완전한 Cartesian 직선이 아닐 수 있습니다.
- MuJoCo의 실제 원기둥 slip 또는 낙하를 자동 감지하지 않습니다.
- 원기둥 상태는 성공한 명령을 기준으로 관리합니다.
- MuJoCo 재실행 후 상태 초기화가 필요합니다.
- Unity 원기둥은 실행 stage 기반 시각화입니다.
- 현재 FAST 속도 설정은 MuJoCo 시뮬레이션 전용입니다.
- 실제 DROK ARM 하드웨어에서는 아직 동일 조건으로 검증되지 않았습니다.
- 실제 하드웨어 적용 전 충돌, 케이블, 관절 제한, 모터 속도·가속도 및 비상 정지를 별도로 검증해야 합니다.

---

# 24. 현재 완성 상태

현재 구현된 핵심 기능:

```text
Quest 3 목표점 선택
→ ROS Pose 전송
→ 목표 BOX 및 정확한 목표 좌표 판정
→ 현재 원기둥 좌표 기반 Grasp candidate 생성
→ Position-only / Full-pose / Upright Hybrid IK
→ PLACE_RETREAT IK
→ Cubic spline + Poly5 궤적 생성
→ FollowJointTrajectory 실행
→ 원기둥 논리 상태 갱신
→ Quest Unity 로봇 및 원기둥 시각화
→ 반복 Pick-and-Place
```

이 브랜치는 **DROK ARM MuJoCo 기반 Meta Quest 3 VR Target Pick-and-Place의 현재 완성본**입니다.
