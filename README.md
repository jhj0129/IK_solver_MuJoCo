# DROK ARM Quest 3 VR Pick-and-Place — Final

Meta Quest 3에서 받침대의 목표점을 선택하면 Ubuntu ROS 2가 자체 IK와
관절 궤적을 계산하고 MuJoCo에서 원기둥을 Pick-and-Place 합니다.

## 처음 Clone

```bash
cd ~
git clone --branch feature/random-known-pose-pick \
  https://github.com/jhj0129/IK_solver_MuJoCo.git
cd ~/IK_solver_MuJoCo
bash tools/final_vr/setup_after_clone.sh
```

## 컴퓨터를 껐다 켠 뒤 실행

### Terminal 1 — MuJoCo

```bash
cd ~/IK_solver_MuJoCo
bash tools/final_vr/run_mujoco.sh
```

### Terminal 2 — ROS-TCP Endpoint

```bash
cd ~/IK_solver_MuJoCo
bash tools/final_vr/run_ros_tcp_endpoint.sh
```

### Terminal 3 — VR FAST Pick-and-Place

MuJoCo까지 새로 실행한 직후:

```bash
cd ~/IK_solver_MuJoCo
bash tools/final_vr/run_vr_fast.sh 2.5 fresh
```

같은 MuJoCo 세션에서 실행 노드만 다시 시작:

```bash
bash tools/final_vr/run_vr_fast.sh 2.5 preserve
```

## VR 조작

```text
오른쪽 Ray로 목표 받침대 조준
Trigger: 목표점 선택
A 버튼: 목표 확정
```

## IK

```text
Numerical Jacobian
+ Damped Least-Squares
+ Position-only seed IK
+ Full-pose Pick IK
+ Upright 5-DoF Transfer IK
+ Null-space seed continuity 및 joint-limit barrier
```

FAST 방식은 주요 Cartesian endpoint에서 IK를 계산하고 endpoint 사이를
관절공간 궤적으로 연결합니다.

## 궤적

```text
IK joint waypoints
→ natural cubic spline
→ Poly5 time scaling
→ FollowJointTrajectory
```

## 상태 초기화

```bash
bash tools/final_vr/reset_vr_state.sh
```

현재 FAST 실행은 MuJoCo 시뮬레이션 전용입니다.
Quest에 설치된 Unity 앱은 별도이며 이 ROS 2 저장소에는 포함되지 않습니다.
