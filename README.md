# IK_solver_MuJoCo

DROK 6자유도 로봇팔을 대상으로 자체 순기구학(FK), 역기구학(IK), Poly5 궤적 생성, ROS2 제어, MuJoCo 시뮬레이션 및 향후 중력보상을 구현하기 위한 ROS2 Humble 워크스페이스입니다.

현재 기준 저장소:

```text
https://github.com/jhj0129/IK_solver_MuJoCo
```

현재 프로젝트는 최적화나 파일 정리를 하지 않은 개발 스냅샷입니다. `src`뿐 아니라 현재 생성된 `build`, `install`, `log`, 시험 로그와 백업 파일도 그대로 보관합니다.

---

# 실행 구역

이 부분은 컴퓨터를 껐다 켠 뒤 다시 실행할 때 가장 먼저 확인하는 구역입니다.

## 1. 컴퓨터 재부팅 후 MuJoCo 로봇팔 실행

### Terminal 1 — MuJoCo, ros2_control, controller 실행

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch drok_arm_mujoco drok_arm_mujoco.launch.py
```

정상 실행 시 다음 항목이 함께 시작됩니다.

- URDF → MJCF 변환 노드
- MuJoCo viewer
- `robot_state_publisher`
- `controller_manager`
- `joint_state_broadcaster`
- `arm_controller`
- `trajectory_bridge`

MuJoCo viewer에 실제 로봇팔과 그리퍼 메쉬가 표시되고 상단 상태가 `Running`이면 정상입니다.

---

## 2. controller 상태 확인

### Terminal 2

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

정상 기준:

```text
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

command/state interface 확인:

```bash
ros2 control list_hardware_interfaces
```

현재 JOINT1~JOINT6의 command interface는 다음과 같습니다.

```text
JOINT1/position
JOINT2/position
JOINT3/position
JOINT4/position
JOINT5/position
JOINT6/position
```

외부 관절 궤적 명령 토픽:

```text
/arm_controller/joint_trajectory
```

관절 상태 토픽:

```text
/joint_states
```

---

## 3. 현재 관절 상태 한 번 확인

### Terminal 2

```bash
ros2 topic echo /joint_states \
  sensor_msgs/msg/JointState \
  --qos-reliability best_effort \
  --once
```

현재 정상 정지 자세에서 확인된 값:

```text
JOINT1 = -0.0000003676 rad
JOINT2 =  0.2890457138 rad
JOINT3 =  0.2862403585 rad
JOINT4 = -0.0001100327 rad
JOINT5 = -0.0000032149 rad
JOINT6 =  0.0002550573 rad
```

각도로 환산하면 대략:

```text
JOINT1 =  -0.00002 deg
JOINT2 = +16.56110 deg
JOINT3 = +16.40036 deg
JOINT4 =  -0.00630 deg
JOINT5 =  -0.00018 deg
JOINT6 =  +0.01461 deg
```

정지 상태에서 velocity는 거의 0입니다.

확인된 `/joint_states` effort:

```text
JOINT1 =  0.000000 N·m 수준
JOINT2 =  3.107601 N·m
JOINT3 =  3.953129 N·m
JOINT4 =  0.003722 N·m
JOINT5 =  0.000059 N·m
JOINT6 = -0.003902 N·m
```

JOINT2와 JOINT3에 하중이 집중되어 있습니다. 다만 이 effort에는 위치제어기, 감쇠, 마찰 및 접촉 영향이 포함될 수 있으므로 이를 순수 중력토크 `g(q)`로 바로 사용하면 안 됩니다.

---

## 4. JOINT1~JOINT6 개별 구동 방향 시험

MuJoCo launch를 Terminal 1에서 유지한 상태로 실행합니다.

### Terminal 2

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash
source install/setup.bash

python3 tools/joint_direction_test.py
```

시험 프로그램은 각 관절을 작은 각도로 움직인 뒤 원위치로 복귀시킵니다.

확인 완료된 관절 방향:

| 관절 | 양의 명령에 대한 화면상 방향 | 시험 결과 |
|---|---|---|
| JOINT1 | +Yaw | 정상 |
| JOINT2 | +Pitch | 정상 |
| JOINT3 | -Pitch | 정상 |
| JOINT4 | +Roll | 정상 |
| JOINT5 | +Yaw | 정상 |
| JOINT6 | -Roll | 정상 |

JOINT2~JOINT6의 +3도 시험은 모두 정상 판정되었습니다.

시험 결과 파일:

```text
logs/joint_direction_test_20260722_200937.csv
```

시험 도중 비정상 진동, 충돌 또는 다른 관절의 움직임이 보이면 즉시 `Ctrl+C`를 누릅니다.

---

## 5. 전체 워크스페이스 빌드

소스가 변경되었거나 GitHub에서 새로 clone한 경우 실행합니다.

### Terminal 1 또는 별도 빌드 터미널

```bash
cd ~/IK_solver_MuJoCo

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

특정 패키지만 다시 빌드할 때:

```bash
colcon build --symlink-install \
  --packages-select \
  drok_arm_description \
  drok_arm_kinematics \
  drok_arm_trajectory \
  drok_arm_control \
  drok_arm_mujoco
```

빌드 후에는 반드시 다시 source 합니다.

```bash
source ~/IK_solver_MuJoCo/install/setup.bash
```

---

## 6. 순기구학 FK 시험

geometry YAML 경로 설정:

```bash
GEOMETRY=~/IK_solver_MuJoCo/src/drok_arm_kinematics/config/robot_geometry.yaml
```

0 자세 FK:

```bash
ros2 run drok_arm_kinematics test_fk "$GEOMETRY"
```

현재 MuJoCo 시작 자세에 가까운 관절값으로 FK:

```bash
ros2 run drok_arm_kinematics test_fk "$GEOMETRY" \
  -0.0000003676 \
   0.2890457138 \
   0.2862403585 \
  -0.0001100327 \
  -0.0000032149 \
   0.0002550573
```

입력 단위는 radian입니다.

---

## 7. 역기구학 IK 자체 시험

```bash
GEOMETRY=~/IK_solver_MuJoCo/src/drok_arm_kinematics/config/robot_geometry.yaml

ros2 run drok_arm_kinematics test_ik "$GEOMETRY"
```

지정한 Cartesian pose에 대한 IK 계산:

```bash
ros2 run drok_arm_kinematics solve_ik_pose \
  "$GEOMETRY" \
  <x> <y> <z> \
  <roll> <pitch> <yaw>
```

현재 관절각을 초기값으로 함께 입력할 때:

```bash
ros2 run drok_arm_kinematics solve_ik_pose \
  "$GEOMETRY" \
  <x> <y> <z> \
  <roll> <pitch> <yaw> \
  <q1> <q2> <q3> <q4> <q5> <q6>
```

단위:

```text
x, y, z          : meter
roll, pitch, yaw : radian
q1 ~ q6          : radian
```

아직 MuJoCo의 실제 TCP 좌표와 자체 FK 결과의 정밀 비교가 끝나지 않았으므로 검증되지 않은 Cartesian 목표를 바로 로봇팔에 전송하지 않습니다.

---

## 8. Poly5 궤적 시험

```bash
source /opt/ros/humble/setup.bash
source ~/IK_solver_MuJoCo/install/setup.bash

ros2 run drok_arm_trajectory test_poly5
```

현재 Poly5 모듈은 관절 위치, 속도, 가속도가 연결되는 5차 다항식 궤적을 생성합니다.

---

## 9. Cartesian pose → 관절 궤적 노드

실제 MuJoCo controller 명령 노드:

```text
drok_arm_control/pose_joint_trajectory_node
```

기본 command topic:

```text
/arm_controller/joint_trajectory
```

실행 형식:

```bash
GEOMETRY=~/IK_solver_MuJoCo/src/drok_arm_kinematics/config/robot_geometry.yaml

ros2 run drok_arm_control pose_joint_trajectory_node \
  --ros-args \
  -p geometry_yaml:="$GEOMETRY" \
  -p target_x:=<x> \
  -p target_y:=<y> \
  -p target_z:=<z> \
  -p target_roll:=<roll> \
  -p target_pitch:=<pitch> \
  -p target_yaw:=<yaw> \
  -p duration:=3.0 \
  -p point_rate:=100.0 \
  -p use_current_joint_state:=true
```

이 명령은 FK와 MuJoCo TCP 비교가 끝난 뒤 검증된 목표 pose로만 실행합니다.

---

# 현재 구현 상태

## 완료

- DROK 로봇팔 URDF 구성
- ARM_BASE_LINK 및 LINK1~LINK6 메쉬 적용
- 그리퍼 베이스, 좌우 그리퍼 메쉬 적용
- MuJoCo용 URDF 구성
- JOINT1~JOINT6 `ros2_control` 등록
- `joint_trajectory_controller` 활성화
- `/joint_states` 발행
- `/arm_controller/joint_trajectory` 입력 경로 구성
- C++ `trajectory_bridge` 구성
- MuJoCo viewer에서 실제 메쉬 출력
- JOINT1~JOINT6 개별 구동
- 관절 회전 방향 확인
- 자체 FK 라이브러리
- 자체 IK 라이브러리
- Poly5 궤적 라이브러리
- Cartesian pose 기반 관절 궤적 노드 기본 구현

## 현재 바로 다음 작업

1. 자체 FK로 계산한 `gripper_tcp` pose 출력
2. ROS TF의 `ARM_BASE_LINK → gripper_tcp` pose 확인
3. MuJoCo site `gripper_tcp`의 world pose 확인
4. 세 결과의 위치 및 자세 오차 비교
5. FK가 일치하면 IK 결과를 MuJoCo에서 검증
6. 이후 중력보상용 torque/effort command interface 설계

---

# TCP 분석 결과

최종 MJCF에는 다음 요소가 모두 존재합니다.

```text
body: gripper_tcp
site: gripper_tcp
```

MuJoCo site:

```text
name = gripper_tcp
pos  = 0.1394 0 0
quat = 1 0 0 0
```

이 값은 LINK6에서 TCP까지의 고정 변환과 일치합니다.

```text
LINK6 → GRIPPER_BASE   = 0.0800 m
GRIPPER_BASE → center = 0.0594 m
합계                    = 0.1394 m
```

따라서 앞으로 MuJoCo의 실제 TCP ground truth는 `gripper_tcp` site의 world position과 world rotation을 읽는 방식으로 검증합니다.

kinematics 설정:

```text
base_frame = ARM_BASE_LINK
tool_frame = gripper_tcp
movable_joint_count = 6
chain_joint_count = 9
```

파일:

```text
src/drok_arm_kinematics/config/robot_geometry.yaml
```

---

# MuJoCo 메쉬 표시 문제와 수정 사항

## 기존 문제

URDF의 STL을 OBJ/MJCF로 변환한 뒤 최종 geom에 `mesh="..."`는 남아 있었지만 `type="mesh"`가 없었습니다. 이 때문에 MuJoCo viewer에서 링크가 실제 메쉬가 아니라 sphere와 유사한 기본 형상으로 표시되었습니다.

## 수정한 시스템 파일

```text
/opt/ros/humble/local/lib/python3.10/dist-packages/
mujoco_ros2_control/urdf_to_mujoco_utils.py
```

함수:

```text
update_obj_assets()
```

`sub_geom_local = sub_geom.cloneNode(False)` 바로 뒤에 다음 코드를 추가했습니다.

```python
if sub_geom_local.hasAttribute("mesh"):
    sub_geom_local.setAttribute("type", "mesh")
```

최종 검증 결과:

```text
mesh: 20
plane: 1
type 없음: 2
```

ARM_BASE_LINK, LINK1~LINK6, GRIPPER_BASE, GRIPPER_LEFT, GRIPPER_RIGH의 visual/collision geom 20개가 모두 `type="mesh"`로 확인되었습니다.

이 수정은 컴퓨터를 재부팅해도 유지됩니다. 다만 `mujoco_ros2_control`을 재설치하거나 업데이트하면 사라질 수 있습니다.

## 패치 유무 확인

```bash
grep -n -A10 -B3 \
  'sub_geom_local = sub_geom.cloneNode' \
  /opt/ros/humble/local/lib/python3.10/dist-packages/\
mujoco_ros2_control/urdf_to_mujoco_utils.py
```

`type="mesh"` 코드가 없을 때만 아래 패치를 실행합니다.

```bash
sudo python3 - <<'PY'
from pathlib import Path

path = Path(
    "/opt/ros/humble/local/lib/python3.10/dist-packages/"
    "mujoco_ros2_control/urdf_to_mujoco_utils.py"
)

text = path.read_text(encoding="utf-8")

marker = '''                    for sub_geom in sub_geoms:
                        sub_geom_local = sub_geom.cloneNode(False)
'''

replacement = '''                    for sub_geom in sub_geoms:
                        sub_geom_local = sub_geom.cloneNode(False)

                        # MuJoCo mesh geom 보존
                        if sub_geom_local.hasAttribute("mesh"):
                            sub_geom_local.setAttribute("type", "mesh")
'''

if 'sub_geom_local.setAttribute("type", "mesh")' in text:
    print("이미 패치가 적용되어 있습니다.")
elif marker not in text:
    raise RuntimeError("패치 위치를 찾지 못했습니다.")
else:
    backup = Path(str(path) + ".bak_before_mesh_type_fix")
    if not backup.exists():
        backup.write_text(text, encoding="utf-8")
        print("백업 생성:", backup)

    path.write_text(
        text.replace(marker, replacement, 1),
        encoding="utf-8",
    )
    print("패치 완료:", path)
PY
```

패치 후 기존 MuJoCo launch를 완전히 종료하고 다시 실행합니다.

---

# 프로젝트 구성

```text
IK_solver_MuJoCo/
├── src/
│   ├── drok_arm_description/
│   │   ├── meshes/
│   │   ├── urdf/
│   │   ├── launch/
│   │   └── rviz/
│   ├── drok_arm_kinematics/
│   │   ├── include/
│   │   ├── src/
│   │   └── config/robot_geometry.yaml
│   ├── drok_arm_trajectory/
│   │   ├── include/
│   │   └── src/
│   ├── drok_arm_control/
│   │   ├── include/
│   │   └── src/
│   └── drok_arm_mujoco/
│       ├── launch/
│       ├── config/
│       ├── model/
│       └── urdf/
├── tools/
│   └── joint_direction_test.py
├── logs/
│   └── joint_direction_test_20260722_200937.csv
├── build/
├── install/
└── log/
```

패키지 역할:

| 패키지 | 역할 |
|---|---|
| `drok_arm_description` | URDF, STL 메쉬, RViz 모델 |
| `drok_arm_kinematics` | 독립 FK/IK 및 geometry YAML |
| `drok_arm_trajectory` | Poly5 궤적 생성 |
| `drok_arm_control` | pose 기반 이동, joint trajectory 생성, trajectory bridge |
| `drok_arm_mujoco` | MuJoCo, ros2_control, controller 및 launch 구성 |

---

# 주요 설정

## controller

```text
arm_controller
```

형식:

```text
joint_trajectory_controller/JointTrajectoryController
```

제어 관절:

```text
JOINT1
JOINT2
JOINT3
JOINT4
JOINT5
JOINT6
```

현재 command interface:

```text
position
```

현재 controller update rate:

```text
100 Hz
```

## 그리퍼

현재 MuJoCo 기준에서 좌우 그리퍼 관절은 fixed로 처리되어 있습니다.

```text
JOINT7                : fixed
GRIPPER_RIGHT_JOINT   : fixed
```

따라서 현재 제어 대상은 JOINT1~JOINT6입니다.

---

# GitHub 저장 구역

현재 원격 저장소:

```text
https://github.com/jhj0129/IK_solver_MuJoCo
```

기본 브랜치:

```text
main
```

현재 개발 스냅샷 전체를 정리 없이 올리기 위해 `git add -A`를 사용합니다.

## README 적용

다운로드한 README 텍스트의 내용을 다음 파일로 저장합니다.

```text
~/IK_solver_MuJoCo/README.md
```

## 최초 업로드 명령

```bash
cd ~/IK_solver_MuJoCo

git init
git branch -M main

git remote remove origin 2>/dev/null || true
git remote add origin \
  https://github.com/jhj0129/IK_solver_MuJoCo.git

git add -A

git commit -m \
  "Save current DROK arm MuJoCo development state"

git push -u origin main
```

Git 사용자 정보가 설정되지 않았다는 오류가 나오면:

```bash
git config --global user.name "jhj0129"
git config --global user.email "jhj020129@kookmin.ac.kr"
```

그다음 다시 commit과 push를 실행합니다.

```bash
git commit -m \
  "Save current DROK arm MuJoCo development state"

git push -u origin main
```

## 이후 변경사항 저장

```bash
cd ~/IK_solver_MuJoCo

git status

git add -A

git commit -m "Update DROK arm MuJoCo development"

git push
```

---

# 컴퓨터 재부팅 후 작업 재개 순서

## Terminal 1

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch drok_arm_mujoco drok_arm_mujoco.launch.py
```

## Terminal 2

```bash
cd ~/IK_solver_MuJoCo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
ros2 control list_hardware_interfaces
```

## Terminal 3

```bash
ros2 topic echo /joint_states \
  sensor_msgs/msg/JointState \
  --qos-reliability best_effort \
  --once
```

이 세 단계가 정상이라면 기존 개발 상태에서 바로 이어갈 수 있습니다.

---

# 다음 개발 목표

현재 다음 작업은 FK 비교 검증입니다.

```text
현재 JOINT1~JOINT6
        ↓
자체 FK 계산
        ↓
ARM_BASE_LINK → gripper_tcp
        ↓
ROS TF 결과와 비교
        ↓
MuJoCo gripper_tcp site world pose와 비교
```

세 결과가 일치하면 IK 및 Poly5를 이용한 Cartesian 이동 검증으로 넘어갑니다.

최종 목표는 MuJoCo에서 `g(q)`를 검증하고, 실제 로봇팔에 적용할 능동형 중력보상 제어기를 구현하는 것입니다.
