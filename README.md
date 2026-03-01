# Pick2Pack_Custom_PhoneCase-Project
m0609를 사용해 Pick2Pack Custom PhoneCase Packagine 프로젝트

# Pick2Pack — 협동로봇 기반 커스텀 폰케이스 Pick-to-Pack (ROS2)

---

## 1) 시스템 설계 / 아키텍처 / 플로우 차트

### 1-1. 시스템 아키텍처(요약)
- **User UI (Flask)**  
  - 사용자 선택(핸드폰 기종, 액세서리) 후 ROS 토픽 발행: `/signal_case`, `/signal_acc`
  - 공정률 토픽 `/signal_stage` 구독 → 진행률 UI 표시
  - 주문을 Firebase RTDB에 저장(POST)

- **Admin UI (Flask)**
  - 주문 목록/통계 조회(Firebase)
  - 로봇 제어 토픽 발행: `/signal_stop`, `/signal_start`, `/signal_unlock`

- **Control PC (ROS2 + Doosan M0609 제어 노드)**
  - Stage 1~5 공정 수행
  - `/signal_case`, `/signal_acc` 수신 후 **주문 Queue**에 적재 → 반복 실행 가능
  - 일시정지/재개/안전복구 처리(`/signal_stop`, `/signal_start`, `/signal_unlock`)
  - 공정 단계마다 `/signal_stage` 발행(진행률 UI 연동)

- **DB (Firebase RTDB)**
  - `/orders`에 주문 저장
  - Admin UI에서 조회/통계

### 1-2. ROS2 토픽 흐름
- UI → 로봇
  - `/signal_case` (std_msgs/Int32): 케이스/폰 기종 값
  - `/signal_acc` (std_msgs/Int32): 액세서리 값(여러 번 발행 가능)
- 관리자 → 로봇
  - `/signal_stop` (std_msgs/Int32): 일시정지/비상정지
  - `/signal_start` (std_msgs/Int32): 재개
  - `/signal_unlock` (std_msgs/Int32): 안전정지 해제(Reset Safe Stop)
- 로봇 → UI
  - `/signal_stage` (std_msgs/Int32): 공정 단계 진행률(예: 2→3→4→5→6)

---

## 2) 운영체제 환경
- OS: **Ubuntu 22.04 LTS**
- ROS2: **Humble**
- Python: **Python3**
- UI: **Flask**
- DB: **Firebase Realtime Database (RTDB)**

## 3) 사용한 장비 목록
- Doosan Robotics **M0609**
- (그리퍼) 프로젝트 사용 그리퍼/툴(TCP) 구성
- PC 2~3대(역할 분리 권장)
  - Control PC: 로봇 제어(ROS2)
  - UI PC: User UI 실행(Flask + ROS2)
  - Admin PC: Admin UI 실행(Flask + ROS2) — UI PC에서 함께 실행해도 됨
- Firebase RTDB

---

## 4) 의존성

### 4. ROS2 패키지 의존성(pick2pack)
`pick2pack/package.xml` 기준 핵심 의존성:
- `rclpy`
- `std_msgs`
- `launch`
- `launch_ros`

> 두산 로봇 제어(서비스/SDK)는 설치 방식이 환경마다 달라 별도 안내가 필요할 수 있습니다.


## 5) 프로젝트 폴더 구성(처음 실행 기준)
아래처럼 폴더를 구성하면 가장 단순합니다.

```
project_root/
  cobot_ws/                     # ROS2 워크스페이스(=Control PC)
    src/
      pick2pack/              # ROS2 패키지
  ui/                         # UI PC 또는 Admin PC
    User_UI.py
    Admin_UI.py
    GameplayMusic.mp3
    serviceAccountKey.json
    requirements.txt
    templates/
      index.html
      admin.html
```

---

#  빠른 실행(Quickstart)

## A. Control PC (로봇 제어: ROS2 pick2pack)

### A-1) ROS2 환경 준비
```bash
source /opt/ros/humble/setup.bash
```

### A-2) 워크스페이스 생성 및 패키지 배치
```bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
# 여기(~/ros_ws/src)에 pick2pack 폴더를 넣습니다.
```

### A-3) 빌드 및 setup source
```bash
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
```

### A-4) 실행(Launch)
```bash
ros2 launch pick2pack integrated_all.launch.py
```

---

## B. UI PC (User UI 실행)

### B-1) 파일 배치
다음 파일이 **같은 폴더(ui/)**에 있어야 합니다.

- `User_UI.py`
- `serviceAccountKey.json`  (Firebase 관리자 키)
- `GameplayMusic.mp3` (사용하는 경우)
- `templates/index.html`
- `requirements.txt`

### B-2) 파이썬 의존성 설치
```bash
cd ~/project_root/ui
python3 -m venv .venv
source .venv/bin/activate
pip install -U pip
pip install -r requirements.txt
```

### B-3) (중요) ROS2 환경 source
```bash
source /opt/ros/humble/setup.bash
```

### B-4) 실행
```bash
python3 User_UI.py
```

---

## C. Admin UI 실행
```bash
cd ~/project_root/ui
source .venv/bin/activate
source /opt/ros/humble/setup.bash
python3 Admin_UI.py
```

---

# 동작 확인(토픽만으로도 테스트 가능)

## 1) 토픽 목록 확인
```bash
ros2 topic list | egrep "signal_case|signal_acc|signal_stage|signal_stop|signal_start|signal_unlock"
```

## 2) 케이스 값 발행(예: 1)
```bash
ros2 topic pub -1 /signal_case std_msgs/msg/Int32 "{{data: 1}}"
```

## 3) 액세서리 값 발행(예: 2,3 여러 번)
```bash
ros2 topic pub -1 /signal_acc std_msgs/msg/Int32 "{{data: 2}}"
ros2 topic pub -1 /signal_acc std_msgs/msg/Int32 "{{data: 3}}"
```

## 4) 공정률(stage) 수신 확인
```bash
ros2 topic echo /signal_stage
```

## 5) 일시정지/재개/안전복구 테스트
```bash
ros2 topic pub -1 /signal_stop std_msgs/msg/Int32 "{{data: 1}}"
ros2 topic pub -1 /signal_start std_msgs/msg/Int32 "{{data: 1}}"
ros2 topic pub -1 /signal_unlock std_msgs/msg/Int32 "{{data: 1}}"
```

---

# 트러블슈팅(가장 많이 막히는 부분)

## 1) UI에서 토픽 발행했는데 Control PC가 못 받음
### (1) ROS_DOMAIN_ID 통일
```bash
export ROS_DOMAIN_ID=0
echo 'export ROS_DOMAIN_ID=0' >> ~/.bashrc
source ~/.bashrc
```

### (2) 방화벽(UFW) 확인
```bash
sudo ufw status
# 테스트 목적으로 끄기(필요 시)
sudo ufw disable
```

### (3) 같은 네트워크(서브넷)인지 확인
```bash
ip a
```

### (4) UI 실행 전에 ROS2 source 했는지 확인
```bash
source /opt/ros/humble/setup.bash
```

---

## 2) Firebase가 안 붙음(주문 저장/조회 안됨)
- `serviceAccountKey.json` 경로/파일명 확인
- Firebase 프로젝트 권한(관리자 키) 확인
- 네트워크(방화벽/프록시)로 외부 HTTPS가 막혀있지 않은지 확인

---

## 라이선스
Apache License 2.0
