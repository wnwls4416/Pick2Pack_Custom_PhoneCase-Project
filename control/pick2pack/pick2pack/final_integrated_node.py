import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from dsr_msgs2.srv import SetRobotControl
import DR_init
import time
import threading
import queue

# ==============================================================================
# [설정] 로봇 및 공정 상수
# ==============================================================================
# 제어할 로봇의 고유 이름(dsr01)과 모델명(m0609)을 지정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
# 로봇 끝단에 장착된 그리퍼(GripperDA_v2)의 무게와 좌표계(TCP)를 설정하여 정밀한 움직임을 보장
ROBOT_TOOL = "Tool Weight2"
ROBOT_TCP = "GripperDA_v2"
# 기본 이동 속도와 가속도
VELOCITY, ACC = 600.00, 800.00
ON, OFF = 1, 0

# 두산 로보틱스의 하드웨어 상태 번호와 제어 명령 번호
# 로봇 상태 상수
STATE_STANDBY = 1    # 정상 대기
STATE_SAFE_OFF = 3   # 서보 꺼짐
STATE_SAFE_STOP = 5  # 안전 정지

# 제어 명령 상수
CONTROL_RESET_SAFE_STOP = 2
CONTROL_RESET_SAFE_OFF = 3

# --- [전역 변수] ---
# 수신된 주문(케이스 종류, 액세서리 리스트)을 순차적으로 처리하기 위한 대기열
order_queue = queue.Queue()
# 제어상태 변수 정의
is_paused = False
needs_unlock = False

# ==============================================================================
# 1. 통신 전담 노드 (TopicListenerNode)
# ==============================================================================
class TopicListenerNode(Node):
    # 외부(키오스크, 센서 등)에서 들어오는 신호를 실시간으로 감시
    def __init__(self):
        super().__init__('topic_listener')
        self.signal_case = 0
        self.acc_storage = []
        self.acc_timer = None
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST, # 구독자 실행 전에 발행된 내용도 저장
            depth=10
        )
        
        self.create_subscription(Int32, '/signal_case', self.case_callback, qos_profile)
        self.create_subscription(Int32, '/signal_acc', self.acc_callback, qos_profile)
        self.create_subscription(Int32, '/signal_stop', self.stop_callback, qos_profile)
        self.create_subscription(Int32, '/signal_start', self.start_callback, qos_profile)
        self.create_subscription(Int32, '/signal_unlock', self.unlock_callback, qos_profile)
        self.get_logger().info(">>> [통신 노드] 가동 완료.")

    # 작업의 일시 정지 및 재개를 제어
    def stop_callback(self, msg):
        global is_paused
        self.get_logger().warn("[Pause] 일시 정지 수신")
        is_paused = True

    def start_callback(self, msg):
        global is_paused
        self.get_logger().info("[Resume] 작업 재개 수신")
        is_paused = False

            # 로봇이 안전 정지 상태일 때, 사용자가 위험 요소가 없음을 확인하고 보내는 "해제 신호"를 처리
    def unlock_callback(self, msg):
        global needs_unlock
        if msg.data == 1:
            self.get_logger().info("[Unlock] 해제 신호 확인!")
            needs_unlock = True

    def case_callback(self, msg):
        self.get_logger().info(f"[Case] 수신: {msg.data}")
        self.signal_case = msg.data
        self.check_dispatch()

    # 액세서리 번호가 여러 번 들어올 때 이를 리스트로 묶음 (5초 or 6개 이상 시 발행)
    def acc_callback(self, msg):
        self.get_logger().info(f"[Acc] 수신: {msg.data}")
        if not self.acc_storage: self.start_timer()
        self.acc_storage.append(msg.data)
        if len(self.acc_storage) >= 6: self.finalize_list()

    # 5초
    def start_timer(self):
        if self.acc_timer: self.acc_timer.cancel()
        self.acc_timer = self.create_timer(5.0, self.finalize_list)

    # 최종 발행 리스트
    def finalize_list(self):
        if self.acc_timer: self.acc_timer.cancel(); self.acc_timer = None
        if self.acc_storage:
            self.get_logger().info(f"[Acc List] 확정: {self.acc_storage}")
            self.check_dispatch()

    # 케이스 종류와 액세서리 리스트가 모두 갖춰지면 order_queue에 작업(Task)을 집어넣어 로봇이 일을 시작
    def check_dispatch(self):
        if self.signal_case != 0 and self.acc_storage:
            task = {'case': self.signal_case, 'list': self.acc_storage[:]}
            order_queue.put(task)
            self.signal_case = 0
            self.acc_storage = []

# ==============================================================================
# 2. 로봇 제어 전담 노드 (RobotWorkerNode)
# ==============================================================================
class RobotWorkerNode(Node):
    def __init__(self):
        super().__init__('integrated_master')
        # 단계가 끝날 때마다 스테이지 정보 발행
        self.pub_stage = self.create_publisher(Int32, '/signal_stage', 10)
        # 제어용 서보 컨트롤
        self.srv_control = self.create_client(SetRobotControl, f'/{ROBOT_ID}/system/set_robot_control')

    # chesk_pause + movel 기능 정의
    def movel2(self, *args, **kwargs):
        self.check_pause()
        from DSR_ROBOT2 import movel
        return movel(*args, **kwargs)

     # chesk_pause + movej 기능 정의
    def movej2(self, *args, **kwargs):
        self.check_pause()
        from DSR_ROBOT2 import movej
        return movej(*args, **kwargs)

    # 단계가 끝날 때마다 스테이지 정보 발행
    def publish_progress(self, val):
        msg = Int32(); msg.data = val
        self.pub_stage.publish(msg)
        self.get_logger().info(f"-> Stage {val} 진입")


    def call_hw_control(self, control_value):
        # 하드웨어 제어 서비스(srv_control)를 호출하여 로봇의 잠금을 풀거나 전원을 다시 넣는 역할
        # 로봇 제어 서비스가 현재 살아있는지 1초간 확인 : 응답이 없으면 실패(False)를 반환
        if not self.srv_control.wait_for_service(timeout_sec=1.0): return False
        # 두산 로봇 하드웨어 제어를 위한 요청 객체를 생성
        req = SetRobotControl.Request()
        #요청 객체에 실제 명령(예: 안전 정지 해제 등)을 담음
        req.robot_control = control_value
        self.srv_control.call_async(req) # 응답을 기다리지 않고 즉시 반환 : 프로그램 정지 방지
        return True # 명령이 일단 성공적으로 전송

    def check_pause(self): # 로봇이 움직이기 직전마다 호출되어, 로봇의 상태가 정상인지 감시하고 정지 시 복구 절차 
        global is_paused, needs_unlock
        from DSR_ROBOT2 import get_robot_state, drl_script_stop, DR_QSTOP_STO

        while rclpy.ok():
            # 현재 로봇의 하드웨어 상태(정상, 정지, 에러 등) 
            state = get_robot_state()
            if state in [STATE_SAFE_STOP, STATE_SAFE_OFF]: #로봇이 안전 정지(충돌 등) 또는 서보 오프(전원 차단) 상태인지 확인
                self.get_logger().error(f"하드웨어 정지 감지 (Code: {state})")
                drl_script_stop(DR_QSTOP_STO) # 하드웨어가 멈추면 실행 중이던 로봇 동작 스크립트(DRL)도 강제로 중단
                
                while not needs_unlock and rclpy.ok(): # 사용자가 위험 요소를 제거하고 외부(UI 등)에서 '해제(Unlock)' 신호를 줄 때까지 무한 대기
                    self.get_logger().warn(">>> 안전 확인 후 [/signal_unlock]을 보내세요.", throttle_duration_sec=3.0)
                    time.sleep(0.5)

                # 정지 원인에 맞는 적절한 복구 명령 코드를 선택
                cmd = CONTROL_RESET_SAFE_STOP if state == STATE_SAFE_STOP else CONTROL_RESET_SAFE_OFF # 안전 정지상태/안전정지가 아닌데 서보가 꺼진상태
                self.get_logger().info(f"복구 명령 전송 중 (Command: {cmd})...")
                self.call_hw_control(cmd)
                
                # 복구상태 확인 변수 생성
                success_recovery = False
                # 복구 명령을 보낸 후, 로봇이 실제로 준비 완료 상태(STATE_STANDBY)가 되는지 최대 10초간(0.5초 × 20번) 확인
                for i in range(20): # 최대 10초 대기 (0.5초 * 20)
                    time.sleep(0.5)
                    current_state = get_robot_state()
                    # 로봇의 현재 상태가 1(대기 상태)인지 확인 : 로봇의 모든 관절에 힘이 들어가고 명령을 받을 준비가 완벽히 끝난 상태
                    if current_state == STATE_STANDBY:
                        success_recovery = True
                        break
                    self.get_logger().info(f"복구 대기 중... 현재 상태: {current_state}")
                
                # 복구에 성공하면 로봇 설정을 다시 초기화하고 루프를 탈출하여 작업을 재개
                if success_recovery:
                    self.get_logger().info("[성공] 하드웨어가 정상 대기 상태로 전환되었습니다.")
                    self.initialize_robot()
                    needs_unlock = False
                    break
                else:
                    self.get_logger().error("복구 실패. 하드웨어나 비상정지 버튼을 수동 점검하세요.")
                    needs_unlock = False
            elif is_paused: # '일시 정지'를 눌렀다면 잠시 대기
                time.sleep(0.5)
            else:
                break

    def initialize_robot(self):
        from DSR_ROBOT2 import set_robot_mode, set_tool, set_tcp, ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS
        try:
            set_robot_mode(ROBOT_MODE_MANUAL) # 툴 설정이나 TCP를 바꾸기 위해 잠시 수동 모드로 전환
            time.sleep(0.5)
            # 미리 정의된 툴 무게(Tool Weight2)와 그리퍼 좌표계(GripperDA_v2)를 로봇에 적용
            set_tool(ROBOT_TOOL)
            set_tcp(ROBOT_TCP)
            set_robot_mode(ROBOT_MODE_AUTONOMOUS) # 다시 코드로 제어할 수 있는 자동 모드로 전환
            time.sleep(1.0)
            self.get_logger().info("로봇 환경설정 완료 (AUTO 모드)")
        except Exception as e:
            self.get_logger().error(f"초기화 에러: {e}")

    # --------------------------------------------------------------------------
    # 모든 동작 라인 앞에 self.check_pause() 추가
    # --------------------------------------------------------------------------
    def run_stage_2(self, case): # Stage 2: 케이스 픽업
        from DSR_ROBOT2 import posx, movej, movel, wait, set_digital_output, DR_MV_MOD_ABS, DR_MV_MOD_REL
        go_home = [0, 0, 90, 0, 90, 0]
        def hard_grasp(): set_digital_output(2, ON); set_digital_output(1, ON); set_digital_output(3, OFF); wait(1.0)
        def release(): set_digital_output(1, OFF); set_digital_output(2, ON); set_digital_output(3, OFF); wait(1.0)
        
        self.publish_progress(2)
        self.check_pause(); release()
        self.movej2(go_home, vel=100.0, acc=80.0)
        
        if case == 1:
            self.movel2(posx([573.34, -191.01, 178.67, 1.98, -180.00, -88.54]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS)
            self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL); hard_grasp()
            self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL); self.movej2(go_home, vel=100.0, acc=80.0)
            self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
            self.movel2(posx([309.36, -416.28, 238.83, 116.52, -179.97, -153.74]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS)
            self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL); release()
        elif case == 2:
            self.movel2(posx([573.34, -143.05, 178.67, 158.53, -180.00, 68.01]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS)
            self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL); hard_grasp()
            self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC,radius=40.0,  ref=0, mod=DR_MV_MOD_REL); self.movej2(go_home, vel=100.0, acc=80.0)
            self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
            self.movel2(posx([309.36, -416.28, 238.83, 116.52, -179.97, -153.74]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS)
            self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL); release()
        elif case == 3:
            self.movel2(posx([573.34, -92.34, 178.67, 168.06, -180.00, 77.54]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS)
            self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL); hard_grasp()
            self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC,radius=40.0,  ref=0, mod=DR_MV_MOD_REL); self.movej2(go_home, vel=100.0, acc=80.0)
            self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
            self.movel2(posx([309.36, -416.28, 238.83, 116.52, -179.97, -153.74]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS)
            self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, ref=0, mod=DR_MV_MOD_REL); release()
        
        self.movel2(posx([0.0, 50.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY,acc=ACC,ref=0,mod=DR_MV_MOD_REL)
        self.movel2(posx([0.0, 0.0, 180.0, 0.0, 0.0, 0.0]), vel=VELOCITY,acc=ACC,ref=0,mod=DR_MV_MOD_REL)
        self.movej2(go_home, vel=100.0, acc=80.0)

    def run_stage_3(self): #Stage 3: 비즈 작업 공정
        from DSR_ROBOT2 import posx, movej, movel, wait, set_digital_output, set_ref_coord, task_compliance_ctrl, set_stiffnessx, set_desired_force, get_tool_force, release_force, release_compliance_ctrl, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_FC_MOD_ABS, DR_MV_RA_DUPLICATE
        go_home = [0, 0, 90, 0, 90, 0]
        def hard_grasp(): set_digital_output(2, ON); set_digital_output(1, ON); set_digital_output(3, OFF); wait(1.0)
        def release(): set_digital_output(1, OFF); set_digital_output(2, ON); set_digital_output(3, OFF); wait(1.0)
        def hard_release() : set_digital_output(1,OFF); set_digital_output(2,OFF); set_digital_output(3,ON); wait(1.5)
        
        self.publish_progress(3)
        self.check_pause(); release(); 
        self.movel2(posx([210.28, 242.00, 147.84, 152.87, 180.00, 62.60]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); hard_grasp()
        self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
        self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
        self.movel2(posx([329.92, -442.16, 208.80, 142.97, 179.99, 146.24]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
        self.movel2(posx([-50.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 0.0, 180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        movej(go_home, vel=100.0, acc=80.0)
        
        self.check_pause(); release(); 
        self.movel2(posx([430.36, 135.05, 164.79, 87.97, 180.00, 86.74]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); hard_grasp(); wait(0.5)
        self.movel2(posx([0.0, 0.0, 70.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, -80.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([430.36, 55.05, 134.79, 90.00, -140.37, 88.77]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 90.0, 0.0, 0.0, 0.0, 0.0]), vel=[90.0, 67.43], acc=[1000.0, 269.7], ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([430.36, 145.05, 134.79, 90.00, -149.32, 88.77]), vel=[90.0, 67.43], acc=[1000.0, 269.7], ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, -90.0, 0.0, 0.0, 0.0, 0.0]), vel=[90.0, 67.43], acc=[1000.0, 269.7], ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([430.37, 55.05, 134.80, 11.3, 180.0, 10.07]), vel=[90.0, 67.43], acc=[1000.0, 269.7], ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([430.36, -155.33, 134.79, 56.42, -180.0, 55.19]), vel=[90.0, 67.43], acc=[1000.0, 269.7], ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([411.65, -20.73, 33.86, 128.5, -180.0, -138.01]), vel=[90.0, 67.43], acc=[300.0, 169.7], ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([411.65, -18.94, 33.86, 90.0, -125.26, -176.51]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([411.65, -20.73, 33.86, 128.5, -180.0, -138.01]), vel=[150.0, 72.38], acc=[300.0, 189.5], ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([430.36, -155.33, 134.79, 56.42, -180.0, 55.19]), vel=[150.0, 72.38], acc=[1000.0, 289.5], ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([430.36, 135.05, 164.79, 87.97, 180.0, 86.74]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 0.0, -95.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
        self.movel2(posx([0.0, 0.0, 95.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([666.01, 12.55, 164.79, 167.88, -180.0, 166.65]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE); hard_grasp()
        self.movel2(posx([666.01, 12.55, 50.45, 14.71, 180.0, 13.48]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([-110.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=[50.0, 64.13], acc=[1000.0, 256.5], radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        
        self.check_pause(); set_ref_coord(1); task_compliance_ctrl(); set_stiffnessx([3000.0, 3000.0, 100.0, 200.0, 200.0, 200.0], time=0.0)
        set_desired_force([0.0, 0.0, 20.0, 0.0, 0.0, 0.0], [0,0,1,0,0,0], time=0.0, mod=DR_FC_MOD_ABS)
        
        start_time = time.time()
        while rclpy.ok():
            self.check_pause() # 힘 제어 루프 내부에도 일시정지 확인
            force = get_tool_force()
            if 15.0 <= force[2] <= 30.0: break
            if time.time() - start_time > 5.0: break
            wait(0.05)
            
        release_force(time=0.0); release_compliance_ctrl()
        self.movel2(posx([569.36, 12.55, 98.09, 34.41, -180.0, 33.18]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE); hard_release()
        self.movel2(posx([0.0, 0.0, -97.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); hard_grasp()
        self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
        self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([301.36, -379.02, 180.72, 131.41, 179.82, 130.49]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); hard_release()
        self.movel2(posx([0.0, 0.0, 180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0); release()

    def run_stage_4(self, items): #Stage 4: 액세서리 작업 공정
        from DSR_ROBOT2 import posx, movej, movel, wait, set_digital_output, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
        go_home = [0, 0, 90, 0, 90, 0]
        def hard_grasp(): set_digital_output(2, ON); set_digital_output(1, ON); set_digital_output(3, OFF); wait(1.5)
        def grasp(): set_digital_output(2, OFF); set_digital_output(1, ON); set_digital_output(3, OFF); wait(1.0)
        def release(): set_digital_output(1, OFF); set_digital_output(2, ON); set_digital_output(3, OFF); wait(1.0)
        self.publish_progress(4)
        for item in items:
            self.check_pause() # 루프마다 체크
            self.get_logger().info(f"액세서리 {item} 처리 중")
            if item == 2: # masking_tape1
                self.check_pause(); release(); movel(posx([286.03, 242.00, 114.50, 69.98, 180.00, -20.29]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); grasp()
                self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
                self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
                self.movel2(posx([226.15, -451.20, 175.22, 115.68, -179.99, 123.34]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
                self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
            elif item == 3: # masking_tape2
                self.check_pause(); release(); movel(posx([360.44, 242.00, 114.50, 169.92, -180.0, 79.65]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); grasp()
                self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
                self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
                self.movel2(posx([226.15, -451.20, 185.22, 115.68, -179.99, 123.34]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -150.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
                self.movel2(posx([0.0, 0.0, 170.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
            elif item == 4: # grip_tok
                self.check_pause(); release(); movel(posx([505.18, 242.00, 130.0, 22.94, 180.0, -67.33]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); grasp()
                self.movel2(posx([0.0, 0.0, 100.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
                self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
                self.movel2(posx([256.78, -378.10, 212.03, 128.71, 179.96, 127.85]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
                self.movel2(posx([-40.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movel(posx([0.0, 0.0, 180.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
            elif item == 5: # strap
                self.check_pause(); release(); movel(posx([578.67, 242.00, 170.0, 110.62, 180.0, 20.35]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -50.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); hard_grasp()
                self.movel2(posx([0.0, 0.0, 80.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
                self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
                self.movel2(posx([352.3, -380.85, 242.07, 135.16, -179.98, 133.92]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -110.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
                self.movel2(posx([-50.0, 0.0, 0.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movel(posx([0.0, 0.0, 110.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
            elif item == 6: # wallet
                self.check_pause(); release(); movel(posx([634.93, 175.45, 386.29, 46.46, 97.28, 91.69]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([634.93, 175.45, 146.29, 46.46, 97.28, 91.69]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE); hard_grasp()
                self.movel2(posx([0.0, 0.0, 240.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)
                self.movel2(posx([0.0, 0.0, 200.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL)
                self.movel2(posx([308.79, -386.39, 183.14, 143.96, -179.98, -126.52]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
                self.movel2(posx([0.0, 0.0, -90.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=0.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); release()
                self.movel2(posx([0.0, 0.0, 90.0, 0.0, 0.0, 0.0]), vel=VELOCITY, acc=ACC, radius=40.0, ref=0, mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE); movej(go_home, vel=100.0, acc=80.0)

    def run_stage_5(self): #Stage 5: 패키징 작업 공정
        from DSR_ROBOT2 import movej, wait, posj, posx, movel, set_digital_output, DR_MV_MOD_ABS, DR_MV_MOD_REL, DR_MV_RA_DUPLICATE
        go_home = [0, 0, 90, 0, 90, 0]
        def release(): set_digital_output(1, OFF); set_digital_output(2, ON); set_digital_output(3, OFF); wait(1.0)
        def release_greeting(): set_digital_output(1, OFF); set_digital_output(2, ON); set_digital_output(3, OFF)
        def grasp_greeting(): set_digital_output(1, ON); set_digital_output(2, OFF); set_digital_output(3, OFF)

        self.publish_progress(5)
        # box_close()
        self.check_pause(); release()
        self.movel2(posx([280.00,-115.00,40.00,89.27,179.98,88.89]),vel=150.00, acc=200.00,ref=0,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movej2(posj([21.97,5.12,127.89,-41.7,72.13,44.35]), vel=150.00, acc=200.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.00,-120.00, 0.00,0.00,0.00,0.00]), vel=150.00, acc=200.00,ref=0,mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movej2(posj([-48.87, 9.41, 59.80, -26.86, 78.36, -51.18]), vel=800.00, acc=800.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movej2(go_home, vel=150.00, acc=200.00)
        self.movel2(posx([280.00,-115.00,40.00,89.27,179.98,88.89]),vel=150.00, acc=200.00,ref=0,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movej2(posj([21.97,5.12,127.89,-41.7,72.13,44.35]), vel=150.00, acc=200.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.movel2(posx([0.00, -350.00, 0.00, 0.00, 0.00, 0.00]), vel=300.00, acc=300.00, ref=0,mod=DR_MV_MOD_REL, ra=DR_MV_RA_DUPLICATE)
        self.movej2(go_home, vel=150.00, acc=200.00)
        # bowing()
        for _ in range(2):
            self.movej2(posj([-70.00, -10.00, 70.00, 0.00, 30.00, 0.00]), vel=100.00, acc=100.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            self.check_pause(); release_greeting()
            self.movej2(posj([-70.00, 10.00, 100.00, 0.00, 100.00, 0.00]), vel=100.00, acc=100.00,mod=DR_MV_MOD_ABS,  ra=DR_MV_RA_DUPLICATE)
            self.check_pause(); grasp_greeting()
        self.movej2(posj([-70.00, -10.00, 70.00, 0.00, 30.00, 0.00]), vel=100.00, acc=100.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
        self.check_pause(); release_greeting(); movej(go_home, vel=150.00, acc=200.00)
        # swing()
        for _ in range(2):
            self.movej2(posj([20.00, -15.00, 30.00, 0.00, 30.00, 0.00]),vel=100.00, acc=100.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            self.check_pause(); release_greeting()
            self.movej2(posj([20.00, 10.00, 50.00, 0.00, 40.00, 0.00]),vel=100.00, acc=100.00,mod=DR_MV_MOD_ABS, ra=DR_MV_RA_DUPLICATE)
            self.check_pause(); grasp_greeting()
        self.movej2(go_home, vel=150.00, acc=200.00)
        self.publish_progress(6)

    # --- [로봇 작업 메인 루프] ---
    def worker_loop(self): # 로봇이 작업을 감시하고 수행하는 무한 루프 함수를 정의
        while rclpy.ok(): # ROS2 시스템이 정상적으로 작동하는 동안 이 루프를 계속 반복
            try:
                task = order_queue.get(timeout=0.1)  # 주문 대기열(큐)에서 작업을 하나 가져오려고 시도 # 없으면 에러(queue.Empty)를 발생
                
                self.get_logger().info(f"[로봇 시작] Case {task['case']} 처리 시작")
                self.initialize_robot() # 로봇의 모드를 자동(AUTO)으로 바꾸고 툴/TCP 설정을 초기화하여 동작 준비
                
                try:
                    self.run_stage_2(task['case'])
                    self.run_stage_3()
                    self.run_stage_4(task['list'])
                    self.run_stage_5()
                    self.get_logger().info("[로봇 완료] 모든 공정 종료. 다음 명령 대기.")
                except Exception as e:
                    self.get_logger().error(f"[로봇 에러] 동작 중 문제 발생: {e}")
                
                order_queue.task_done() # 져온 작업이 공식적으로 완료
                
            except queue.Empty: pass # 큐가 비어있을 때 발생하는 타임아웃을 무시하고 계속 루프
            except Exception as e: self.get_logger().error(f"워커 루프 에러: {e}")

# ==============================================================================
# 3. 메인 실행부
# ==============================================================================
def main(args=None): # 프로그램의 진입점(Entry Point) 함수를 정의
    rclpy.init(args=args) # ROS2 파이썬 통신 시스템을 초기화
    
    listener_node = TopicListenerNode() # 외부 신호(주문, 정지 신호 등)를 듣는 노드 객체를 생성
    worker_node = RobotWorkerNode() # 로봇에게 명령을 내리고 움직이는 노드 객체를 생성
    
    # 두산 로보틱스 라이브러리(DRL)가 어떤 로봇을 제어하고 어떤 노드를 통해 통신할지 내부 변수에 할당
    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    DR_init.__dsr__node = worker_node
    
    # 최대 4개의 스레드를 사용하는 실행기(Executor)를 생성 
    # '신호 수신'과 '로봇 동작'이 서로 방해하지 않고 병렬로 작동
    # 스레드 분리: MultiThreadedExecutor를 쓰지 않으면 로봇이 Stage 2 동작을 하는 동안(약 몇 초간) 외부에서 오는 '비상 정지' 신호를 처리할 수 없음
    executor = MultiThreadedExecutor(num_threads=4)
    # 통신 전담 노드를 실행기에 등록
    executor.add_node(listener_node)
    
    # 실행기가 통신 신호를 감시하는 작업(spin)을 별도의 백그라운드 스레드에서 돌리도록 설정
    # spin_thread를 daemon=True로 설정하여, 메인 프로그램이 종료될 때 통신 스레드도 즉시 같이 종료되도록 안전 장치를 마련
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    # 백그라운드 통신 감시를 시작 : 동작 중에도 신호를 받을 수 있음
    spin_thread.start()
    
    try:
        worker_node.worker_loop()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        listener_node.destroy_node()
        worker_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()