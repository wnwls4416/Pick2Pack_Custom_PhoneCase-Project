#!/usr/bin/env python3

# ROS 2 통신 레이어
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# 웹 서버 레이어
from flask import Flask, render_template, request, jsonify

# 유틸리티 레이어
import time, threading, datetime

# 데이터베이스 레이어
import firebase_admin
from firebase_admin import credentials, db

# 오디오 레이어
import pygame  # 👈 [추가] 음악 재생 라이브러리

# Flask 앱 인스턴스
app = Flask(__name__)

# [설정] Firebase
FIREBASE_KEY_PATH = '/home/rokey/cobot_ws/src/serviceAccountKey.json'
FIREBASE_DB_URL =  'https://pick2pack-f8d20-default-rtdb.firebaseio.com'

# [설정] 배경음악 경로
MUSIC_PATH = '/home/rokey/cobot_ws/src/GameplayMusic.mp3'

# 전역 변수(ROS2 통신 객체)
ros_node = None
pub_case = None; pub_acc = None; sub_stage = None
current_progress = 0; current_message = "대기 중..."        # 진행률 + 상태 메시지

# 가격표 (저장용)
PRICES = {
    'iPhone 14 Pro': 15000, 'iPhone 16': 15000, 'Galaxy S23': 15000,
    '비즈': 5000, '마스킹테이프1': 4000, '마스킹 테이프1': 4000,
    '마스킹테이프2': 4000, '마스킹 테이프2': 4000,
    '그립톡': 10000, '스트랩': 14000, '카드지갑': 18000
}

# ---------------------------------------------------------
# [음악] 배경음악 재생 함수
# ---------------------------------------------------------
def play_background_music():
    try:
        pygame.mixer.init() # 오디오 드라이버 초기화
        pygame.mixer.music.load(MUSIC_PATH) # 파일 로드
        pygame.mixer.music.set_volume(0.5)  # 볼륨 설정 (0.0 ~ 1.0)
        pygame.mixer.music.play(-1) # -1: 무한 반복 재생
        print(f" [Music] 🎵 배경음악 재생 시작: {MUSIC_PATH}")
    except Exception as e:
        print(f" [Error] 음악 재생 실패: {e}")

# ---------------------------------------------------------
# [Firebase] 주문 상세 내역 저장 (관리자 화면용 데이터 생성)
# ---------------------------------------------------------
def init_firebase():
    if not firebase_admin._apps:
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_DB_URL})


# 주문 로그 저장
def save_order_log(phone, accessories, total_cost):
    """관리자가 보기 편하게 문자열로 변환해서 저장"""
    try:
        # 1. 폰 정보 포맷팅
        p_price = PRICES.get(phone, 15000)
        phone_str = f"{phone} ({p_price:,}원)"

        # 2. 악세서리 정보 포맷팅
        acc_list_str = []
        for acc in accessories:
            clean_acc = acc.replace(" ", "")
            # 띄어쓰기 유무로 가격 찾기
            price = PRICES.get(acc) or PRICES.get(clean_acc) or 0
            acc_list_str.append(f"{acc}({price:,}원)")
        
        acc_str = ", ".join(acc_list_str) if acc_list_str else "선택 없음"

        # 3. DB 저장
        db.reference('orders').push({
            'timestamp': datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            'phone_detail': phone_str,
            'acc_detail': acc_str,
            'total_cost': total_cost
        })
        print(" [Firebase] 주문 내역 저장 완료")
    except Exception as e:
        print(f" [오류] Firebase 저장 실패: {e}")

# ---------------------------------------------------------
# [ROS 2] 로직
# ---------------------------------------------------------

# 로봇에서 오는 신호를 UI 상태로 변환
def stage_callback(msg):
    global current_progress, current_message
    val = msg.data
    if val == 3: current_progress, current_message = 10, "케이스 완료"
    elif val == 4: current_progress, current_message = 35, "비즈 옮기기 완료"
    elif val == 5: current_progress, current_message = 75, "악세서리 옮기기 완료"
    elif val == 6: current_progress, current_message = 100, "패키징 완료"


# Flask 서버와 병렬 동작
def start_ros_node():
    global ros_node, pub_case, pub_acc, sub_stage
    if not rclpy.ok(): rclpy.init()     # ROS2 runtime 초기화

    # Publisher 생성(UI -> 로봇)
    ros_node = rclpy.create_node('user_ui_node') 
    pub_case = ros_node.create_publisher(Int32, '/signal_case', 10)
    pub_acc = ros_node.create_publisher(Int32, '/signal_acc', 10)

    #Subscriber 생성 (로봇 -> UI)
    sub_stage = ros_node.create_subscription(Int32, '/signal_stage', stage_callback, 10)
    print(" [User UI] ROS 2 노드 시작 (Port 5000)")
    rclpy.spin(ros_node)

def publish_sequence(phone_code, acc_list):
    global pub_case, pub_acc
    if pub_case is None: return
    # 폰 전송
    msg = Int32(); msg.data = phone_code
    pub_case.publish(msg)
    time.sleep(1.0)
    # 악세서리 전송
    acc_map = {'비즈':1, '마스킹테이프1':2, '마스킹 테이프1':2, '마스킹테이프2':3, '마스킹 테이프2':3, '그립톡':4, '스트랩':5, '카드지갑':6}
    for item in acc_list:
        clean = item.replace(" ", "")
        code = acc_map.get(clean) or acc_map.get(item)
        if code:
            msg.data = code
            if pub_acc: pub_acc.publish(msg)
            time.sleep(0.5)

# ---------------------------------------------------------
# [Flask] 라우팅
# ---------------------------------------------------------

# 홈페이지
@app.route('/')
def home():
    return render_template('index.html')

# 주문 처리 엔드 포인트
@app.route('/order', methods=['POST'])
def order_robot():
    global current_progress, current_message
    current_progress, current_message = 0, "작업 대기 중..."
    
    data = request.json
    phone = data.get('phone')
    acc = data.get('accessories', [])
    total = data.get('total_cost', 0)

    # 1. DB 저장 (별도 스레드)
    threading.Thread(target=save_order_log, args=(phone, acc, total)).start()

    # 2. 로봇 명령 생성
    code = 0
    if phone == 'iPhone 14 Pro': code = 1
    elif phone == 'iPhone 16': code = 2
    elif phone == 'Galaxy S23': code = 3
    
    threading.Thread(target=publish_sequence, args=(code, acc)).start()
    return jsonify({"status": "success", "data": code})

# 상태 조회 엔드포인트
@app.route('/status', methods=['GET'])
def get_status():
    return jsonify({"progress": current_progress, "message": current_message})

# 메인 실행 블록
if __name__ == '__main__':
    try:
        init_firebase()
        
        # 👇 [추가] 앱 시작 시 음악 재생
        play_background_music()

        # ROS 노드 시작(daemon thread)
        threading.Thread(target=start_ros_node, daemon=True).start()
        
        # Flask 서버 시작(main thread)
        app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)
    except KeyboardInterrupt: pass
    finally:
        if ros_node: ros_node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        try: pygame.mixer.quit() # 종료 시 음악 리소스 해제
        except: pass