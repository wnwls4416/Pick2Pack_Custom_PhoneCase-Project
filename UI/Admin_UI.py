#!/usr/bin/env python3

# 웹 서버 레이어
from flask import Flask, render_template, jsonify, request
import os
import sys

# 데이터베이스 레이어
import firebase_admin
from firebase_admin import credentials, db

# 데이터 분석 레이어
from collections import Counter

# 멀티스레딩 레이어
import threading

# ROS 2 통신 레이어
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

# Flask 앱 인스턴스
app = Flask(__name__)

# [설정] 경로를 현재 파일 위치 기준으로 자동 설정 (파일 경로 에러 방지)
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
FIREBASE_KEY_PATH = os.path.join(CURRENT_DIR, 'serviceAccountKey.json')

FIREBASE_DB_URL = 'https://pick2pack-f8d20-default-rtdb.firebaseio.com'
# [수정됨] URL 오타 수정 (pick12pack -> pick2pack)
FIREBASE_CONSOLE_URL = "https://console.firebase.google.com/project/pick2pack-f8d20/database/pick2pack-f8d20-default-rtdb/data"

# ROS2 노드 클래스
class AdminRosNode(Node):
    def __init__(self):
        super().__init__('admin_ui_node')

        # Publisher 생성 (관리자 -> 로봇)
        self.stop_signal_pub = self.create_publisher(Int32, '/signal_stop', 10)
        self.start_signal_pub = self.create_publisher(Int32, '/signal_start', 10)
        self.stage_pub = self.create_publisher(Int32, '/signal_stage', 10)
        self.unlock_signal_pub = self.create_publisher(Int32, '/signal_unlock', 10)
        self.get_logger().info('✅ [Admin] ROS2 Admin Node Started')

    def send_stop_only(self):
        """긴급 정지 신호 전송"""
        msg = Int32()
        msg.data = 1
        self.stop_signal_pub.publish(msg)
        self.get_logger().warn('🚨 [Admin] EMERGENCY STOP SIGNAL SENT')

    def send_resume_only(self):
        """작업 재개 및 스테이지 초기화"""
        msg_start = Int32()
        msg_start.data = 1
        self.start_signal_pub.publish(msg_start)
        
        msg_stage = Int32()
        msg_stage.data = 0
        self.stage_pub.publish(msg_stage)
        self.get_logger().info('🔄 [Admin] RESUME SIGNAL SENT')

    def send_unlock_only(self):
        """안전 잠금 해제"""
        msg = Int32()
        msg.data = 1
        self.unlock_signal_pub.publish(msg)
        self.get_logger().info('🔓 [Admin] SAFETY UNLOCK SIGNAL SENT')

ros_node = None

def init_firebase():
    """Firebase 초기화 (키 파일 확인 포함)"""
    if not os.path.exists(FIREBASE_KEY_PATH):
        print(f"❌ [Error] Firebase 키 파일을 찾을 수 없습니다: {FIREBASE_KEY_PATH}")
        sys.exit(1) # 파일 없으면 강제 종료

    if not firebase_admin._apps:
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_DB_URL})
        print("🔥 [Firebase] Connected Successfully")

# --- Flask 라우트 ---

@app.route('/')
def admin_page():
    # 여기서 db_url을 html로 넘겨줍니다.
    return render_template('admin.html', db_url=FIREBASE_CONSOLE_URL)

@app.route('/api/orders')
def get_orders():
    try:
        ref = db.reference('orders')
        # 데이터가 많을 경우를 대비해 최근 50개만 가져오기
        snapshot = ref.order_by_key().limit_to_last(50).get()
        data_list = []
        if snapshot:
            for key, val in snapshot.items():
                data_list.append(val)
            data_list.reverse() # 최신순 정렬
        return jsonify({"status": "success", "data": data_list})
    except Exception as e:
        print(f"❌ [API Error] /api/orders: {e}")
        return jsonify({"status": "error", "message": str(e)})

@app.route('/api/statistics')
def get_statistics():
    try:
        ref = db.reference('orders')
        snapshot = ref.get()
        
        if not snapshot: 
            return jsonify({"status": "success", "phones": {}, "accessories": {}})

        phone_counter = Counter()
        acc_counter = Counter()
        
        for key, val in snapshot.items():
            # 데이터 방어 로직: .get() 사용
            raw_phone = val.get('phone_detail', 'Unknown')
            if isinstance(raw_phone, str):
                phone_name = raw_phone.split('(')[0].strip()
                phone_counter[phone_name] += 1
            
            raw_acc = val.get('acc_detail', '')
            if raw_acc and raw_acc != "선택 없음" and isinstance(raw_acc, str):
                items = raw_acc.split(', ')
                for item in items:
                    clean_name = item.split('(')[0].strip()
                    acc_counter[clean_name] += 1
                    
        return jsonify({"status": "success", "phones": dict(phone_counter), "accessories": dict(acc_counter)})
    except Exception as e:
        print(f"❌ [API Error] /api/statistics: {e}")
        return jsonify({"status": "error", "message": str(e)})

# --- 제어 API ---

@app.route('/api/control/emergency', methods=['POST'])
def control_emergency():
    if ros_node:
        ros_node.send_stop_only()
        return jsonify({"status": "success"})
    return jsonify({"status": "error", "message": "ROS Node not initialized"})

@app.route('/api/control/resume', methods=['POST'])
def control_resume():
    if ros_node:
        ros_node.send_resume_only()
        return jsonify({"status": "success"})
    return jsonify({"status": "error", "message": "ROS Node not initialized"})

@app.route('/api/control/unlock', methods=['POST'])
def control_unlock():
    if ros_node:
        ros_node.send_unlock_only()
        return jsonify({"status": "success"})
    return jsonify({"status": "error", "message": "ROS Node not initialized"})

# ROS 스레드 함수
def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"⚠️ ROS Spin Error: {e}")

if __name__ == '__main__':
    try:
        # 1. ROS 초기화
        rclpy.init(args=None)
        ros_node = AdminRosNode()

        # 2. ROS 스레드 실행
        t = threading.Thread(target=ros_spin_thread, args=(ros_node,))
        t.daemon = True
        t.start()

        # 3. Firebase 초기화
        init_firebase()

        # 4. Flask 서버 실행 (Blocking)
        print("🚀 [Server] Starting Flask on port 5001...")
        app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)

    except KeyboardInterrupt:
        print("\n🛑 Shutting down...")
    finally:
        # 종료 시 리소스 정리
        if ros_node:
            ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()