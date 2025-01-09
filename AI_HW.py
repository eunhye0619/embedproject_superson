import socket
import serial
from cvzone.HandTrackingModule import HandDetector
import cv2
import math
import time
import threading

# 카메라 설정
cap = cv2.VideoCapture(0)
cap.set(3, 1280)  # 가로 해상도
cap.set(4, 720)   # 세로 해상도
# 손 검출 설정
detector = HandDetector(detectionCon=0.9, maxHands=1)

# 손가락 관절 정의
finger_joints = {
    "A": [3],          # 엄지
    "B": [5, 6],       # 검지1
    "C": [6, 7],       # 검지2
    "D": [9, 10],      # 중지1
    "E": [10, 11],     # 중지2
    "F": [13, 14],     # 약지1
    "G": [14, 15],     # 약지2
    "H": [17, 18],     # 새끼손가락1
    "I": [18, 19],     # 새끼손가락2
    "J": [0],          # 손목
}

# 각도 계산 함수
def calculate_servo_angles(lmList):
    def angle_between_points(p1, p2, p3):
        try:
            # 벡터 계산
            v1 = (p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2])
            v2 = (p3[0] - p2[0], p3[1] - p2[1], p3[2] - p2[2])
            dot_product = sum(v1[i] * v2[i] for i in range(3))
            mag_v1 = math.sqrt(sum(v1[i] ** 2 for i in range(3)))
            mag_v2 = math.sqrt(sum(v2[i] ** 2 for i in range(3)))
            return math.degrees(math.acos(dot_product / (mag_v1 * mag_v2 + 1e-6)))
        except Exception as e:
            print(f"Error calculating angle: {e}")
            return 0

    servo_angles = []

    # 각도 계산
    for finger, joints in finger_joints.items():
        if len(joints) == 1:  # 손목처럼 단일 관절
            angle = angle_between_points(lmList[0], lmList[joints[0]], lmList[5])
            servo_angles.append(max(0, min(180, int(angle))))
        elif len(joints) == 2:  # 두 개의 관절로 이루어진 손가락
            angle1 = angle_between_points(lmList[joints[0] - 1], lmList[joints[0]], lmList[joints[1]])
            angle2 = angle_between_points(lmList[joints[0]], lmList[joints[1]], lmList[joints[1] + 1])
            servo_angles.append(max(0, min(180, int(angle1))))
            servo_angles.append(max(0, min(180, int(angle2))))

    return servo_angles

# UDP 설정
UDP_IP = "10.150.150.170"  # STM32 보드의 IP 주소 (변경 필요)
UDP_PORT = 5052           # STM32에서 수신할 포트 (변경 필요)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# UART 설정
UART_PORT = "/dev/ttyACM0"  # UART 포트
UART_BAUDRATE = 115200
ser = serial.Serial(UART_PORT, UART_BAUDRATE, timeout=1)

# 손가락 각도 출력 및 전송 함수
def send_servo_angles(servo_angles):
    """
    각 손가락의 각도를 출력하고 UDP 및 UART로 전송
    """
    finger_labels = list(finger_joints.keys())
    for label, angle in zip(finger_labels, servo_angles):
        
        message = f"{label}{str(angle).zfill(3)}"  # 예: A090
        print(message)

        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

        if ser.is_open:
            ser.write((message + "\n").encode())
        time.sleep(0.5)

# UART 데이터 수신 쓰레드 함수
def receive_uart_data():
    """
    UART로 STM32에서 수신한 데이터를 터미널에 출력합니다.
    """
    while True:
        if ser.is_open and ser.in_waiting > 0:
            try:
                # UART로 들어온 데이터 읽기
                received_data = ser.readline().decode('utf-8').strip()
                if received_data:
                    print(f"STM32: {received_data}")
            except Exception as e:
                print(f"UART: {e}")

# UART 수신 쓰레드 시작
uart_thread = threading.Thread(target=receive_uart_data, daemon=True)
uart_thread.start()

# 메인 루프
last_send_time = time.time()  # 마지막 전송 시간 기록

while True:
    success, img = cap.read()

    if not success:
        print("카메라에서 이미지를 가져올 수 없습니다.")
        break

    # 화면 크기 조정
    img = cv2.resize(img, (0, 0), fx=0.5, fy=0.5)

    hands, img = detector.findHands(img)

    if hands:
        hand = hands[0]
        lmList = hand["lmList"]

        if len(lmList) >= 21:
            # 각도 계산
            servo_angles = calculate_servo_angles(lmList)

            # 손가락 관절 표시
            for finger, joints in finger_joints.items():
                for joint in joints:
                    cv2.circle(img, (lmList[joint][0], lmList[joint][1]), 4, (0, 255, 0), cv2.FILLED)

            # 7초마다 각도 전송
            current_time = time.time()
            if current_time - last_send_time >= 7:
                threading.Thread(target=send_servo_angles, args=(servo_angles,)).start()
                last_send_time = current_time

    # 화면 표시
    cv2.imshow("Hand Tracking screen", img)

    # ESC 키로 종료
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()

# 소켓 및 UART 닫기
sock.close()
if ser.is_open:
    ser.close()
