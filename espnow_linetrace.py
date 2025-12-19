import cv2
import numpy as np
import serial  # 추가
import time    # 추가

# ---------------- 1. 전처리 ----------------
def preprocess(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5,5), 0)
    edges = cv2.Canny(blur, 50, 150)
    return edges

# ---------------- 2. 관심 영역(사다리꼴) ----------------
def region_of_interest(img):
    h, w = img.shape
    trap = np.array([[(0,h), (w*0.45, h*0.6), (w*0.55, h*0.6), (w,  h)]], dtype=np.int32)
    mask = np.zeros_like(img)
    cv2.fillPoly(mask, trap, 225)
    return cv2.bitwise_and(img, mask)

# ---------------- 3. 후프 변환으로 점 모으기 ----------------
def hough_points(roi):
    lines = cv2.HoughLinesP(roi, 2, np.pi/180, 90, minLineLength=50, maxLineGap=60)
    left, right = [], []
    if lines is None: return left, right
    for x1,y1,x2,y2 in lines[:,0]:
        slope = (y2-y1)/(x2-x1+1e-6)
        if abs(slope) < 0.3: continue
        if slope < 0: left += [(x1,y1), (x2,y2)]
        else: right += [(x1,y1), (x2,y2)]
    return left, right

# ---------------- 4. 2차 다항식 피팅(직선·곡선 OK) ----------------
def fit_poly(points, y_max, deg=2):
    if len(points) < 18: deg = 1
    xs, ys = zip(*points)
    fit = np.polyfit(ys, xs, deg)
    f = np.poly1d(fit)
    y_vals = np.linspace(y_max, y_max*0.6, 100).astype(int)
    x_vals = f(y_vals).astype(int)
    pts = np.stack((x_vals, y_vals), axis=1)
    return pts

# ---------------- 5. 스티어링 각도 계산 ----------------
def steering_angle(left_pts, right_pts, frame):
    h, w, _ = frame.shape
    if len(left_pts)==0 or len(right_pts)==0:
        return 0, 0
    mid_bottom = int((left_pts[0,0] + right_pts[0,0]) / 2)
    deviation  = mid_bottom - w//2
    angle_rad  = np.arctan2(deviation, h*0.6)
    angle_deg = int(angle_rad * 180/np.pi)
    return angle_deg, deviation

# ---------------- 6. PWM 값 계산 함수 (마이크로초 단위로 수정) ----------------
def calculate_pwm_us(angle, deviation):
    Kp = 3
    servo_pwm_us = 1500 + (deviation * Kp)
    servo_pwm_us = int(np.clip(servo_pwm_us, 1000, 2000))

    base_throttle_us = 1600
    Kt = 100

    throttle_pwm_us = base_throttle_us - (abs(angle) * Kt)
    throttle_pwm_us = int(np.clip(throttle_pwm_us, 1520, base_throttle_us))
    return servo_pwm_us, throttle_pwm_us

# ---------------- 7. 최종 합성 ----------------
def draw(frame, left_pts, right_pts, angle, servo_pwm, throttle_pwm):
    h, w, _ = frame.shape
    roi_vertices = np.array([[(0+40, h*0.89), (w * 0.45, h * 0.6), (w * 0.5, h * 0.6), (w-100, h*0.89)]], dtype=np.int32)
    
    cv2.polylines(frame, [roi_vertices], True, (0, 0, 255), 3) # 빨간색으로 ROI 테두리 그리기
    lane = np.zeros_like(frame)
    if len(left_pts):  cv2.polylines(lane, [left_pts], False, (225,0,0), 10)
    if len(right_pts): cv2.polylines(lane, [right_pts], False, (225,0,0), 10)

    if len(left_pts) and len(right_pts):
        pts = np.vstack((left_pts, right_pts[::-1]))
        cv2.fillPoly(lane, [pts], (0, 200, 0))
        center = (left_pts + right_pts) // 2
        cv2.polylines(lane, [center], False, (0, 0, 255), 6)
        mid_bottom = int((left_pts[0,0] + right_pts[0,0]) / 2)
        h, w, _ = frame.shape
        center_x = w // 2
        cv2.circle(lane, (mid_bottom, h-1), 12, (255,0,0), -1)
        cv2.circle(lane, (center_x, h-1), 12, (0,140,255), -1)
        cv2.line(lane, (center_x, h-1), (mid_bottom, h-1), (0,0,255), 4)

    blended = cv2.addWeighted(frame, 0.8, lane, 1, 0)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(blended, f"Servo (us): {servo_pwm}", (30, 50), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(blended, f"Throttle (us): {throttle_pwm}", (30, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(blended, f"Steering: {angle:+d} deg", (30, 150), font, 1.2, (255, 255, 255), 2, cv2.LINE_AA)
    return blended

# ---------------- (추가) 시리얼 통신 설정 ----------------
# TODO: ESP32 송신 보드가 연결된 시리얼 포트 이름으로 변경하세요.
PORT = '/dev/cu.usbmodem31301'
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"시리얼 포트 {PORT}에 연결되었습니다.")
    time.sleep(2) # ESP32 보드가 준비될 시간을 줍니다.
except serial.SerialException as e:
    print(f"오류: 시리얼 포트 {PORT}에 연결할 수 없습니다.")
    print(e)
    exit()
# ----------------------------------------------------

# ---------------- 8. 메인 루프 ----------------
link = "/Users/wonseo/Documents/vscode_project/ac_automuscar/test.mp4"
cap = cv2.VideoCapture(0)     

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break

    edges = preprocess(frame)
    roi   = region_of_interest(edges)
    left_pts_raw, right_pts_raw = hough_points(roi)

    h = frame.shape[0]
    left_pts  = fit_poly(left_pts_raw,  h)  if left_pts_raw  else []
    right_pts = fit_poly(right_pts_raw, h)  if right_pts_raw else []

    angle, deviation = steering_angle(left_pts, right_pts, frame)
    servo_pwm_us, throttle_pwm_us = calculate_pwm_us(angle, deviation)
    out = draw(frame, left_pts, right_pts, angle, servo_pwm_us, throttle_pwm_us)

    # ---------------- (추가) 계산된 PWM 값을 시리얼로 전송 ----------------
    try:
        # "서보PWM,스로틀PWM\n" 형식의 문자열로 만들어 전송합니다.
        pwm_data = f"{servo_pwm_us},{throttle_pwm_us}\n"
        ser.write(pwm_data.encode('utf-8'))
        print(f"전송된 값 (서보, 스로틀): {servo_pwm_us} us, {throttle_pwm_us} us")
    except Exception as e:
        print(f"시리얼 전송 중 오류 발생: {e}")
        break # 오류 발생 시 루프 종료
    # -----------------------------------------------------------------

    cv2.imshow('Lane Detection', out)
    cv2.imshow('Edge Detection', edges)

    width1 = out.shape[1]

    #time.sleep(0.02) # 약 20ms 딜레이 (초당 50회 전송)
    if cv2.waitKey(1)&0xFF == 27: break

# ---------------- (추가) 시리얼 포트 닫기 ----------------
ser.close()
# ----------------------------------------------------
cap.release()
cv2.destroyAllWindows()
