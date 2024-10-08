import numpy as np
import matplotlib.pyplot as plt

# 차량 파라미터
L = 2.0  # 차량의 축거
Ld = 2.0  # Look-ahead distance (목표점까지의 거리)
delta_max = np.radians(25)  # 최대 조향각 (라디안)

# 2차 곡선 경로 생성
a, b, c = 0.1, 0.0, 0.0
x_path = np.linspace(0, 20, 100)
y_path = a * x_path**2 + b * x_path + c  # y_path 수정

# 초기 차량 상태 [x, y, theta]
state = np.array([0.0, y_path[0], np.radians(0.0)])  # 초기 차량 위치와 방향

def pure_pursuit_control(state, x_path, y_path, Ld, L):
    # 가장 가까운 경로상의 점 찾기
    distances = np.sqrt((x_path - state[0])**2 + (y_path - state[1])**2)
    index = np.argmin(distances)

    # 목표점 설정 (Look-ahead 기반)
    look_ahead_index = min(index + int(Ld * 10), len(x_path) - 1)
    goal = [x_path[look_ahead_index], y_path[look_ahead_index]]

    # 차량 좌표계로 목표점 변환
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    alpha = np.arctan2(dy, dx) - state[2]

    # 조향각 계산
    delta = np.arctan2(2.0 * L * np.sin(alpha) / Ld, 1.0)  # 수정된 조향각 계산식
    delta = np.clip(delta, -delta_max, delta_max)  # 최대 조향각을 넘지 않도록
    return delta, goal

# 시뮬레이션 루프
dt = 0.1  # 시간 간격
speed = 1.0  # 차량 속도 설정

# 시각화 설정
plt.figure(figsize=(12, 6))  # 그래프 크기 조정

# 종료 플래그
running = True
def on_key(event):
    global running
    if event.key == 'q':
        running = False

# 키 이벤트 연결
plt.gcf().canvas.mpl_connect('key_press_event', on_key)

# 시뮬레이션 루프
while running:
    delta, goal = pure_pursuit_control(state, x_path, y_path, Ld, L)

    # 차량 상태 업데이트 (간단한 운동학 모델)
    state[0] += speed * np.cos(state[2]) * dt  # x 위치 업데이트
    state[1] += speed * np.sin(state[2]) * dt  # y 위치 업데이트
    state[2] += delta / L * np.tan(delta) * dt  # 차량 방향 업데이트

    # 시각화
    plt.cla()
    plt.plot(x_path, y_path, "-r", label="Path")  # 경로 그리기
    plt.plot(state[0], state[1], "ob", label="Car")  # 차량 그리기
    plt.plot(goal[0], goal[1], "xg", label="Goal")  # 목표 지점 그리기
    plt.axis("equal")
    plt.xlim(-5, 25)  # x축 한계 설정
    plt.ylim(-5, 5)   # y축 한계 설정
    plt.grid()
    plt.legend()
    plt.pause(0.001)

plt.close()  # 시뮬레이션 종료 후 플롯 닫기
