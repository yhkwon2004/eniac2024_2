import numpy as np
import matplotlib.pyplot as plt

# 차량 파라미터
L = 2.0  # 차량의 축거
Ld = 5.0  # Look-ahead distance (목표점까지의 거리)
delta_max = np.radians(25)  # 최대 조향각 (라디안)


# 코스 정의
def create_course(course_number):
    x_path = np.linspace(0, 40, 100)  # x좌표를 40까지 확장
    if course_number == 1:
        y_path_upper = 0.5 * np.sin(x_path) + 2  # 첫 번째 코스 (사인 곡선)
        y_path_lower = 0.5 * np.sin(x_path)
    elif course_number == 2:
        y_path_upper = 1.0 + np.cos(x_path)  # 두 번째 코스 (코사인 곡선)
        y_path_lower = 1.0 + np.cos(x_path) - 1.5
    elif course_number == 3:
        y_path_upper = (0.5 * x_path / 40) + 2  # 세 번째 코스 (직선 상승)
        y_path_lower = (0.5 * x_path / 40) + 1
    elif course_number == 4:
        y_path_upper = 2 - 0.1 * (x_path - 20) ** 2  # 네 번째 코스 (포물선)
        y_path_lower = 0.5 * np.sin(x_path)  # 아래는 사인 곡선
    else:
        raise ValueError("Invalid course number")

    return x_path, y_path_upper, y_path_lower


# 초기 설정
current_course = 1
total_courses = 4
course_repeats = 4

# 초기 차량 상태 [x, y, theta]
state = np.array([0.0, 0.0, np.radians(0.0)])  # 중앙선 시작점에서 출발
time_elapsed = 0.0  # 경과 시간
speed = 1.0  # 차량 속도 설정 (속도를 높임)


def pure_pursuit_control(state, x_path, y_path_center, Ld, L):
    # 가장 가까운 중앙선상의 점 찾기
    distances_center = np.sqrt((x_path - state[0]) ** 2 + (y_path_center - state[1]) ** 2)
    index = np.argmin(distances_center)

    # 목표점 설정 (Look-ahead 기반)
    look_ahead_index = min(index + int(Ld * 10), len(x_path) - 1)
    goal = [x_path[look_ahead_index], y_path_center[look_ahead_index]]

    # 차량 좌표계로 목표점 변환
    dx = goal[0] - state[0]
    dy = goal[1] - state[1]
    alpha = np.arctan2(dy, dx) - state[2]

    # 조향각 계산
    delta = np.arctan2(2.0 * L * np.sin(alpha) / Ld, 1.0)  # 조향각 계산식
    delta = np.clip(delta, -delta_max, delta_max)  # 최대 조향각을 넘지 않도록

    return delta, goal


def draw_motorcycle(ax, state):
    # 오토바이의 모양을 그리기 위한 함수
    bike_length = 1.0
    bike_width = 0.3
    # 오토바이의 네 꼭짓점 계산
    bike = np.array([
        [-bike_length / 2, -bike_width / 2],
        [bike_length / 2, -bike_width / 2],
        [bike_length / 2, bike_width / 2],
        [-bike_length / 2, bike_width / 2],
        [-bike_length / 2, -bike_width / 2]
    ])

    # 오토바이 회전 및 이동
    rotation_matrix = np.array([
        [np.cos(state[2]), -np.sin(state[2])],
        [np.sin(state[2]), np.cos(state[2])]
    ])
    bike_rotated = bike @ rotation_matrix.T
    bike_rotated += state[:2]  # 오토바이의 위치 추가

    ax.plot(bike_rotated[:, 0], bike_rotated[:, 1], 'b', linewidth=3)  # 오토바이 그리기

    # 바퀴 그림
    wheel_radius = 0.1
    front_wheel_center = state[:2] + np.array([0.5, 0])
    rear_wheel_center = state[:2] + np.array([-0.5, 0])

    front_wheel = plt.Circle(front_wheel_center, wheel_radius, color='g', fill=True)
    rear_wheel = plt.Circle(rear_wheel_center, wheel_radius, color='g', fill=True)

    ax.add_artist(front_wheel)
    ax.add_artist(rear_wheel)

    # 조향각 표시
    wheel_angle = np.degrees(state[2])  # 라디안을 도로 변환
    ax.text(state[0] + 0.5, state[1] + 0.5, f"Steering Angle: {wheel_angle:.1f}°", fontsize=10, color='red')


# 시뮬레이션 루프
dt = 0.1  # 시간 간격

plt.figure(figsize=(12, 6))  # 그래프 크기 조정

# 종료 플래그
running = True


def on_key(event):
    global running
    if event.key == 'q':
        running = False


# 키 이벤트 연결
plt.gcf().canvas.mpl_connect('key_press_event', on_key)

for _ in range(course_repeats):
    for current_course in range(1, total_courses + 1):
        # 코스 생성
        x_path, y_path_upper, y_path_lower = create_course(current_course)
        y_path_center = (y_path_upper + y_path_lower) / 2  # 중앙선 업데이트
        state = np.array([0.0, y_path_center[0], np.radians(0.0)])  # 중앙선 시작점에서 출발
        time_elapsed = 0.0  # 경과 시간 초기화

        while running:  # Q 키가 눌리지 않는 동안 계속 회전
            time_elapsed += dt  # 경과 시간 업데이트

            if time_elapsed > 3.0:  # 3초 후에 출발
                delta, goal = pure_pursuit_control(state, x_path, y_path_center, Ld, L)

                # 차량 상태 업데이트 (간단한 운동학 모델)
                state[0] += speed * np.cos(state[2]) * dt  # 속도를 반영하여 위치 업데이트
                state[1] += speed * np.sin(state[2]) * dt
                state[2] += delta / L * np.tan(delta) * dt  # 차량 방향 업데이트

                # 차량이 오른쪽으로만 주행하도록 설정
                if state[0] >= 40:  # x좌표가 40에 도달하면
                    state[0] = 40  # x좌표를 40으로 고정
                    # 차량 방향을 바꾸지 않도록 조정
                    state[2] = np.radians(0)  # 차량의 방향을 오른쪽으로 고정

            # 시각화
            plt.cla()
            plt.plot(x_path, y_path_upper, "-r", label="Upper Lane")  # 상단 차선
            plt.plot(x_path, y_path_lower, "-r", label="Lower Lane")  # 하단 차선
            plt.plot(x_path, y_path_center, "--g", label="Center Line")  # 중앙선
            draw_motorcycle(plt.gca(), state)  # 오토바이 그리기
            plt.axis("equal")
            plt.xlim(-5, 40)  # x축 한계 설정
            plt.ylim(-5, 10)  # y축 한계 설정
            plt.grid()
            plt.legend()

            # 차량 상태 설명 추가
            plt.text(state[0] + 0.5, state[1] + 0.5, f"Position: ({state[0]:.2f}, {state[1]:.2f})", fontsize=10,
                     color='blue')
            plt.text(state[0] + 0.5, state[1], f"Theta: {np.degrees(state[2]):.2f} degrees", fontsize=10, color='blue')
            plt.pause(0.001)

plt.close()  # 시뮬레이션 종료 후 플롯 닫기
