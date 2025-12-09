import math
from DM_CAN import *
import serial
import time
import sys
import statistics

# ==============================================================================
# 1. 초기 설정
# ==============================================================================
serial_port = '/dev/ttyACM0'   # 라즈베리파이
baud_rate = 921600

Motor1 = Motor(DM_Motor_Type.DM4310, 0x01, 0x00)

try:
    serial_device = serial.Serial(serial_port, baud_rate, timeout=0.01)
except Exception as e:
    print(f"Error opening serial port {serial_port}: {e}")
    sys.exit(1)

MotorControl1 = MotorControl(serial_device)
MotorControl1.addMotor(Motor1)

# 모드 + enable
MotorControl1.switchControlMode(Motor1, Control_Type.VEL)
MotorControl1.enable(Motor1)

# ==============================================================================
# 2. 루프 파라미터
# ==============================================================================
control_frequency = 1000.0   # Hz
T = 1.0 / control_frequency   # 주기
N_ITER = 10000               # 총 반복 횟수

print(f"Start Loop @ {control_frequency} Hz, {N_ITER} iters")

# 메모리에 미리 배열 준비 (I/O 없음)
dt_list = [0.0] * N_ITER
exec_list = [0.0] * N_ITER

overrun_count = 0

# ==============================================================================
# 3. 실시간 제어 루프
# ==============================================================================
t0 = time.perf_counter()
prev = t0
deadline = t0 + T

for i in range(N_ITER):

    # 루프 시작 시간
    t_loop = time.perf_counter()

    # dt = 진짜 루프 간격 → 지터 측정용
    dt_list[i] = t_loop - prev
    prev = t_loop

    # ------------------------
    # 제어 명령 송신
    # ------------------------
    q_dot = 0.1
    MotorControl1.control_Vel(Motor1, q_dot)

    # 제어 계산 실행시간
    t_after = time.perf_counter()
    exec_list[i] = t_after - t_loop

    # ------------------------
    # 주기 유지
    # ------------------------
    if t_after < deadline:
        time.sleep(deadline - t_after)
    else:
        overrun_count += 1  # deadline miss

    deadline += T


# ==============================================================================
# 4. 종료 및 파일 저장(I/O는 루프 끝나고 한 번만)
# ==============================================================================
print("Stopping motor...")
MotorControl1.disable(Motor1)
serial_device.close()

print("Saving logs...")
with open("dt_log.txt", "w") as f:
    for dt in dt_list:
        f.write(f"{dt}\n")

with open("exec_log.txt", "w") as f:
    for ex in exec_list:
        f.write(f"{ex}\n")

print("\n=== RESULTS ===")
dt_ms = [x * 1000.0 for x in dt_list[5:]]  # 앞부분 워밍업 제외
print(f"mean dt   : {statistics.mean(dt_ms):.6f} ms")
print(f"stdev dt  : {statistics.pstdev(dt_ms):.6f} ms")
print(f"min/max dt: {min(dt_ms):.6f} ms / {max(dt_ms):.6f} ms")

print(f"Overruns  : {overrun_count} / {N_ITER}")
