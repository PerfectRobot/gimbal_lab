import math
from DM_CAN import *
import serial
import time
import sys
import statistics
import gc

gc.disable()  # 선택: GC 끄기 (지터 더 줄이려고)


def generate_trapezoid_velocity_profile(total_angle, vmax, accel, freq):
    """
    Build a trapezoidal (or triangular) velocity profile that respects maximum speed/acceleration.
    Returns a list of per-sample velocity commands for the given loop frequency.
    """
    if total_angle <= 0:
        raise ValueError("total_angle must be positive")
    if vmax <= 0 or accel <= 0 or freq <= 0:
        raise ValueError("vmax, accel, and freq must be positive")

    dt = 1.0 / freq
    profile = []
    pos = 0.0
    vel = 0.0
    eps = 1e-9

    while pos < total_angle - eps or vel > eps:
        dist_to_go = max(total_angle - pos, 0.0)
        decel_dist = (vel ** 2) / (2.0 * accel) if accel > 0 else float("inf")

        if dist_to_go <= decel_dist:
            vel = max(vel - accel * dt, 0.0)
        else:
            vel = min(vel + accel * dt, vmax)

        profile.append(vel)
        pos += vel * dt

        if len(profile) > int(freq * 120):  # safety guard: 2 minutes worth of samples
            raise RuntimeError("Velocity profile generation exceeded expected length")

    return profile

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


time.sleep(0.5)            # 모터 안정화

MotorControl1.set_zero_position(Motor1)   # <<< 핵심
time.sleep(0.3)            # zero 적용 기다림

MotorControl1.refresh_motor_status(Motor1)
print("Zero set. Initial pos:", Motor1.getPosition())


# ==============================================================================
# 2. 루프 파라미터
# ==============================================================================
control_frequency = 1000.0   # Hz
T = 1.0 / control_frequency  # 주기

# 180도(π rad) 이동을 위한 사다리꼴 속도 프로파일 설정
TOTAL_MOVE_RAD = math.pi
MAX_VELOCITY = 1.0           # rad/s
MAX_ACCEL = 2.0              # rad/s^2

velocity_profile = generate_trapezoid_velocity_profile(
    total_angle=TOTAL_MOVE_RAD,
    vmax=MAX_VELOCITY,
    accel=MAX_ACCEL,
    freq=control_frequency,
)
N_ITER = len(velocity_profile)
profile_duration = N_ITER * T

print(f"Trapezoid profile: {TOTAL_MOVE_RAD:.3f} rad in {profile_duration:.3f} s ({N_ITER} samples)")

print(f"Start Loop @ {control_frequency} Hz, {N_ITER} iters")

# 메모리에 미리 배열 준비 (I/O 없음)
dt_list   = [0.0] * N_ITER
exec_list = [0.0] * N_ITER

pos_list  = [0.0] * N_ITER  # 위치(rad)
vel_list  = [0.0] * N_ITER  # 속도(rad/s)
trq_list  = [0.0] * N_ITER  # 토크(Nm)
vel_cmd_list = [0.0] * N_ITER

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
    q_dot = velocity_profile[i]
    MotorControl1.control_Vel(Motor1, q_dot)
    # control_Vel 내부에서 recv() 호출 → Motor1 상태가 갱신됨

    #MotorControl1.refresh_motor_status(Motor1)

    # 모터 상태 읽어서 RAM에 저장 (I/O 아님)
    pos_list[i] = Motor1.getPosition()
    vel_list[i] = Motor1.getVelocity()
    trq_list[i] = Motor1.getTorque()
    vel_cmd_list[i] = q_dot

    # 제어 계산 + 상태 읽기까지 실행시간
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

with open("pos_log.txt", "w") as f:
    for p in pos_list:
        f.write(f"{p}\n")

with open("vel_log.txt", "w") as f:
    for v in vel_list:
        f.write(f"{v}\n")

with open("trq_log.txt", "w") as f:
    for tau in trq_list:
        f.write(f"{tau}\n")

with open("vel_cmd_log.txt", "w") as f:
    for cmd in vel_cmd_list:
        f.write(f"{cmd}\n")

print("\n=== RESULTS ===")
dt_ms = [x * 1000.0 for x in dt_list[5:]]  # 앞부분 워밍업 제외
print(f"mean dt   : {statistics.mean(dt_ms):.6f} ms")
print(f"stdev dt  : {statistics.pstdev(dt_ms):.6f} ms")
print(f"min/max dt: {min(dt_ms):.6f} ms / {max(dt_ms):.6f} ms")

print(f"Overruns  : {overrun_count} / {N_ITER}")
