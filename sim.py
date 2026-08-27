import mujoco
import mujoco.viewer
import sys
import tty
import termios
import select
import time
import math
import numpy as np

# Keyboard state
forward = 0
turn = 0
last_key_time = 0
KEY_TIMEOUT = 0.1  # seconds - resets state if no key press within this time

# Save original terminal settings
orig_settings = termios.tcgetattr(sys.stdin)

def setup_terminal():
    tty.setcbreak(sys.stdin.fileno())

def restore_terminal():
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, orig_settings)

def poll_keys():
    global forward, turn, last_key_time
    if select.select([sys.stdin], [], [], 0)[0]:
        ch = sys.stdin.read(1)
        last_key_time = time.time()
        if ch == 'w':
            forward = 1
        elif ch == 's':
            forward = -1
        elif ch == 'a':
            turn = -1
        elif ch == 'd':
            turn = 1
        elif ch == ' ':
            forward = 0
            turn = 0

    # Auto-reset if no key press for KEY_TIMEOUT seconds
    if forward != 0 or turn != 0:
        if time.time() - last_key_time > KEY_TIMEOUT:
            forward = 0
            turn = 0

# Load model and create data
model = mujoco.MjModel.from_xml_path("robot.xml")
data = mujoco.MjData(model)

# Tilt chassis 10 degrees around x-axis
tilt_angle = math.radians(10)
data.qpos[0:3] = [0, 0, 0.135]
data.qpos[3:7] = [math.cos(tilt_angle / 2), math.sin(tilt_angle / 2), 0, 0]

mujoco.mj_forward(model, data)

# Get chassis body ID
chassis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "chassis")

print(f"Timestep: {model.opt.timestep}s", flush=True)
print(f"Chassis ID: {chassis_id}", flush=True)
print("Controls: W=forward S=back A=left D=right Space=stop", flush=True)

# PD gains
Kp = 1.0
Kd = 0.5
Kf = 0.3   # forward torque
Kt = 0.5   # turn torque

# Simulation loop
setup_terminal()
step = 0
try:
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            poll_keys()

            # Measure tilt
            w, x, y, z = data.xquat[chassis_id]
            tilt = np.arctan2(
                2 * (w * y + x * z),
                1 - 2 * (y * y + z * z)
            )

            # Calculate balance torque
            balance_torque = Kp * tilt + Kd

            # Add forward and turn commands
            left_torque = balance_torque + Kf * forward + Kt * turn
            right_torque = balance_torque + Kf * forward - Kt * turn

            data.ctrl[0] = left_torque
            data.ctrl[1] = right_torque

            mujoco.mj_step(model, data)
            step += 1

            if step % 100 == 0:
                tilt_degrees = np.degrees(tilt)
                print(f"Tilt: {tilt_degrees:7.2f}° | L: {left_torque:7.3f} R: {right_torque:7.3f} | Fwd: {forward} Turn: {turn}", flush=True)

            viewer.sync()
            time.sleep(0.001)
finally:
    restore_terminal()
