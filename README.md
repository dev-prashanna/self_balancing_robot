# Self-Balancing Robot

A two-wheeled self-balancing robot simulated in MuJoCo with PD control and WASD keyboard controls.

## Requirements

```bash
pip install mujoco numpy
```

## Usage

```bash
python sim.py
```

## Controls

| Key | Action |
|-----|--------|
| W | Move forward |
| S | Move backward |
| A | Turn left |
| D | Turn right |
| Space | Stop |

## Controller

PD balance controller with differential drive steering:

```
τ_balance = Kp * θ + Kd * θ̇
τ_left  = τ_balance + Kf * forward + Kt * turn
τ_right = τ_balance + Kf * forward - Kt * turn
```

| Gain | Value | Purpose |
|------|-------|---------|
| Kp | 1.0 | Proportional (tilt correction) |
| Kd | 0.5 | Derivative (damping) |
| Kf | 0.2 | Forward/back drive |
| Kt | 0.1 | Turn differential |
