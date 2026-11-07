# Autonomous Differential-Drive Robot (MATLAB)

A 2D MATLAB simulation of an autonomous two-wheeled robot that navigates between waypoints. The robot uses simulated DC motors with realistic response delays, PWM voltage control, PI speed controllers, wheel encoders and sensor noise.

**Goal:** design a robot that can autonomously complete a course while coping with imperfect motors and sensors.

**Team:** [Name 1], [Name 2], [Name 3]

**Demo video:** [link to YouTube / Google Drive]

---

## Results

> Add plots here as you produce them (save them in `results/`).

| Scenario | Completion time (s) | RMS path error (m) | Final position error (m) |
|---|---|---|---|
| Ideal motors | | | |
| Unequal motors, no PI | | | |
| Unequal motors, with PI | | | |
| Noisy encoders, with PI | | | |

![Robot path comparison](results/path_comparison.png)
![Motor step response](results/motor_step_response.png)

---

## Features

- [x] Differential-drive kinematics and animated 2D robot
- [ ] First-order DC motor model with response delay
- [ ] PWM duty-cycle control with voltage saturation
- [ ] Unequal motor efficiencies
- [ ] PI speed controllers with anti-windup
- [ ] Waypoint navigation (heading and position control)
- [ ] Wheel encoder model (quantisation and noise)
- [ ] Performance comparison and dashboard

---

## Requirements

- MATLAB (version: [your version])
- No toolboxes required (base MATLAB only)

---

## How to run

1. Clone the repository:
   ```
   git clone [repository URL]
   ```
2. Open MATLAB and set the repository folder as the Current Folder.
3. Add the source folder to the path:
   ```matlab
   addpath('src');
   ```
4. Run the main simulation:
   ```matlab
   run('main/runSimulation.m')
   ```

---

## Project structure

```
robot-sim/
├── README.md
├── .gitignore
├── main/
│   └── runSimulation.m        % ties everything together
├── src/
│   ├── navController.m        % waypoint / heading controller
│   ├── robotKinematics.m      % differential-drive kinematics
│   ├── motorModel.m           % DC motor dynamics
│   ├── piController.m         % PI speed controller
│   └── encoderModel.m         % encoder quantisation and noise
├── tests/
│   ├── test_motorStepResponse.m
│   └── test_piTuning.m
├── results/                   % saved plots
└── docs/
    └── notes.md
```

---

## How it works

### 1. Kinematics
The robot is modelled as a unicycle. Left and right wheel speeds are converted to forward speed `v` and turn rate `omega`, then integrated to update `x`, `y` and `theta`.

### 2. Motor model
Each motor is modelled as a first-order system:

```
tau * d(omega)/dt = K*V - omega
```

where `tau` is the time constant, `K` is the steady-state gain and `V` is the applied voltage.

### 3. PWM and voltage limits
The controller outputs a duty cycle between -1 and 1. The average motor voltage is `V = duty * Vsupply`.

### 4. Speed control
A PI controller on each wheel compares the reference speed with the measured speed and adjusts the duty cycle, with anti-windup when the output saturates.

### 5. Navigation
A proportional controller turns the robot towards the next waypoint and drives forward at a speed that falls as it gets closer.

### 6. Sensors
Wheel encoders are simulated with discrete ticks and measurement noise, and position is estimated by dead reckoning.

---

## Parameters

| Parameter | Symbol | Value | Unit |
|---|---|---|---|
| Wheel base | `L` | 0.20 | m |
| Wheel radius | `R` | 0.033 | m |
| Motor time constant | `tau` | 0.1 | s |
| Motor gain | `K` | 3.3 | rad/s per V |
| Supply voltage | `Vsupply` | 6 | V |
| Waypoint tolerance | `tolerance` | 0.10 | m |

---

## Team workflow

- `main` always contains code that runs.
- Each task is developed on its own branch (e.g. `day2-motor-model`) and merged through a pull request.
- Commit messages are short and in the present tense (e.g. `Add first-order DC motor model`).
- Pull before starting work and before pushing.

---

## Lessons learned

> Fill this in at the end: what worked, what did not, what you would do differently.

---

## Future work

- Extended Kalman filter fusing encoders with a noisy gyro
- Comparison of PI, PID and feedforward control
- Obstacle avoidance

---

## Authors

- [Name 1] ([GitHub profile link])
- [Name 2] ([GitHub profile link])
- [Name 3] ([GitHub profile link])
