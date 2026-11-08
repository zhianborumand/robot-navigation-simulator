
# Autonomous Differential-Drive Robot Simulator

**A MATLAB project exploring autonomous navigation, motor dynamics and feedback control.**

This project involves developing a 2D simulation of a differential-drive robot capable of navigating to specified destinations. The aim is to investigate how motor characteristics, control algorithms and sensor measurements affect the robot's movement and accuracy.

The simulator is being developed in stages, starting with basic robot movement before introducing motor control, waypoint navigation and more realistic operating conditions.

**Developers:** Zhian Borumand, Rubaiyat Mashrafi, Clive Fernandes  
**Software:** MATLAB R2022b (base MATLAB)  
**Status:** In development, Days 1–2 completed

---

## 1. Project Objectives

- Model a differential-drive robot using MATLAB.
- Simulate DC motor acceleration and PWM speed control.
- Investigate the effects of unequal motor characteristics.
- Implement PI controllers to regulate wheel velocities.
- Develop autonomous navigation between multiple waypoints.
- Introduce simulated wheel encoders and measurement noise.
- Evaluate navigation accuracy and controller performance.

## 2. Development Progress

### Day 1: Robot Movement and Autonomous Navigation

**Status: Completed**

We developed a 2D simulation using differential-drive kinematics. The initial experiments tested straight-line and circular movement by adjusting the velocities of the two wheels.

We then implemented a proportional heading controller, allowing the robot to navigate autonomously towards a target at `(2,2)` metres.

Three heading controller gains were tested:

| Heading gain | Observed behaviour |
|---|---|
| K = 0.5 | Wider curved trajectory |
| K = 2.0 | Faster heading correction |
| K = 5.0 | Sharper turning and a more direct approach |

All three configurations reached the target within the specified 10 cm tolerance.

### Day 2: DC Motor Modelling and PWM Control

**Status: Completed**

The initial simulation assumed that both motors could change speed instantly. We introduced a simplified first-order motor model to simulate acceleration and deceleration.

Our experiments covered:

- Motor response times using time constants of 0.3 s and 0.8 s.
- PWM duty-cycle changes and their effects on wheel speed.
- Differences between two motors receiving identical commands.
- Comparison of equal and unequal motor characteristics.

The results demonstrated that motors with different response times can produce different wheel velocities during acceleration, even when receiving identical PWM commands.

## 3. How It Works

### Differential-Drive Kinematics

The robot uses two independently driven wheels. Its forward and angular velocities are calculated using:

$$
v = \frac{v_R + v_L}{2}
$$

$$
\omega = \frac{v_R - v_L}{L}
$$

Where:
- `vR` = Right wheel velocity (m/s)
- `vL` = Left wheel velocity (m/s)
- `L` = Distance between wheels (m)
- `v` = Robot forward velocity (m/s)
- `omega` = Angular velocity (rad/s)

These velocities are used to update the robot's position and heading during each simulation step.

### DC Motor Model

Each motor is represented using a first-order response:

$$
\frac{dv}{dt} = \frac{v_{\text{target}}-v}{\tau}
$$

Where `tau` is the motor time constant, determining how quickly the motor responds to a speed command.

### PWM Control

PWM is approximated using:

$$
v_{\text{target}} = D v_{\max}
$$

Where:
- `D` = PWM duty cycle (0 to 1)
- `vMax` = Assumed maximum wheel speed (m/s)

The current model is simplified and does not simulate the complete electrical characteristics of a DC motor.

### Autonomous Navigation

The robot calculates its distance and heading relative to a target position.

A proportional controller adjusts the wheel velocities to reduce heading error, while forward speed decreases as the robot approaches the destination.

The robot stops when it enters the specified target tolerance.

## 4. Project Structure

```text
robot-navigation-simulator/
|
|-- day_1/
|   |-- robot_straight.m
|   |-- robot_circle.m
|   |-- robot_autonomous.m
|
|-- day_2/
|   |-- motor_test.m
|   |-- motor_pwm_test.m
|   |-- two_motor_test.m
|
|-- README.md
```

Each script represents a separate experiment and can be executed independently.

## 5. How to Run

**Requirements:** MATLAB R2022b or a compatible version. No additional toolboxes required.

1. Clone the repository:

```bash
git clone https://github.com/zhianborumand/robot-navigation-simulator.git
```

2. Open MATLAB and set the repository directory as the current folder.

3. Run the desired simulation.

For autonomous navigation:

```matlab
run('day_1/robot_autonomous.m')
```

For the two-motor comparison:

```matlab
run('day_2/two_motor_test.m')
```

Parameters such as wheel speed, motor time constant and controller gains can be changed directly within the scripts.

## 6. Development Roadmap

| Stage | Work | Status |
|---|---|---|
| Day 1 | Robot kinematics and autonomous navigation | Completed |
| Day 2 | Motor dynamics and PWM control | Completed |
| Day 3 | Motor integration and PI speed control | Planned |
| Days 4–5 | Waypoint navigation and encoder modelling | Planned |
| Days 6–7 | Performance testing and visual improvements | Planned |

### Planned Performance Tests

Once the complete simulation is integrated, we intend to compare:

| Scenario | Investigation |
|---|---|
| Ideal motors | Baseline navigation performance |
| Unequal motors | Effects of motor mismatch |
| PI-controlled motors | Improvements from speed feedback |
| Noisy wheel encoders | Effects of measurement uncertainty |

Performance will be evaluated using completion time, final position error and trajectory-tracking accuracy.

## 7. Collaboration

Development is coordinated through GitHub.

The `main` branch is intended to contain working versions of the project. Individual features can be developed on separate branches and reviewed through pull requests before merging.

## 8. Future Work

Potential extensions include:

- Trajectory optimisation.
- Obstacle avoidance.
- Sensor fusion.
- Comparison of PI, PID and feedforward control.
- More detailed DC motor electrical modelling.

---

## Authors

- Zhian Borumand
- Rubaiyat Mashrafi
- Clive Fernandes
