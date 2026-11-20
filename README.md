# Model-based design: from Simulink models to an Arduino controller

Three labs from the course **Model-Based Software Design** (MSc Mechatronic Engineering, Politecnico di Torino, 2026, grade 30/30). They take one workflow from a physical model to embedded code: model the plant and the controller, model the control logic as a state machine, verify it against requirement test cases, then generate code and run it on a microcontroller.

| Folder | What it contains | Tools |
|---|---|---|
| `01_dc_motor_pid/` | Parametric Simulink model of a DC motor (J, b, Ke, Kt, R, L) and a PID speed controller (Kp = 100, Ki = 200, Kd = 10). Parameters live in data dictionaries (`*.sldd`); models follow the MAAB guidelines. | Simulink, data dictionaries |
| `02_smart_bin_logic/` | Stateflow control logic for a smart garbage bin: a proximity sensor reading `distance_cm` drives a servo lid (closed / half open / open: 0°, 90°, 180°) and a status LED (off, on, blinking at 2 s period, 50 % duty). Includes timers (5 s request, 10 s open window, 30 s lock-out) and the five requirement test cases. | Simulink, Stateflow, test cases |
| `03_arduino_deployment/` | The same logic wired to the Arduino support package (HC-SR04 ultrasonic sensor on pins 5/4, servo on pin 8, LED on pin 13), built with full executable export into `controller_auno.hex` and run on an Arduino Uno simulated in SimulIDE (`smartbin.sim1`). | Embedded code generation, Arduino Uno, SimulIDE |

## How to run
- Open the `.slx` files in MATLAB/Simulink (with Stateflow and the Simulink Support Package for Arduino Hardware for folder 03).
- `01_dc_motor_pid/build_lab1_models.m` rebuilds the lab 1 models from script.
- Folder 03: load `controller_auno.hex` into the Arduino Uno of `smartbin.sim1` in SimulIDE 1.1.0, move the voltage source that emulates the hand distance and watch the servo and LED.

Author: Raman Jahandari
