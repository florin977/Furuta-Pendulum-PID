# Inverted Furuta Pendulum (PID & State Machine Control)

A real-time embedded control system designed to maintain equilibrium on a physical rotary inverted pendulum (Furuta pendulum). This project implements a discrete PID controller and a custom state machine on an Arduino Leonardo to constantly adjust the position of a NEMA17 stepper motor based on real-time feedback from an optical encoder.

## Demo

https://github.com/user-attachments/assets/03d556b9-91ac-409e-8e29-39d9371ed93b

---

## Hardware Architecture

The physical setup consists of a driven horizontal arm and a freely rotating vertical pendulum.

* **Microcontroller:** Arduino Leonardo (ATmega32U4)
* **Actuator:** NEMA17 Stepper Motor
* **Sensor:** Optical Encoder (configured for 1200 counts/rev via hardware interrupts)
* **Motor Driver:** TB6600

---

## Software Stack & Features

* **Language:** C++ (Arduino Framework)
* **Dependencies:** `AccelStepper.h`
* **State Machine:** Implements robust state transitions (`WAITING`, `SWINGING_UP`, `BALANCING`) based on angular thresholds.
* **Signal Processing:** Utilizes a software low-pass filter to smooth the velocity derivatives calculated from the raw encoder data, preventing high-frequency noise from destabilizing the derivative term.
* **Serial Interface:** Includes a real-time serial command listener (115200 baud) for live debugging and state toggling.

---

## Control Theory Implementation

### 1. PID Balancing Controller

The primary balancing loop calculates the error $e(t)$ between the target upright position ($180^\circ$) and the current position. The control signal $u(t)$ sent to the stepper motor speed controller is defined as:

$$u(t) = K_p e(t) + K_i \int e(t)dt + K_d \frac{de(t)}{dt}$$

*Current tuning parameters: $K_p = 325$, $K_i = 0.1$, $K_d = 0.01$*

### 2. Velocity Smoothing (Low-Pass Filter)

To calculate the derivative term accurately without amplifying sensor noise, the angular velocity is passed through a discrete low-pass filter:

$$v_{t} = \alpha \left(\frac{\Delta \theta}{\Delta t}\right) + (1 - \alpha) v_{t-1}$$

### 3. Experimental Controllers

The codebase also includes the mathematical framework for advanced control algorithms (currently disabled/in-development):

* **LQR (Linear Quadratic Regulator):** Computes the optimal control matrix based on the state vector $x$ (stepper position, stepper velocity, pendulum angle, pendulum velocity) using $u = -Kx$.
* **Energy-Based Swing-Up:** Calculates kinetic and potential energy to pump the pendulum up to the balancing threshold.

---

## Getting Started

### Installation & Flashing

1. Clone the repository:

   ```bash
   git clone https://github.com/florin977/Furuta-Pendulum-PID.git
   ```

2. Open the project in Arduino IDE.

3. Install the AccelStepper library via the Library Manager

4. Verify the pin definitions in the code match your hardware:

    Encoder: Pins 2 & 3 (Hardware Interrupts)

    Stepper: Pin 8 (DIR), Pin 9 (STEP)

5. Build and flash the firmware to your Arduino Leonardo.

### Serial Commands

Open the Serial Monitor at **115200 baud**. You can send the following single-character commands to interact with the pendulum:

* `P` : Toggle live telemetry printing (Pendulum Angle vs. Target Speed).
* `R` : Reset current position and zero out the integral accumulators.
* `S` : Toggle between `WAITING` and `SWINGING_UP` states.

---

## License

This project is open-source and available under the MIT License.
