#include <AccelStepper.h>
#include <AS5600.h>
#include <Wire.h>

// Define the stepper motor connections
#define dirPin 8  // Direction
#define stepPin 9 // Step
#define LED_PIN 13

#define encoderA 2
#define encoderB 3

#define frac(x) (int(1000 * (x - int(x))))

// Create an instance of the AccelStepper class
AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

#define stepsPerRevolution 200 * 4 // 200 steps per revolution * 4 microsteps

volatile long encoderPosition = 0;

// Create an instance of the AS5600 class
// AMS_5600 ams5600;
float pendulum_initial_position = 0.0f;
float pendulum_target_deg = 180.0;
float motor_target_pos = 0.0;

float prevError = 0.0;
float integral = 0.0;

// // Variables for PID control
// float Kp = 3.0;
// float Ki = 0.0;
// float Kd = 0.0;

// Ziegler-Nichols parameters
float Ku = 2.0; // critical gain
float Tu = 0.1; // oscillation period

// Variables for PID control
float Kp = 1.5 * Ku;
float Ki = 0.15 * Kp / Tu;
float Kd = 0 * Kp * Tu / 8;

// Define the control frequency and period
const int controlFrequency = 1000;                // in Hertz
const float controlPeriod = 1 / controlFrequency; // in seconds

// Define variables for moving average filter
double pendulum_actual_deg;

// State machine variables
enum State
{
    WAITING,
    BALANCING
};
State state = WAITING;

bool can_print = false;

void tare_pendulum_encoder()
{
    pendulum_initial_position = 0.0;
    pendulum_initial_position = convertRawAngleToDegrees();
}

// Computes the smoothing coefficient for an exponential low-pass filter based on the desired cutoff frequency.
// This coefficient, `alpha`, determines how much of the previous state and the new value contribute to the filtered result.
//
// The coefficient is calculated based on the following parameters:
// - `freq`: Desired cutoff frequency in Hz (cycles per second). This defines the point below which the signal will pass
//          through the filter and above which it will be attenuated. Frequencies higher than this cutoff will be filtered out.
// - `dt`: Time interval between updates (in seconds). This value is the time step between consecutive filter updates.
//
// The coefficient `alpha` is bounded between 0.0 and 1.0:
// - 0.0 means no smoothing (only the latest value is used, and no memory of previous states).
// - 1.0 means no update (only the previous state is retained, and the new values are ignored).
//
// The function computes `alpha` using the following formula, where `omega` is the angular frequency corresponding to the desired cutoff frequency:
//   omega = 2 * π * freq
//   alpha = (1 - omega * dt / 2) / (1 + omega * dt / 2)
//
// After computing the coefficient, it is clamped between 0.0 and 1.0 to ensure stability and prevent extreme values.
//
// Example usage:
//   double alpha = alpha_from_freq(500.0, 0.002); // For a 500 Hz cutoff with a 2ms time step.
//
// Returns:
//   The clamped smoothing coefficient `alpha`, which is used in the exponential low-pass filter.
double alpha_from_freq(double freq, double dt)
{
    // Compute the angular frequency (omega)
    double omega = 2.0 * M_PI * freq;

    // Compute the smoothing coefficient (alpha)
    double coeff = (1.0 - omega * dt / 2.0) / (1.0 + omega * dt / 2.0);

    // Clamp the coefficient between 0.0 and 1.0 to prevent instability
    if (coeff < 0.0)
    {
        coeff = 0.0;
    }
    if (coeff > 1.0)
    {
        coeff = 1.0;
    }

    return coeff;
}

void print_plot(float target, float pos)
{
    static unsigned long t0 = 0;
    if (can_print)
    {
        unsigned long t1 = millis();
        if (t1 - t0 > 10)
        {
            t0 = t1;
            // Print the actual and target motor positions
            Serial.print(stepper.currentPosition());
            Serial.print(",");
            Serial.print(target);
            Serial.print(",");
            Serial.println(pos);
        }
    }
}

void check_serial()
{
    if (Serial.available())
    {
        char buffer[1024];
        int size = Serial.readBytes(buffer, 1024);
        if (size >= 0)
        {
            char cmd = buffer[0];
            switch (cmd)
            {
            case 'P':
            case 'p':
                can_print = !can_print;
                break;
            case 'M':
            case 'm':
                //print_magnet_info();
                break;
            case 'T':
            case 't':
                tare_pendulum_encoder();
                break;
            }
        }
    }
}

void setup()
{
    Serial.begin(115200); // Start the serial communication
    pinMode(encoderA, INPUT_PULLUP);
    pinMode(encoderB, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, CHANGE);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(200000);
    stepper.setAcceleration(100000);

    // Set the enable pin for the stepper motor driver and
    // invert it because we are using a DRV8825 board with an
    // active-low enable signal (LOW = enabled, HIGH = disabled)
/*    stepper.setEnablePin(5);
    stepper.setPinsInverted(false, false, true);
*/
    // Set initial position
    tare_pendulum_encoder();

    digitalWrite(LED_PIN, LOW);

    pendulum_actual_deg = convertRawAngleToDegrees();
}

void blink()
{
    static unsigned long t0 = 0;
    static bool is_on = false;
    int period = can_print ? 500 : 100;
    unsigned long t1 = millis();
    if (t1 - t0 > period)
    {
        digitalWrite(LED_PIN, is_on ? HIGH : LOW);
        is_on = !is_on;
        t0 = t1;
    }
}

void loop()
{
    blink();

    check_serial();

    static unsigned long prevTimeUS = 0;
    unsigned long currentTimeUS = micros();
    unsigned long dt = currentTimeUS - prevTimeUS;
    float elapsedTime = (float)dt * 1e-6;
    prevTimeUS = currentTimeUS;

    // first, we wait for a person to move the pendulum close to the vertical position.
    // then we start the motor and we try to balance it.
    // we should not command the motor beyond +-90 degrees from its starting position.

    double alpha = alpha_from_freq(500.0, (double)dt * 1e-6);
    // Get the pendulum position
    pendulum_actual_deg = alpha * pendulum_actual_deg + (1.0 - alpha) * convertRawAngleToDegrees();

    // Find closest upright target
    int revs = pendulum_actual_deg / 360;
    pendulum_target_deg = 180.0f * (pendulum_actual_deg > 0 ? 1.0f : -1.0f) + 360.0f * (float)revs;

    float margin_in_deg = 20.0; // in degrees
    bool pendulum_close_to_vertical = fabs(pendulum_target_deg - pendulum_actual_deg) <= margin_in_deg;

    if (state == WAITING)
    {
        if (pendulum_close_to_vertical)
        {
            // Switch the state to 'balancing'
            state = BALANCING;

            // Enable the motor outputs
            //stepper.enableOutputs();

            // Set the motor target position to the current position
            motor_target_pos = stepper.currentPosition();
        }
    }
    else if (state == BALANCING)
    {
        if (!pendulum_close_to_vertical)
        {
            // Switch the state to 'waiting'
            state = WAITING;

            // Stop the motor
            stepper.stop();

            // Disable the motor outputs
            //stepper.disableOutputs();

            // Reset the PID control variables
            prevError = 0.0;
            integral = 0.0;
        }
    }

    if (state == BALANCING && elapsedTime >= controlPeriod)
    {
        // Calculate the error
        float error = pendulum_target_deg - pendulum_actual_deg;

        // Calculate the integral
        integral += error * elapsedTime;

        // Calculate the derivative
        float derivative = (error - prevError) / elapsedTime;
        prevError = error;

        // Calculate the PID output
        float output = Kp * error + Ki * integral + Kd * derivative;

        // Limit the output to prevent the motor from moving too fast
        long output_limit = convertDegreesToSteps(30);
        if (output > output_limit)
        {
            output = output_limit;
        }
        else if (output < -output_limit)
        {
            output = -output_limit;
        }

        // Update motor target position based on PID output
        motor_target_pos = stepper.currentPosition() + output;

        // Limit the motor target position to prevent the motor from moving beyond +-90 degrees
        /*if (abs(motor_target_pos) > convertDegreesToSteps(90))
        {
            motor_target_pos = stepper.currentPosition();
        }
        */
        // Move the motor
        stepper.moveTo(motor_target_pos);
    }

    if (state == BALANCING)
    {
        // Move the motor to the target position
        stepper.moveTo(motor_target_pos);

        // Run the stepper motor
        stepper.run();
    }

    print_plot(motor_target_pos, pendulum_actual_deg);
}

long convertDegreesToSteps(float degrees)
{
    // Convert degrees to steps
    return degrees * stepsPerRevolution / 360;
}

/*
 * Convert the raw angle from the AS5600 magnetic encoder to degrees.
 */

long readEncoder() {
  bool A = digitalRead(encoderA);
  bool B = digitalRead(encoderB);

  encoderPosition += (A == B) ? 1 : -1;

  return encoderPosition;
}

float convertRawAngleToDegrees()
{
    static long raw_prev = 0;
    static bool first_reading = true;
    static float position = 0.0f;
    // Get the current position of the AS5600
    noInterrupts();
    long raw = encoderPosition;
    interrupts();

    if (first_reading)
    {
        raw_prev = raw;
        first_reading = false;
    }
    long delta = raw - raw_prev;
    // Handle wrap around
    /*if (delta > 599)
        delta -= 1200;
    if (delta < -599)
        delta += 1200;
    */// Map the 0-1199 segments of the Encoder to 0–360 degrees
    // 360 degrees / 1200 segments = 0.3 degrees per segment
    position += (float)delta * 0.3;
    raw_prev = raw;

    return position;
}