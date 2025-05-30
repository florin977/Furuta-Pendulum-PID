#include <AccelStepper.h>

// Gains for the system (tune these for a more stable system)
float K[4] = {0, 0, 574.8237, 0}; // Calculated in Matlab; First 2 with '+' instead of '-' ?

float Kp = 700;
float Ki = 1;
float Kd = 0.01;

float integral = 0;
bool firstPIDcall = true;

#define ENCODER_A 2
#define ENCODER_B 3

#define DIR_PIN 8
#define STEP_PIN 9

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Constants
#define STEPS_PER_REV 200 * 4 // 200 Full steps * 4 microstepping
#define ENCODER_COUNTS 1200
#define DEGREES_PER_COUNT 360 / 1200
#define SWING_UP_START 100
#define SWING_UP_STEP 5
#define RADIANS_PER_DEGREE 0.0174532925

volatile long rawEncoderPosition = 0;

// All angles in DEGREES

float pendulumInitialPosition = 0.0;
float pendulumTargetPosition = 180.0;
float motorTargetSpeed = 0.0;

float previousError = 0.0;

float controlFrequency = 100; // in Hertz
float controlPeriod = 1 / controlFrequency; // in seconds

bool justStarted = true;

double pendulumCurrentPosition = 0.0;
long stepperCurrentPosition = 0;

bool canPrintDebug = false;
bool firstPendulumVelocityCall = true;
bool firstStepperVelocityCall = true;

long swingUpPosition = SWING_UP_START;

enum STATE 
{
  SWINGING_UP,
  BALANCING,
  WAITING
};

STATE state = SWINGING_UP;

void resetPosition()
{
  pendulumInitialPosition = 0.0;
  pendulumCurrentPosition = 0.0;
  rawEncoderPosition = 0;
  integral = 0;
  swingUpPosition = SWING_UP_START;
  firstPendulumVelocityCall = true;
  firstStepperVelocityCall = true;
  firstPIDcall = true;
}

// Alpha coeffiecient for software low-pass filter
double alphaFromFrequency(double freq, double dt)
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

void printDebugging(float targetSpeed, float currendPendulumPosition)
{
  static unsigned long lastPrintTime = 0;

  if (canPrintDebug)
  {
    unsigned long currentPrintTime = millis();

    if (currentPrintTime - lastPrintTime >= 50) // 50 miliseconds delay
    {
      lastPrintTime = currentPrintTime;
      Serial.print("Current Stepper Target Speed: ");
      Serial.print(targetSpeed);
      Serial.print("| Current Pendulum Angle: ");
      Serial.println(currendPendulumPosition);
    }
  }
}

// P: toggle print debug; R: set current position as 0; S: stop/start swing up
void handleInput()
{
  if (Serial.available())
  {
    char buffer[1024];
    int size = Serial.readBytes(buffer, 1024);

    if (size >= 0)
    {
      char command = toUpperCase(buffer[0]);

      switch (command) 
      {
        case 'P':
          canPrintDebug = !canPrintDebug;
          break;
        case 'R':
          resetPosition();
          break;
        case 'S':
          if (state == WAITING)
          {
            state = SWINGING_UP;
          }
          else 
          {
            state = WAITING;
          }
          break;
        }

    } 
  }
}

long updateEncoder()
{
  bool A = digitalRead(ENCODER_A);
  bool B = digitalRead(ENCODER_B);

  rawEncoderPosition += (A == B) ? 1 : -1;

  // Reduce to (-360, 360)
  if (abs(rawEncoderPosition) >= ENCODER_COUNTS)
  {
    rawEncoderPosition = 0;
  }

  return rawEncoderPosition;
}

float convertRawAngleToDegrees()
{
  return (float) rawEncoderPosition * DEGREES_PER_COUNT;
}

float sign(float value)
{
  if (value > 0.0)
  {
    return 1.0;
  }
  if (value < 0.0)
  {
    return -1.0;
  }

  return 0.0;
}

float computePendulumVelocity(float currentPendulumVelocityPosition, float elapsedPendulumVeclocityTime)
{
  static float previousPendulumVelocityPosition = 0.0;

  if (firstPendulumVelocityCall)
  {
    firstPendulumVelocityCall = false;
    previousPendulumVelocityPosition = currentPendulumVelocityPosition;
    return 0.0;
  }

  float velocity = (currentPendulumVelocityPosition - previousPendulumVelocityPosition) / elapsedPendulumVeclocityTime;
  previousPendulumVelocityPosition = currentPendulumVelocityPosition;

  return velocity;
}

float computeStepperVelocity(long currentStepperVelocityPosition, float elapsedStepperVeclocityTime)
{
  static float previousStepperVelocityPosition = 0.0;

  if (firstStepperVelocityCall)
  {
    firstStepperVelocityCall = false;
    previousStepperVelocityPosition = currentStepperVelocityPosition;
    return 0.0;
  }

  float velocity = (currentStepperVelocityPosition - previousStepperVelocityPosition) / elapsedStepperVeclocityTime;
  previousStepperVelocityPosition = currentStepperVelocityPosition;

  return velocity;
}

float computeLQR(long stepperPosition, float stepperVecloity, double pendulumPosition, float pendulumVelocity)
{
  float output = -(K[0] * stepperPosition + K[1] * stepperVecloity + K[2] * pendulumPosition + K[3] * pendulumVelocity); // u = -Kx
  
  return output;
}

float computePID(double pendulumPosition, float elapsedTime)
{
  static float previousError = 0;
  float error = pendulumTargetPosition - pendulumPosition;

  if (firstPIDcall)
  {
    firstPIDcall = false;
    previousError = error;
  }

  float proportional = error;
  integral += error * elapsedTime;
  float derivative = (error - previousError) / elapsedTime;
  previousError = error;

  float output = Kp * proportional + Ki * integral + Kd * derivative;

  return output;
}

float SwingUpController(double pendulumPosition, float pendulumVelocity)
{
  if (fabs(pendulumPosition) <= 10.0)
  {
    swingUpPosition += sign((float)swingUpPosition) * SWING_UP_STEP;
    swingUpPosition = -swingUpPosition;
  }

  return swingUpPosition;
}

void setup()
{
  Serial.begin(115200);
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);

  stepper.setMaxSpeed(200000);
  stepper.setAcceleration(100000);

  resetPosition();

  pendulumCurrentPosition = convertRawAngleToDegrees();
  swingUpPosition = SWING_UP_START;
}

void loop()
{
  pendulumCurrentPosition = convertRawAngleToDegrees();

  if (state != SWINGING_UP)
  {
    stepper.runSpeed();
  }
  else 
  {
    stepper.run();
  }

  handleInput();

  static unsigned long previousTimeMicro = 0;
  static unsigned long previousControlTimeMicro = 0.0;
  unsigned long currentTimeMicro = micros();
  unsigned long dt = currentTimeMicro - previousTimeMicro;
  float elapsedTimeMicro = dt;
  previousTimeMicro = currentTimeMicro;

  float elapsedTimeSeconds = (float) dt * 1e-6;

  double alpha = alphaFromFrequency(500.0, (double) dt * 1e-6);

  // Find closes upright target
  pendulumTargetPosition = 180.0 * (pendulumCurrentPosition > 0 ? 1.0 : -1.0);

  float balancingMargin = 25.0; // in DEGREES
  bool pendulumNearVertical = fabs(pendulumTargetPosition - pendulumCurrentPosition) <= balancingMargin;

  switch (state)
  {
    case WAITING:
      if (pendulumNearVertical)
      {
        state = BALANCING;
        break;
      }
      stepper.setSpeed(0);

    break;
    
    case BALANCING:
    float pendulumVelocity;
    float stepperVelocity;

    swingUpPosition = SWING_UP_START;

      if (!pendulumNearVertical)
      {
        state = WAITING;

        stepper.setSpeed(0);

        previousError = 0;
        integral = 0;
        firstPIDcall = true;
        break;
      }

      stepperCurrentPosition = stepper.currentPosition();

      pendulumVelocity = computePendulumVelocity(pendulumCurrentPosition, elapsedTimeSeconds); 
      stepperVelocity = computeStepperVelocity(stepperCurrentPosition, elapsedTimeSeconds); // Use last target speed ?

      if ((currentTimeMicro - previousControlTimeMicro) * 1e-6 >= controlPeriod)
      {
        previousControlTimeMicro = currentTimeMicro;

        //float output = computeLQR(stepperCurrentPosition, stepperVelocity, pendulumCurrentPosition - pendulumTargetPosition, pendulumVelocity);
        float output = computePID(pendulumCurrentPosition, elapsedTimeSeconds);

        stepper.setSpeed(output);
      }

      break;

      case SWINGING_UP:
        if (pendulumNearVertical)
        {
          state = BALANCING;
          break;
        }
        if ((currentTimeMicro - previousControlTimeMicro) * 1e-6 >= controlPeriod && stepper.distanceToGo() == 0)
        {
          previousControlTimeMicro = currentTimeMicro;
          
          pendulumVelocity = computePendulumVelocity(pendulumCurrentPosition, elapsedTimeSeconds);

          float output = SwingUpController(pendulumCurrentPosition, pendulumVelocity);

          stepper.moveTo(output);
        }

      break;

  }
}