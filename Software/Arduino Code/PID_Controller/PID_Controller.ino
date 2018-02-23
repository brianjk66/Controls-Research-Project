#include "Servo.h"
#include "MsTimer2.h"

/* Default Tuning Variables */
#define P_GAIN          (3)     // Proportional gain
#define I_GAIN          (1.5)   // Integral gain
#define D_GAIN          (0.3)   // Derivative gain
#define MIN_I_TERM      (-250)  // Minimum Contribution of iTerm in PI controller
#define MAX_I_TERM      (250)   // Maximum Contribution of iTerm in PI controller
#define COMMAND         (0)     // Commanded/Requested pitch (in degrees from horizontal)
#define FREQUENCY       (100)   // Refresh rate of controlller (in Hz)

/* Hardware Restrictions */
#define MIN_ANGLE       (-60)
#define MAX_ANGLE       (30)
#define MAX_FREQ        (1000) // Maximum refresh rate (in Hz)

/* Pin Numbers */
#define ESC_PIN         (6)   // PWM pin for signaling ESC
#define DRIVE_PIN       (10)  // Drive pin for power MOSFET
#define SENSOR_PIN      (A0)    // Pin for reading sensor values

/* See this link for more info:
   https://www.embedded.com/design/prototyping-and-development/4211211/PID-without-a-PhD
*/
typedef struct PID_t {
  double        input;    // Input to controller (requested value)
  double        Ki;       // Integral gain
  double        Kp;       // Proportional gain
  double        Kd;       // Derivative gain
  double        dt;       // Period between updates (in seconds)
  double        old_error;// Last error value
  double        iState;   // Integrator state (accumulated error)
} PID;

volatile PID controller; // PID controller for the system
volatile double pitch;   // Measured pitch
Servo ESC;               // ESC to drive motor
volatile int drive;      // Drive signal value fed to ESC

// ================================================================
// ===                    HELPER FUNCTIONS                      ===
// ================================================================

/* Update PID output signal using current system state */
void updatePID() {
  // P, I, & D terms
  double pTerm, iTerm, dTerm;

  // Measure rotary sensor value and filter with 10 point averager
  double runningSum = 0;
  for (int i = 0; i < 10; i++)
    runningSum -= (0.3656 * analogRead(SENSOR_PIN)) - 185.64;
  //pitch = runningSum / 100;

  // Round to nearest hundredth
  pitch = round(10 * runningSum) / 100.0;
  
  // Controller error is difference between input and current state
  double error = controller.input - pitch;

  // Calculate the proportional term
  pTerm = controller.Kp * error;

  // Calculate the integral state with appropriate min/max constraints
  // TODO: Look into anti-windup code
  controller.iState += error * controller.dt;
  controller.iState = constrain(controller.iState, MIN_I_TERM/controller.Ki, MAX_I_TERM/controller.Ki);

  // Calculate the integral term
  iTerm  = controller.Ki * controller.iState;

  // Calculate the derivative term
  dTerm  = controller.Kd * ((error - controller.old_error)/controller.dt);

  // Update the dState of the controller
  controller.old_error = error;

  // Add PID terms to get new drive signal (0-1000 scale)
  drive = pTerm + iTerm + dTerm;

  // Send new drive signal to ESC
  setSpeed(&ESC, drive);
}

/* Arm ESC for first use upon startup */
void arm(Servo *ESC) {
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms for ESC to power down
  setSpeed(ESC, 0);               // Set speed to 0
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(500);                     // 500ms delay for ESC to respond
}

/* Calibrate ESC's PWM range for first use */
void calibrate(Servo *ESC) {
  digitalWrite(DRIVE_PIN, LOW);   // Disconnect ESC from power
  delay(500);                     // Wait 500ms
  setSpeed(ESC, 1000);            // Request full speed
  digitalWrite(DRIVE_PIN, HIGH);  // Reconnect ESC to power
  delay(5000);                    // Wait 5s
  setSpeed(ESC, 0);               // Request 0 speed
}

/* Drive ESC with 0-1000 drive signal */
void setSpeed(Servo *ESC, int drive) {
  // Scale drive signal to ESC's range of accepted values
  int us = map(drive, 0, 1000, 1000, 2000); //Scale drive signal to ESC's accepted range
  ESC->writeMicroseconds(us);
}

/* Change PID tuning parameters via Serial interface */
void tuneController(volatile PID *pid) {
  // Reset controller state
  pid->iState = 0;    // Reset the integrator state
  pid->old_error = 0; // Reset the derivative state

  Serial.print("Set Proportional Gain (Current Value: ");
  Serial.print(pid->Kp);
  Serial.print(")\n"); 
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->Kp = Serial.parseFloat();                // Set new proportial gain

  Serial.print("Set Integrator Gain (Current Value: ");
  Serial.print(pid->Ki);
  Serial.print(")\n");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->Ki = Serial.parseFloat();                // Set new integral gain

  Serial.print("Set Derivative Gain (Current Value: ");
  Serial.print(pid->Kd);
  Serial.print(")\n");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  pid->Kd = Serial.parseFloat();                // Set new derivative gain

  // wait for ready
  Serial.print("New values set. Send any character to resume...\n");
  while (Serial.available() && Serial.read());  // empty buffer
  while (!Serial.available());                  // wait for data
  while (Serial.available() && Serial.read());  // empty buffer again
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Attach ESC
  ESC.attach(ESC_PIN);

  // Configure I/O pins
  pinMode(DRIVE_PIN, OUTPUT);
  digitalWrite(DRIVE_PIN, LOW);

  // Setup initial PID controller values
  controller.input      = COMMAND;
  controller.Kp         = P_GAIN;
  controller.Ki         = I_GAIN;
  controller.Kd         = D_GAIN;
  controller.dt         = 1.0 / FREQUENCY; // period = 1/frequency
  controller.iState     = 0;
  controller.old_error  = 0;
    
  Serial.print("Send any character to calibrate ESC or 's' to skip...\n");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  if (Serial.read() == 's') {
    // Only arm ESC if user opted to skip calibration
    Serial.print("Arming ESC... ");
    arm(&ESC);
    Serial.print("Arming complete\n");
  } else {
    // Otherwise, perform full calibration
    Serial.print("Calibrating ESC... ");
    calibrate(&ESC);
    Serial.print("Calibration complete\n");
  }

  // Wait for ready
  Serial.print("\nSend any character to begin...\n");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // Attach ISR for timer interrupt
  MsTimer2::set(1000/FREQUENCY, updatePID); // 1,000/frequency = period (in milliseconds)
  MsTimer2::start();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // See if user wants to tune controller or change commanded angle
  if (Serial.available()) {
    // 'p' for pause
    if (Serial.peek() == 'p') {
      MsTimer2::stop();     // Disable interrupts
      Serial.read();                // Flush buffer
      setSpeed(&ESC, 0);            // Kill power to the motor(s)

      // Reset controller state
      controller.iState = 0;    // Reset the integrator state
      controller.old_error = 0; // Reset the derivative state
      
      // wait for ready
      Serial.print("Send any character to resume...\n");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // 't' for tune
    else if (Serial.peek() == 't') {
      MsTimer2::stop();     // Disable interrupts
      Serial.read();                // Flush buffer
      setSpeed(&ESC, 0);            // Kill power to the motor(s)

      // Call tuning subroutine
      tuneController(&controller);

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // 'c' for calibrate
    else if (Serial.peek() == 'c') {
      MsTimer2::stop();     // Disable interrupts
      Serial.read();                // Flush buffer
      setSpeed(&ESC, 0);            // Kill power to the motor(s)

      // Reset controller state
      controller.iState = 0;    // Reset the integrator state
      controller.old_error = 0; // Reset the derivative state

      // Call calibration subroutine
      Serial.print("Calibrating ESC... ");
      calibrate(&ESC);
      Serial.print("Calibration complete\n");

      // wait for ready
      Serial.print("Send any character to resume...\n");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // 'f' for frequency
    else if (Serial.peek() == 'f') {
      MsTimer2::stop();     // Disable interrupts
      Serial.read();                // Flush buffer
      setSpeed(&ESC, 0);            // Kill power to the motor(s)

      // Reset controller state
      controller.iState = 0;    // Reset the integrator state
      controller.old_error = 0; // Reset the derivative state

      // Ask user to set new refresh rate
      Serial.print("Set new refresh rate in Hz (1-");
      Serial.print(MAX_FREQ);
      Serial.print(")\n");
      while (Serial.available() && Serial.read());  // empty buffer
      while (!Serial.available());                  // wait for data
      int newFreq = Serial.parseInt();              // Read user input
      
      // Check for valid new frequency
      if (newFreq > 0 && newFreq <= MAX_FREQ) {
        controller.dt = 1.0 / newFreq;        // Set new controller dt
        MsTimer2::set(1000 / newFreq, updatePID);  // Set new interrupt period (in milliseconds)
      }
      
      // wait for ready
      Serial.print("Send any character to resume...\n");
      while (Serial.available() && Serial.read()); // empty buffer
      while (!Serial.available());                 // wait for data
      while (Serial.available() && Serial.read()); // empty buffer again

      // Re-enable interrupts and continue
      MsTimer2::start();
    }
    // Otherwise, treat input as new pitch request
    else {
      // See if user sent new pitch request
      double newAngle = Serial.parseFloat();

      // If new angle is within acceptable range, update input angle
      if (newAngle >= MIN_ANGLE && newAngle <= MAX_ANGLE) {
        controller.input = newAngle;
      }
    }
  }

  // Print pitch and drive info to serial
  Serial.print("Pitch:\t");
  Serial.print(pitch, 2);
  Serial.print("\tDrive:\t");
  Serial.println(drive);
}
