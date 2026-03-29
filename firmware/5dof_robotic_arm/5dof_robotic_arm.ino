#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  125
#define SERVOMAX  575
#define SERVO_FREQ 60

// Speed control (milliseconds per degree)
int SERVO_SPEED = 15;  // Default: 15ms/deg (lower = faster)

// Store current angles
int currentAngles[5] = {80, 70, 50, 125, 0};  // Rest position on power-up (prevents struggling)
const char* jointNames[] = {"Base", "Shoulder", "Elbow", "Wrist", "Gripper"};

void setup() {
  Serial.begin(115200);
  Serial.println("================================================");
  Serial.println("   5-DOF Robotic Arm Controller");
  Serial.println("   Format: servo angle (e.g., '0 90')");
  Serial.println("   Speed: 99 speed (e.g., '99 15') - 5 to 100");
  Serial.println("   Current Speed: " + String(SERVO_SPEED) + "ms/deg");
  Serial.println("================================================");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);

  // CRITICAL: Initialize servos to REST POSITION on power-up (prevents struggling)
  Serial.println("Initializing to rest position [80, 70, 50, 125, 0]...");
  for(int i=0; i<5; i++) {
    moveServo(i, currentAngles[i]);
    delay(50);  // Small delay between servos to prevent power spike
  }
  delay(500);
  Serial.println("✓ Initialized to rest position");
  displayPositions();
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line.length() == 0) return;

    // Check for multi-move command: "M a1 a2 a3 a4 a5"
    if (line.startsWith("M ")) {
      handleMultiMove(line.substring(2));
      return;
    }

    int spaceIndex = line.indexOf(' ');
    if (spaceIndex == -1) return;

    int servoNum = line.substring(0, spaceIndex).toInt();
    int inputVal = line.substring(spaceIndex + 1).toInt();

    // Check for speed command (servoNum 99 = set speed)
    if (servoNum == 99) {
      SERVO_SPEED = constrain(inputVal, 5, 100);
      Serial.print("Speed set to: ");
      Serial.print(SERVO_SPEED);
      Serial.println("ms/deg");
      return;
    }

    if (servoNum >= 0 && servoNum <= 4) {
      int targetAngle;

      // Negative values are relative (e.g., -10 means move back 10 degrees)
      // Positive values are absolute positions
      if (inputVal < 0) {
        targetAngle = currentAngles[servoNum] + inputVal;
      } else {
        targetAngle = inputVal;  // Absolute position
      }

      // Constrain to physical limits (0-180)
      targetAngle = constrain(targetAngle, 0, 180);

      // Move single servo (for individual commands)
      slowMoveSingle(servoNum, targetAngle);

      Serial.print("Servo ");
      Serial.print(servoNum);
      Serial.print(" moved to: ");
      Serial.println(targetAngle);

      displayPositions();
    }
  }
}

// Handle multi-move command: "a1 a2 a3 a4 a5"
void handleMultiMove(String angles) {
  int targets[5];
  int idx = 0;
  int lastPos = 0;

  // Parse 5 angles from space-separated string
  for (int i = 0; i < 5 && idx < angles.length(); i++) {
    int spacePos = angles.indexOf(' ', lastPos);
    if (spacePos == -1) spacePos = angles.length();

    String part = angles.substring(lastPos, spacePos);
    targets[i] = part.toInt();
    targets[i] = constrain(targets[i], 0, 180);

    lastPos = spacePos + 1;
    idx = lastPos;
  }

  Serial.println("Moving to: " + String(targets[0]) + " " + String(targets[1]) +
                 " " + String(targets[2]) + " " + String(targets[3]) +
                 " " + String(targets[4]));

  // Move all servos simultaneously
  slowMoveMulti(targets);

  displayPositions();
}

void moveServo(int num, int angle) {
  currentAngles[num] = angle;
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulse);
}

// Move single servo slowly (for individual commands)
void slowMoveSingle(int num, int target) {
  int startAngle = currentAngles[num];

  if (target > startAngle) {
    for (int pos = startAngle; pos <= target; pos++) {
      writeToPCA(num, pos);
      delay(SERVO_SPEED);
    }
  } else {
    for (int pos = startAngle; pos >= target; pos--) {
      writeToPCA(num, pos);
      delay(SERVO_SPEED);
    }
  }
  currentAngles[num] = target;
}

// Move multiple servos simultaneously
void slowMoveMulti(int targets[5]) {
  int starts[5];
  int diffs[5];
  int maxSteps = 0;

  // Store start positions and calculate differences
  for (int i = 0; i < 5; i++) {
    starts[i] = currentAngles[i];
    diffs[i] = targets[i] - starts[i];
    int steps = abs(diffs[i]);
    if (steps > maxSteps) maxSteps = steps;
  }

  // If no movement needed, return
  if (maxSteps == 0) return;

  // Move all servos together
  for (int step = 0; step <= maxSteps; step++) {
    for (int i = 0; i < 5; i++) {
      if (diffs[i] != 0) {
        // Calculate current position for this servo
        float progress = (float)step / maxSteps;
        int pos = starts[i] + (int)(diffs[i] * progress);
        writeToPCA(i, pos);
      }
    }
    delay(SERVO_SPEED);
  }

  // Update stored angles
  for (int i = 0; i < 5; i++) {
    currentAngles[i] = targets[i];
  }
}

// Low-level write to PCA9685
void writeToPCA(int num, int angle) {
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(num, 0, pulse);
}

void displayPositions() {
  Serial.println("--- Current Positions ---");
  for (int i = 0; i < 5; i++) {
    Serial.print(jointNames[i]);
    Serial.print(": ");
    Serial.print(currentAngles[i]);
    Serial.println(" deg");
  }
}
