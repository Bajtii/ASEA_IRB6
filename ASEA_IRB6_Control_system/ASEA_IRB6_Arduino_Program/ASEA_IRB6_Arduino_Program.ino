#include <Arduino.h>
#include <stdlib.h>
#include <math.h>

// ======================================================
// Pin Configuration
// ======================================================
const int X_DIR_PIN = 5;
const int X_PULSE_PIN = 18;

const int Y_DIR_PIN = 4;
const int Y_PULSE_PIN = 2;

const int Z_DIR_PIN = 21;
const int Z_PULSE_PIN = 22;

const int A_DIR_PIN = 25;
const int A_PULSE_PIN = 26;

#define HOME_X_PIN 14
#define HOME_Y_PIN 12
#define HOME_Z_PIN 27
#define HOME_A_PIN 33

#define ENABLE_PIN 32

// ======================================================
// Timing Configuration
// ======================================================
const int pulseDuration = 420;
const int pulseInterval = 420;
const int homePulseInterval = 85;

// ======================================================
// Current Position State
// ======================================================
long currentSteps[4] = {0, 0, 0, 0};

// ======================================================
// Dynamic Trajectory Storage
// Each row: [theta1, theta2, theta3, theta4]
// ======================================================
float (*angles)[4] = nullptr;
size_t capacity = 0;
int count = 0;

// ======================================================
// Function Declarations
// ======================================================
bool addAngles(float t1, float t2, float t3, float t4);

void parseAnglesBlock(String block);
void parseAnglesData(const String &rawData);

void moveAllSteppers(long s1, long s2, long s3, long s4);
void calculateStepsAndMove(float thetas[4], bool isLast);

void selectAxisAndAngle();
void selectAxisAndSteps();

void moveSingleStepper(long steps, int motor);

void homeRobot();
void processAngles();

// ======================================================
// Add Trajectory Point
// ======================================================
bool addAngles(float t1, float t2, float t3, float t4) {

  if (count >= (int)capacity) {

    size_t newCapacity = (capacity == 0) ? 10 : (capacity * 2);

    float (*newPtr)[4] =
        (float(*)[4])realloc(angles, newCapacity * sizeof(*angles));

    if (!newPtr) {
      Serial.println("Memory allocation failed.");
      return false;
    }

    angles = newPtr;
    capacity = newCapacity;
  }

  angles[count][0] = t1;
  angles[count][1] = t2;
  angles[count][2] = t3;
  angles[count][3] = t4;

  count++;

  return true;
}

// ======================================================
// Parse Single Trajectory Block
// Example: {10.0, -20.0, 30.0, 0.0}
// ======================================================
void parseAnglesBlock(String block) {

  block.replace('{', ' ');
  block.replace('}', ' ');

  block.trim();

  if (block.endsWith(",")) {
    block.remove(block.length() - 1);
  }

  if (block.startsWith(",")) {
    block = block.substring(1);
  }

  block.trim();

  float values[4];
  int index = 0;

  while (index < 4 && block.length() > 0) {

    int commaPos = block.indexOf(',');
    String token;

    if (commaPos == -1) {
      token = block;
      block = "";
    } else {
      token = block.substring(0, commaPos);
      block.remove(0, commaPos + 1);
    }

    token.trim();

    if (token.length() > 0) {
      values[index++] = token.toFloat();
    }

    block.trim();
  }

  if (index == 4) {
    addAngles(values[0], values[1], values[2], values[3]);
  } else {
    Serial.println("Invalid trajectory block skipped.");
  }
}

// ======================================================
// Parse Multiple Trajectory Blocks
// ======================================================
void parseAnglesData(const String &rawData) {

  int start = 0;

  while (true) {

    int bracePos = rawData.indexOf('}', start);

    if (bracePos == -1) {
      break;
    }

    String block = rawData.substring(start, bracePos + 1);

    parseAnglesBlock(block);

    start = bracePos + 1;
  }
}

// ======================================================
// Setup
// ======================================================
void setup() {

  Serial.begin(115200);

  while (!Serial) {}

  pinMode(X_DIR_PIN, OUTPUT);
  pinMode(X_PULSE_PIN, OUTPUT);

  pinMode(Y_DIR_PIN, OUTPUT);
  pinMode(Y_PULSE_PIN, OUTPUT);

  pinMode(Z_DIR_PIN, OUTPUT);
  pinMode(Z_PULSE_PIN, OUTPUT);

  pinMode(A_DIR_PIN, OUTPUT);
  pinMode(A_PULSE_PIN, OUTPUT);

  pinMode(HOME_X_PIN, INPUT_PULLUP);
  pinMode(HOME_Y_PIN, INPUT_PULLUP);
  pinMode(HOME_Z_PIN, INPUT_PULLUP);
  pinMode(HOME_A_PIN, INPUT_PULLUP);

  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, HIGH);

  Serial.println("Select option:");
  Serial.println("1: Home Robot");
  Serial.println("2: Select Axis and Angle");
  Serial.println("3: Select Axis and Steps");
  Serial.println("4: Execute Trajectory");
  Serial.println("5: Send Trajectory Angles");
}

// ======================================================
// Main Loop
// ======================================================
void loop() {

  static int menuOption = 0;

  if (Serial.available() > 0) {

    String input = Serial.readStringUntil('\n');

    input.trim();

    if (input.equalsIgnoreCase("back")) {

      menuOption = 0;

      Serial.println("Returned to main menu.");
      Serial.println("Select option:");
      Serial.println("1: Home Robot");
      Serial.println("2: Select Axis and Angle");
      Serial.println("3: Select Axis and Steps");
      Serial.println("4: Execute Trajectory");
      Serial.println("5: Send Trajectory Angles");

      return;
    }

    // ==================================================
    // Main Menu
    // ==================================================
    if (menuOption == 0) {

      if (input == "1") {

        menuOption = 1;

        homeRobot();

        Serial.println("Robot homed.");

        menuOption = 0;
      }

      else if (input == "2") {

        menuOption = 2;

        selectAxisAndAngle();

        menuOption = 0;
      }

      else if (input == "3") {

        menuOption = 3;

        selectAxisAndSteps();

        menuOption = 0;
      }

      else if (input == "4") {

        menuOption = 4;

        if (count <= 0) {
          Serial.println("No trajectory data loaded.");
        } else {
          processAngles();
        }

        menuOption = 0;
      }

      else if (input == "5") {

        menuOption = 5;

        if (angles) {
          free(angles);
          angles = nullptr;
        }

        capacity = 0;
        count = 0;

        Serial.println("Send trajectory data in format:");
        Serial.println("{10.0, -20.0, 30.0, 0.0},");
        Serial.println("{15.0, -25.0, 35.0, 0.0},");
        Serial.println("Type 'back' to return.");
      }

      else {
        Serial.println("Invalid option. Choose 1-5.");
      }
    }

    // ==================================================
    // Receive Trajectory Data
    // ==================================================
    else if (menuOption == 5) {

      int before = count;

      parseAnglesData(input);

      int added = count - before;

      Serial.print("Added: ");
      Serial.print(added);

      Serial.print(" | Total points: ");
      Serial.println(count);

      if (added > 0) {

        int last = count - 1;

        Serial.print("Last point: (");

        Serial.print(angles[last][0], 6);
        Serial.print(", ");

        Serial.print(angles[last][1], 6);
        Serial.print(", ");

        Serial.print(angles[last][2], 6);
        Serial.print(", ");

        Serial.print(angles[last][3], 6);

        Serial.println(")");
      }

      Serial.println("Send next line or type 'back'.");
    }

    else {
      Serial.println("Type 'back' to return to the main menu.");
    }
  }
}

// ======================================================
// Execute Full Trajectory
// ======================================================
void processAngles() {

  if (count <= 0) {
    Serial.println("No trajectory data available.");
    return;
  }

  Serial.println("Starting trajectory...");

  digitalWrite(ENABLE_PIN, LOW);

  for (int i = 0; i < count; i++) {

    float thetas[4] = {
      angles[i][0],
      angles[i][1],
      angles[i][2],
      angles[i][3]
    };

    bool isLast = (i == count - 1);

    calculateStepsAndMove(thetas, isLast);
  }

  Serial.println("Trajectory complete.");
}

// ======================================================
// Homing Procedure
// ======================================================
void homeRobot() {

  Serial.println("Starting homing procedure...");

  while (digitalRead(HOME_Z_PIN) == HIGH) {

    digitalWrite(Z_DIR_PIN, HIGH);

    digitalWrite(Z_PULSE_PIN, HIGH);
    delayMicroseconds(pulseDuration);

    digitalWrite(Z_PULSE_PIN, LOW);
    delayMicroseconds(homePulseInterval);
  }

  Serial.println("Z axis homed.");

  while (digitalRead(HOME_Y_PIN) == HIGH) {

    digitalWrite(Y_DIR_PIN, HIGH);

    digitalWrite(Y_PULSE_PIN, HIGH);
    delayMicroseconds(pulseDuration);

    digitalWrite(Y_PULSE_PIN, LOW);
    delayMicroseconds(homePulseInterval);
  }

  Serial.println("Y axis homed.");

  while (digitalRead(HOME_X_PIN) == HIGH) {

    digitalWrite(X_DIR_PIN, HIGH);

    digitalWrite(X_PULSE_PIN, HIGH);
    delayMicroseconds(pulseDuration);

    digitalWrite(X_PULSE_PIN, LOW);
    delayMicroseconds(homePulseInterval);
  }

  Serial.println("X axis homed.");

  while (digitalRead(HOME_A_PIN) == HIGH) {

    digitalWrite(A_DIR_PIN, HIGH);

    digitalWrite(A_PULSE_PIN, HIGH);
    delayMicroseconds(pulseDuration);

    digitalWrite(A_PULSE_PIN, LOW);
    delayMicroseconds(homePulseInterval);
  }

  Serial.println("A axis homed.");

  long stepsX = 3050;
  long stepsY = -20492;
  long stepsZ = -11375;
  long stepsA = -63500;

  moveAllSteppers(stepsX, stepsY, stepsZ, stepsA);

  currentSteps[0] = 0;
  currentSteps[1] = 0;
  currentSteps[2] = 0;
  currentSteps[3] = 0;

  Serial.println("Robot homing complete.");
}

// ======================================================
// Move All Axes Simultaneously
// ======================================================
void moveAllSteppers(long s1, long s2, long s3, long s4) {

  digitalWrite(X_DIR_PIN, (s1 >= 0) ? HIGH : LOW);
  digitalWrite(Y_DIR_PIN, (s2 >= 0) ? HIGH : LOW);
  digitalWrite(Z_DIR_PIN, (s3 >= 0) ? HIGH : LOW);
  digitalWrite(A_DIR_PIN, (s4 >= 0) ? HIGH : LOW);

  long a1 = labs(s1);
  long a2 = labs(s2);
  long a3 = labs(s3);
  long a4 = labs(s4);

  long maxSteps = max(max(a1, a2), max(a3, a4));

  for (long i = 0; i < maxSteps; i++) {

    if (i < a1) {
      digitalWrite(X_PULSE_PIN, HIGH);
      delayMicroseconds(pulseDuration);
      digitalWrite(X_PULSE_PIN, LOW);
    }

    if (i < a2) {
      digitalWrite(Y_PULSE_PIN, HIGH);
      delayMicroseconds(pulseDuration);
      digitalWrite(Y_PULSE_PIN, LOW);
    }

    if (i < a3) {
      digitalWrite(Z_PULSE_PIN, HIGH);
      delayMicroseconds(pulseDuration);
      digitalWrite(Z_PULSE_PIN, LOW);
    }

    if (i < a4) {
      digitalWrite(A_PULSE_PIN, HIGH);
      delayMicroseconds(pulseDuration);
      digitalWrite(A_PULSE_PIN, LOW);
    }

    delayMicroseconds(pulseInterval);
  }

  digitalWrite(X_DIR_PIN, LOW);
  digitalWrite(Y_DIR_PIN, LOW);
  digitalWrite(Z_DIR_PIN, LOW);
  digitalWrite(A_DIR_PIN, LOW);
}

// ======================================================
// Convert Angles to Steps and Execute Motion
// ======================================================
void calculateStepsAndMove(float thetas[4], bool isLast) {

  long target[4] = {

    lroundf(-thetas[0] * 452.548f),
    lroundf( thetas[1] * 463.532f),
    lroundf( thetas[2] * 490.778f),
    lroundf( thetas[3] * 375.809f)
  };

  if (target[1] != 0) {
    target[1] = -target[1];
  }

  long delta[4];

  for (int i = 0; i < 4; i++) {
    delta[i] = target[i] - currentSteps[i];
  }

  moveAllSteppers(delta[0], delta[1], delta[2], delta[3]);

  for (int i = 0; i < 4; i++) {
    currentSteps[i] = target[i];
  }

  Serial.println("Angles and step deltas:");

  for (int i = 0; i < 4; i++) {

    Serial.print("Theta");
    Serial.print(i + 1);

    Serial.print(": ");
    Serial.print(thetas[i], 4);

    Serial.print(" deg | deltaSteps: ");

    Serial.println(labs(delta[i]));
  }

  // Return to origin after last trajectory point
  if (isLast) {

    digitalWrite(ENABLE_PIN, HIGH);

    float reversed[4] = {
      -thetas[0],
      -thetas[1],
      -thetas[2],
      -thetas[3]
    };

    long ret[4] = {

      lroundf(-reversed[0] * 452.548f),
      lroundf( reversed[1] * 463.532f),
      lroundf( reversed[2] * 490.778f),
      lroundf( reversed[3] * 375.809f)
    };

    if (ret[1] != 0) {
      ret[1] = -ret[1];
    }

    moveAllSteppers(ret[0], ret[1], ret[2], ret[3]);

    for (int i = 0; i < 4; i++) {
      currentSteps[i] = 0;
    }

    Serial.println("Return motion complete. Position reset.");
  }
}

// ======================================================
// Manual Axis Control by Angle
// ======================================================
void selectAxisAndAngle() {

  Serial.println("Select axis (1-4):");
  Serial.println("1=Theta1 2=Theta2 3=Theta3 4=Theta4");

  while (true) {

    if (Serial.available() > 0) {

      String input = Serial.readStringUntil('\n');

      input.trim();

      int axis = input.toInt();

      if (axis >= 1 && axis <= 4) {

        Serial.println("Enter angle:");

        while (true) {

          if (Serial.available() > 0) {

            String angleInput = Serial.readStringUntil('\n');

            angleInput.trim();

            float angle = angleInput.toFloat();

            long steps = lroundf(
              angle * (
                axis == 1 ? 451.67f :
                axis == 2 ? 968.31f :
                axis == 3 ? 491.935f :
                            354.61f
              )
            );

            if (axis == 1) steps = -steps;
            if (axis == 2) steps = -steps;

            moveSingleStepper(steps, axis);

            Serial.println("Motion complete.");

            break;
          }
        }

        break;
      }

      else {
        Serial.println("Invalid axis. Choose 1-4.");
      }
    }
  }
}

// ======================================================
// Manual Axis Control by Step Count
// ======================================================
void selectAxisAndSteps() {

  Serial.println("Select axis (1-4):");
  Serial.println("1=Theta1 2=Theta2 3=Theta3 4=Theta4");

  while (true) {

    if (Serial.available() > 0) {

      String input = Serial.readStringUntil('\n');

      input.trim();

      int axis = input.toInt();

      if (axis >= 1 && axis <= 4) {

        Serial.println("Enter step count:");

        while (true) {

          if (Serial.available() > 0) {

            String stepInput = Serial.readStringUntil('\n');

            stepInput.trim();

            long steps = stepInput.toInt();

            if (axis == 1) steps = -steps;
            if (axis == 2) steps = -steps;

            moveSingleStepper(steps, axis);

            Serial.println("Motion complete.");

            break;
          }
        }

        break;
      }

      else {
        Serial.println("Invalid axis. Choose 1-4.");
      }
    }
  }
}

// ======================================================
// Single Stepper Motor Movement
// ======================================================
void moveSingleStepper(long steps, int motor) {

  Serial.print("Motor ");
  Serial.print(motor);

  Serial.print(": ");

  Serial.println(steps);

  switch (motor) {

    case 1:
      digitalWrite(X_DIR_PIN, steps >= 0 ? HIGH : LOW);
      break;

    case 2:
      digitalWrite(Y_DIR_PIN, steps >= 0 ? HIGH : LOW);
      break;

    case 3:
      digitalWrite(Z_DIR_PIN, steps >= 0 ? HIGH : LOW);
      break;

    case 4:
      digitalWrite(A_DIR_PIN, steps >= 0 ? HIGH : LOW);
      break;
  }

  long n = labs(steps);

  for (long i = 0; i < n; i++) {

    switch (motor) {

      case 1:
        digitalWrite(X_PULSE_PIN, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(X_PULSE_PIN, LOW);
        break;

      case 2:
        digitalWrite(Y_PULSE_PIN, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(Y_PULSE_PIN, LOW);
        break;

      case 3:
        digitalWrite(Z_PULSE_PIN, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(Z_PULSE_PIN, LOW);
        break;

      case 4:
        digitalWrite(A_PULSE_PIN, HIGH);
        delayMicroseconds(pulseDuration);
        digitalWrite(A_PULSE_PIN, LOW);
        break;
    }

    delayMicroseconds(pulseInterval);
  }

  switch (motor) {

    case 1:
      digitalWrite(X_DIR_PIN, LOW);
      break;

    case 2:
      digitalWrite(Y_DIR_PIN, LOW);
      break;

    case 3:
      digitalWrite(Z_DIR_PIN, LOW);
      break;

    case 4:
      digitalWrite(A_DIR_PIN, LOW);
      break;
  }
}