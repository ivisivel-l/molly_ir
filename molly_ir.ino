
/* Molly the turtuise
IR controlled version - DIAGNOSTIC VERSION
by Lev Kunin lev.y.kunin@gmail.com

IR Remote Control Features:

Key 1: Toggle robot on/off
Key 2: Start continuous walking
Key 3: Stop robot
Arrow keys: Manual control of individual limbs

UP: Left arm step
DOWN: Right arm step
LEFT: Turn left
RIGHT: Right leg step

OK: Manual left leg step
*/

#include <DIYables_IRcontroller.h>
#include <Servo.h>
#define IR_RECEIVER_PIN 14 // The Arduino pin connected to IR controller

DIYables_IRcontroller_17 irController(IR_RECEIVER_PIN, 200); // debounce time is 200ms
Servo right_elbow;
Servo left_elbow;
Servo left_sholder;  
Servo right_sholder;
Servo left_leg;
Servo right_leg;
Servo right_foot;
Servo left_foot;

int default_delay_ms = 2;
const int trigPin = 2;
const int echoPin = 3;

const bool navigate = 1;

// Non-blocking timing variables
unsigned long previousMillis = 0;
unsigned long servoMoveTime = 0;
unsigned long lastDiagnosticTime = 0;
bool robotEnabled = true;

// State machine variables
enum RobotState {
  IDLE,
  LEFT_ARM_STEP,
  RIGHT_ARM_STEP,
  LEFT_LEG_STEP,
  RIGHT_LEG_STEP,
  TURNING_RIGHT,
  TURNING_LEFT
};

enum ServoMoveState {
  SERVO_IDLE,
  SERVO_MOVING
};

RobotState currentState = IDLE;
ServoMoveState servoState = SERVO_IDLE;

// Walking sequence variables
int walkCounter = 0;
int stepSequence = 0;

// Servo movement variables
Servo* currentServo = nullptr;
int targetAngle = 0;
int currentAngle = 0;
int stepDirection = 0;
int moveDelay = default_delay_ms;

// Step state machines for each limb
int leftArmStepState = 0;
int rightArmStepState = 0;
int leftLegStepState = 0;
int rightLegStepState = 0;
int turnRightStepState = 0;
int turnLeftStepState = 0;
int turnLeftLegStepState = 0;
int turnRightArmStepState = 0;
int turnRightLegStepState = 0;
int turnLeftArmStepState = 0;

// Function declarations
void startServoMove(Servo* servo, int angle, int delayMs = default_delay_ms);
bool left_arm_step_forward();
bool right_arm_step_forward();
bool left_leg_step_forward();
bool right_leg_step_forward();
bool turn_right();
bool turn_left();
bool turn_left_leg_step();
bool turn_right_arm_step();
bool turn_right_leg_step();
bool turn_left_arm_step();
void do_what_you_are_told_on_ir();
void handleServoMovement(unsigned long currentMillis);
void handleRobotStateMachine(unsigned long currentMillis);
float get_distance();
bool is_too_close();
void initializeServos();
void diagnosticPrint(unsigned long currentMillis);

void setup() {   
  // distance sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // IR receiver setup
  Serial.begin(9600);
  
  // Wait a moment for serial to initialize
  delay(1000);
  
  Serial.println("=== MOLLY ROBOT DIAGNOSTIC MODE ===");
  Serial.print("IR Receiver Pin: ");
  Serial.println(IR_RECEIVER_PIN);
  
  // Test if pin is properly configured
  pinMode(IR_RECEIVER_PIN, INPUT);
  Serial.print("IR Pin state after pinMode: ");
  Serial.println(digitalRead(IR_RECEIVER_PIN));
  
  // Initialize IR controller
  Serial.println("Initializing IR controller...");
  irController.begin();

  Serial.println("IR controller initialized.");
  
  // attach motors
  right_foot.attach(7);
  left_foot.attach(5);
  left_sholder.attach(10);
  right_sholder.attach(9);
  left_elbow.attach(8);
  right_elbow.attach(11);
  left_leg.attach(4);
  right_leg.attach(6);  

  // Initialize all motors to 90 degrees with non-blocking setup
  initializeServos();
  
  Serial.println("Robot initialized. Use IR remote to control.");
  Serial.println("=== DIAGNOSTIC INFO WILL PRINT EVERY 5 SECONDS ===");
  Serial.println("Expected IR commands:");
  Serial.println("Key 1: Toggle robot on/off");
  Serial.println("Key 2: Start walking");
  Serial.println("Key 3: Stop robot");
  Serial.println("Arrow keys: Manual limb control");
  Serial.println("====================================");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Print diagnostic info every 5 seconds
  diagnosticPrint(currentMillis);
  
  // Always check for IR commands
  do_what_you_are_told_on_ir();
  
  // Only run robot logic if enabled
  if (robotEnabled) {
    // Handle servo movements
    handleServoMovement(currentMillis);
    
    // Handle main robot state machine
    handleRobotStateMachine(currentMillis);
  }
}

void diagnosticPrint(unsigned long currentMillis) {
  if (currentMillis - lastDiagnosticTime >= 5000) {
    lastDiagnosticTime = currentMillis;
    Serial.println("--- DIAGNOSTIC INFO ---");
    Serial.print("Robot State: ");
    switch(currentState) {
      case IDLE: Serial.println("IDLE"); break;
      case LEFT_ARM_STEP: Serial.println("LEFT_ARM_STEP"); break;
      case RIGHT_ARM_STEP: Serial.println("RIGHT_ARM_STEP"); break;
      case LEFT_LEG_STEP: Serial.println("LEFT_LEG_STEP"); break;
      case RIGHT_LEG_STEP: Serial.println("RIGHT_LEG_STEP"); break;
      case TURNING_RIGHT: Serial.println("TURNING_RIGHT"); break;
      case TURNING_LEFT: Serial.println("TURNING_LEFT"); break;
    }
    Serial.print("Robot Enabled: ");
    Serial.println(robotEnabled ? "YES" : "NO");
    Serial.print("IR Pin Reading: ");
    Serial.println(digitalRead(IR_RECEIVER_PIN));
    Serial.print("Distance: ");
    Serial.print(get_distance());
    Serial.println(" cm");
    Serial.println("Waiting for IR commands...");
    Serial.println("----------------------");
  }
  return false;
}

bool turn_left_arm_step() {
  switch (turnLeftArmStepState) {
    case 0:
      startServoMove(&left_elbow, 0);
      turnLeftArmStepState = 1;
      return false;
    case 1:
      startServoMove(&left_sholder, 180);
      turnLeftArmStepState = 2;
      return false;
    case 2:
      startServoMove(&left_elbow, 90);
      turnLeftArmStepState = 3;
      return false;
    case 3:
      startServoMove(&left_sholder, 90);
      turnLeftArmStepState = 0;
      return true;
  }
  return false;
}

void do_what_you_are_told_on_ir() {
  Key17 key = irController.getKey();
  
  // DIAGNOSTIC: Print any key detected, even NONE
  static unsigned long lastKeyCheck = 0;
  static int keyCheckCount = 0;
  keyCheckCount++;
  
  // Every 1000 checks, print that we're checking
  if (keyCheckCount >= 1000) {
    Serial.print(".");  // Print dot to show we're checking IR
    keyCheckCount = 0;
  }
  
  if (key != Key17::NONE) {
    Serial.println();  // New line after dots
    Serial.print("IR KEY DETECTED: ");
    Serial.println((int)key);
    
    switch (key) {
      case Key17::KEY_1:
        robotEnabled = !robotEnabled;
        Serial.println(robotEnabled ? "Robot ENABLED" : "Robot DISABLED");
        if (!robotEnabled) {
          currentState = IDLE;
          servoState = SERVO_IDLE;
        } else {
          // Robot is being enabled - initialize servos to default positions
          initializeServos();
          Serial.println("Robot ready for commands - servos initialized");
        }
        break;
      case Key17::KEY_2:
        if (robotEnabled) {
          if (currentState == IDLE) {
            Serial.println("Starting walk sequence");
            currentState = LEFT_ARM_STEP;
            stepSequence = 0;
            leftArmStepState = 0;
          } else {
            Serial.println("Robot is already walking");
          }
        }
        break;
      case Key17::KEY_3:
        Serial.println("Stopping robot");
        currentState = IDLE;
        servoState = SERVO_IDLE;
        break;
      case Key17::KEY_UP:
        Serial.println("Manual left arm step");
        if (currentState == IDLE) {
          currentState = LEFT_ARM_STEP;
          leftArmStepState = 0;
        }
        break;
      case Key17::KEY_DOWN:
        Serial.println("Manual right arm step");
        if (currentState == IDLE) {
          currentState = RIGHT_ARM_STEP;
          rightArmStepState = 0;
        }
        break;
      case Key17::KEY_OK:
        Serial.println("Manual left leg step");
        if (currentState == IDLE) {
          currentState = LEFT_LEG_STEP;
          leftLegStepState = 0;
        }
        break;
      case Key17::KEY_RIGHT:
        Serial.println("Manual right leg step");
        if (currentState == IDLE) {
          currentState = RIGHT_LEG_STEP;
          rightLegStepState = 0;
        }
        break;
      case Key17::KEY_LEFT:
        Serial.println("Turn LEFT command");
        if (robotEnabled && currentState == IDLE) {
          currentState = TURNING_LEFT;
          turnLeftStepState = 0;
          turnRightLegStepState = 0;
          turnLeftArmStepState = 0;
        }
        break;

      default:
        Serial.print("Unknown key: ");
        Serial.println((int)key);
        break;
    }
  }
}

void handleServoMovement(unsigned long currentMillis) {
  if (servoState == SERVO_MOVING && currentServo != nullptr) {
    if (currentMillis - servoMoveTime >= moveDelay) {
      servoMoveTime = currentMillis;
      
      currentAngle += stepDirection;
      currentServo->write(currentAngle);
      
      // Check if we've reached the target
      if ((stepDirection > 0 && currentAngle >= targetAngle) ||
          (stepDirection < 0 && currentAngle <= targetAngle)) {
        servoState = SERVO_IDLE;
        currentServo = nullptr;
      }
    }
  }
}

void handleRobotStateMachine(unsigned long currentMillis) {
  switch (currentState) {
    case IDLE:
      // Do nothing, wait for commands
      break;
      
    case LEFT_ARM_STEP:
      if (servoState == SERVO_IDLE) {
        if (navigate && is_too_close()) {
          currentState = TURNING_RIGHT;
          turnRightStepState = 0;
          turnLeftLegStepState = 0;
          turnRightArmStepState = 0;
          break;
        }
        
        if (left_arm_step_forward()) {
          // Step completed, move to next in sequence or next step
          if (stepSequence >= 0) {
            currentState = RIGHT_ARM_STEP;
            rightArmStepState = 0;
            stepSequence = 1;
          } else {
            currentState = IDLE;
          }
        }
      }
      break;
      
    case RIGHT_ARM_STEP:
      if (servoState == SERVO_IDLE) {
        if (navigate && is_too_close()) {
          currentState = TURNING_RIGHT;
          turnRightStepState = 0;
          turnLeftLegStepState = 0;
          turnRightArmStepState = 0;
          break;
        }
        
        if (right_arm_step_forward()) {
          if (stepSequence >= 0) {
            currentState = LEFT_LEG_STEP;
            leftLegStepState = 0;
            stepSequence = 2;
          } else {
            currentState = IDLE;
          }
        }
      }
      break;
      
    case LEFT_LEG_STEP:
      if (servoState == SERVO_IDLE) {
        if (navigate && is_too_close()) {
          currentState = TURNING_RIGHT;
          turnRightStepState = 0;
          turnLeftLegStepState = 0;
          turnRightArmStepState = 0;
          break;
        }
        
        if (left_leg_step_forward()) {
          if (stepSequence >= 0) {
            currentState = RIGHT_LEG_STEP;
            rightLegStepState = 0;
            stepSequence = 3;
          } else {
            currentState = IDLE;
          }
        }
      }
      break;
      
    case RIGHT_LEG_STEP:
      if (servoState == SERVO_IDLE) {
        if (navigate && is_too_close()) {
          currentState = TURNING_RIGHT;
          turnRightStepState = 0;
          turnLeftLegStepState = 0;
          turnRightArmStepState = 0;
          break;
        }
        
        if (right_leg_step_forward()) {
          if (stepSequence >= 0) {
            // Continue walking - restart sequence
            currentState = LEFT_ARM_STEP;
            leftArmStepState = 0;
            stepSequence = 0;
            walkCounter++;
          } else {
            currentState = IDLE;
          }
        }
      }
      break;
      
    case TURNING_RIGHT:
      if (servoState == SERVO_IDLE) {
        if (turn_right()) {
          // Turn completed, check if still too close
          if (navigate && is_too_close()) {
            // Still too close, keep turning
            turnRightStepState = 0;
            turnLeftLegStepState = 0;
            turnRightArmStepState = 0;
          } else {
            currentState = LEFT_ARM_STEP;
            leftArmStepState = 0;
          }
        }
      }
      break;
      
    case TURNING_LEFT:
      if (servoState == SERVO_IDLE) {
        if (turn_left()) {
          // Turn completed, return to idle or continue walking
          currentState = LEFT_ARM_STEP;
          leftArmStepState = 0;
        }
      }
      break;
  }
}

bool left_arm_step_forward() {
  switch (leftArmStepState) {
    case 0:
      startServoMove(&left_elbow, 0);
      leftArmStepState = 1;
      return false;
    case 1:
      startServoMove(&left_sholder, 180);
      leftArmStepState = 2;
      return false;
    case 2:
      startServoMove(&left_elbow, 90);
      leftArmStepState = 3;
      return false;
    case 3:
      startServoMove(&left_sholder, 90);
      leftArmStepState = 0;
      return true;
  }
  return false;
}

bool right_arm_step_forward() {
  switch (rightArmStepState) {
    case 0:
      startServoMove(&right_elbow, 180);
      rightArmStepState = 1;
      return false;
    case 1:
      startServoMove(&right_sholder, 0);
      rightArmStepState = 2;
      return false;
    case 2:
      startServoMove(&right_elbow, 90);
      rightArmStepState = 3;
      return false;
    case 3:
      startServoMove(&right_sholder, 90);
      rightArmStepState = 0;
      return true;
  }
  return false;
}

bool left_leg_step_forward() {
  switch (leftLegStepState) {
    case 0:
      startServoMove(&left_leg, 20);
      leftLegStepState = 1;
      return false;
    case 1:
      startServoMove(&left_foot, 0);
      leftLegStepState = 2;
      return false;
    case 2:
      startServoMove(&left_leg, 90);
      leftLegStepState = 3;
      return false;
    case 3:
      startServoMove(&left_foot, 90);
      leftLegStepState = 0;
      return true;
  }
  return false;
}

bool right_leg_step_forward() {
  switch (rightLegStepState) {
    case 0:
      startServoMove(&right_leg, 160);
      rightLegStepState = 1;
      return false;
    case 1:
      startServoMove(&right_foot, 180);
      rightLegStepState = 2;
      return false;
    case 2:
      startServoMove(&right_leg, 90);
      rightLegStepState = 3;
      return false;
    case 3:
      startServoMove(&right_foot, 90);
      rightLegStepState = 0;
      return true;
  }
  return false;
}

bool turn_left_leg_step() {
  switch (turnLeftLegStepState) {
    case 0:
      startServoMove(&left_leg, 20);
      turnLeftLegStepState = 1;
      return false;
    case 1:
      startServoMove(&left_foot, 0);
      turnLeftLegStepState = 2;
      return false;
    case 2:
      startServoMove(&left_leg, 90);
      turnLeftLegStepState = 3;
      return false;
    case 3:
      startServoMove(&left_foot, 90);
      turnLeftLegStepState = 0;
      return true;
  }
  return false;
}

bool turn_right_arm_step() {
  switch (turnRightArmStepState) {
    case 0:
      startServoMove(&right_elbow, 180);
      turnRightArmStepState = 1;
      return false;
    case 1:
      startServoMove(&right_sholder, 0);
      turnRightArmStepState = 2;
      return false;
    case 2:
      startServoMove(&right_elbow, 90);
      turnRightArmStepState = 3;
      return false;
    case 3:
      startServoMove(&right_sholder, 90);
      turnRightArmStepState = 0;
      return true;
  }
  return false;
}

bool turn_right() {
  switch (turnRightStepState) {
    case 0:
      // Execute left leg step as part of turn
      if (turn_left_leg_step()) {
        turnRightStepState = 1;
      }
      return false;
    case 1:
      // Now do right arm step
      if (turn_right_arm_step()) {
        turnRightStepState = 0;
        return true;
      }
      return false;
  }
  return false;
}

bool turn_left() {
  switch (turnLeftStepState) {
    case 0:
      // Execute right leg step as part of left turn
      if (turn_right_leg_step()) {
        turnLeftStepState = 1;
      }
      return false;
    case 1:
      // Now do left arm step
      if (turn_left_arm_step()) {
        turnLeftStepState = 0;
        return true; // Turn completed
      }
      return false;
  }
  return false;
}

void startServoMove(Servo* servo, int angle, int delayMs = default_delay_ms) {
  if (servoState == SERVO_IDLE) {
    currentServo = servo;
    targetAngle = angle;
    currentAngle = servo->read();
    moveDelay = delayMs;
    
    if (currentAngle > targetAngle) {
      stepDirection = -1;
    } else if (currentAngle < targetAngle) {
      stepDirection = 1;
    } else {
      // Already at target
      servoState = SERVO_IDLE;
      return;
    }
    
    servoState = SERVO_MOVING;
    servoMoveTime = millis();
  }
}

void initializeServos() {
  left_foot.write(90);
  right_sholder.write(90);
  right_foot.write(90);
  right_elbow.write(90);
  left_elbow.write(90);
  right_leg.write(90);
  left_leg.write(90);
}

float get_distance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  float duration = pulseIn(echoPin, HIGH);
  float distance = (duration*.0343)/2;
  return distance;
}

bool is_too_close(){
  float distance = get_distance();
  if(distance < 10){ return true;}
  else{ return false;}
}
