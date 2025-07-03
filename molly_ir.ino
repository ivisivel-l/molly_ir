/* Molly the turtuise
IR controlled version
by Lev Kunin lev.y.kunin@gmail.com

IR Remote Control Features:

Key 1: Toggle robot on/off
Key 2: Start continuous walking
Key 3: Stop robot
Arrow keys: Manual control of individual limbs

UP: Left arm step
DOWN: Right arm step
LEFT: Left leg step
RIGHT: Right leg step

*/


#include <DIYables_IRcontroller.h>
#include <Servo.h>
#define IR_RECEIVER_PIN 45 // The Arduino pin connected to IR controller

DIYables_IRcontroller_17 irController(IR_RECEIVER_PIN, 200); // debounce time is 200ms
Servo right_elbow;
Servo left_elbow;
Servo left_sholder;  // create Servo object to control a servo// twelve Servo objects can be created on most boards
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
bool robotEnabled = true;

// State machine variables
enum RobotState {
  IDLE,
  LEFT_ARM_STEP,
  RIGHT_ARM_STEP,
  LEFT_LEG_STEP,
  RIGHT_LEG_STEP,
  TURNING_RIGHT
};

enum ServoMoveState {
  SERVO_IDLE,
  SERVO_MOVING
};

RobotState currentState = IDLE;
ServoMoveState servoState = SERVO_IDLE;

// Walking sequence variables
int walkCounter = 0;
int stepSequence = 0; // 0=left_arm, 1=right_arm, 2=left_leg, 3=right_leg

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

// Function declarations
void startServoMove(Servo* servo, int angle, int delayMs = default_delay_ms);
bool executeLeftArmStep();
bool executeRightArmStep();
bool executeLeftLegStep();
bool executeRightLegStep();
bool executeTurnRight();
void handleIRCommands();
void handleServoMovement(unsigned long currentMillis);
void handleRobotStateMachine(unsigned long currentMillis);
float get_distance();
bool is_too_close();
void initializeServos();

void setup() {   
  // distance sensor setup
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  //IR receiver setup
  Serial.begin(9600);
  irController.begin();
  
  // attach motors
  right_foot.attach(7);
  left_foot.attach(5);
  left_sholder.attach(10);  // attaches the servo on pin 9 to the Servo object
  right_sholder.attach(9);
  left_elbow.attach(8);
  right_elbow.attach(11);
  left_leg.attach(4);
  right_leg.attach(6);  

  // Initialize all motors to 90 degrees with non-blocking setup
  initializeServos();
  
  Serial.println("Robot initialized. Use IR remote to control.");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Always check for IR commands
  handleIRCommands();
  
  // Only run robot logic if enabled
  if (robotEnabled) {
    // Handle servo movements
    handleServoMovement(currentMillis);
    
    // Handle main robot state machine
    handleRobotStateMachine(currentMillis);
  }
}

void handleIRCommands() {
  Key17 key = irController.getKey();
  if (key != Key17::NONE) {
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
          Serial.println("Starting walk sequence");
          currentState = LEFT_ARM_STEP;
          stepSequence = 0;
          leftArmStepState = 0;
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
      case Key17::KEY_LEFT:
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
      default:
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
          break;
        }
        
        if (executeLeftArmStep()) {
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
          break;
        }
        
        if (executeRightArmStep()) {
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
          break;
        }
        
        if (executeLeftLegStep()) {
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
          break;
        }
        
        if (executeRightLegStep()) {
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
        if (executeTurnRight()) {
          // Turn completed, check if still too close
          if (navigate && is_too_close()) {
            // Still too close, keep turning
            turnRightStepState = 0;
          } else {
            // Clear path, resume previous activity
            currentState = LEFT_ARM_STEP;  // Resume walking
            leftArmStepState = 0;
          }
        }
      }
      break;
  }
}

bool executeLeftArmStep() {
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
      return true; // Step completed
  }
  return false;
}

bool executeRightArmStep() {
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
      return true; // Step completed
  }
  return false;
}

bool executeLeftLegStep() {
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
      return true; // Step completed
  }
  return false;
}

bool executeRightLegStep() {
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
      return true; // Step completed
  }
  return false;
}

bool executeTurnRight() {
  switch (turnRightStepState) {
    case 0:
      // Execute left leg step as part of turn
      currentState = LEFT_LEG_STEP;
      leftLegStepState = 0;
      turnRightStepState = 1;
      return false;
    case 1:
      if (currentState == LEFT_LEG_STEP) {
        return false; // Still executing left leg step
      }
      // Left leg step completed, now do right arm step
      currentState = RIGHT_ARM_STEP;
      rightArmStepState = 0;
      turnRightStepState = 2;
      return false;
    case 2:
      if (currentState == RIGHT_ARM_STEP) {
        return false; // Still executing right arm step
      }
      // Turn completed
      turnRightStepState = 0;
      return true;
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
  // Set all motors to 90 degrees - this will happen gradually as the robot starts
  left_sholder.write(90);
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