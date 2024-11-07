#include <Pixy2.h>
#include <Pixy2CCC.h>

#include <SPI.h>
#include <Servo.h>

Pixy2 pixy;
Servo steeringServo;
Servo driveServo;

////////////////////////////////////////////////////////

int motorPin = 8; // Motor PWM pin
int servoPin = 9; // Servo control pin
float deadZone = 0.15;
int baseSpeed = 1200;

////////////////////////////////////////////////////////

int cont = 0;
int signature, x, y, width, height;
float cx, cy, area;

void setup() {
  Serial.begin(9600);
  Serial.print("Starting...\n");
  pixy.init();
  // Initialize motor and servo pins
  //pinMode(motorPin, OUTPUT);
  steeringServo.attach(servoPin);
  driveServo.attach(motorPin);
  steeringServo.write(90); // Set servo to center (90 degrees)
  delay(100);
}

void loop() {
  float turn = pixyCheck();
  delay(150);
  driveServo.writeMicroseconds(baseSpeed);

  if (turn > -deadZone && turn < deadZone) {
    turn = 0;
  }

  if (turn < 0) {
    moveRobot(baseSpeed, -1); // Turn left
  }
  else if (turn > 0) {
    moveRobot(baseSpeed, 1); // Turn right
  }
  else {
    moveRobot(baseSpeed, 0); // Move forward
  }
  delay(5);
  driveServo.writeMicroseconds(baseSpeed);
}

float pixyCheck() {
  static int i = 0;
  int j;
  uint16_t blocks;
  char buf[32];
  
  // Grab blocks
  blocks = pixy.ccc.getBlocks();
  Serial.print("Blocks: ");
  Serial.println(blocks);

  // If there are detected blocks, process them
  if (blocks) {
    signature = pixy.ccc.blocks[0].m_signature;
    height = pixy.ccc.blocks[0].m_height;
    width = pixy.ccc.blocks[0].m_width;
    x = pixy.ccc.blocks[0].m_x;
    y = pixy.ccc.blocks[0].m_y;
    cx = (x + (width / 2));
    cy = (y + (height / 2));
    cx = mapfloat(cx, 0, 320, -1, 1); // Map x position to -1 to 1 range
    area = width * height;
  }
  else {
    cont += 1;
    if (cont == 100) {
      cont = 0;
      cx = 0; // No target, keep moving forward
    }
  }
  return cx;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void moveRobot(int motorSpeed, int turnDirection) {
  // Control motor speed
  
  
  // Control servo for direction
  int servoAngle;
  if (turnDirection > 0) {
    servoAngle = 60; // Turn left
  }
  else if (turnDirection < 0) {
    servoAngle = 120; // Turn right
  }
  else {
    servoAngle = 90; // Go straight
  }
  steeringServo.write(servoAngle);
}
