#include <Pololu3piPlus32U4.h>                  // Pololu 3pi+ library for motor and encoder control
#include <Pololu3piPlus32U4Buttons.h>           // Button control library for 3pi+
using namespace Pololu3piPlus32U4;

#include "particle_filter.h"                    // Particle filter library for localization
#include "odometry.h"                           // Odometry library to calculate position
//#include "Map.h"                               // Map library (not in use)
#include "PDcontroller.h"                       // PID controller for motion control
#include <Servo.h>                              // Servo control (used for sonar positioning)
#include "sonar.h"                              // Sonar sensor library for distance readings
#include <PololuBuzzer.h>                       // Buzzer library for auditory feedback
#include "printOLED.h"                          // OLED display output helper

// Buzzer configuration
PololuBuzzer buzzer;
const char championsSnippet[] PROGMEM = "!L8 c d f8 g8 a4 g4 f8 d8 f4."; // Sound snippet for events

// Global constants
#define PI 3.14159
#define lenOfMap 36                     // Total number of cells in the map (4x9)
#define N_particles 25                  // Number of particles for localization
#define move_noise 0.01                 // Movement noise for particle filter
#define rotate_noise 0.5                // Rotation noise
#define ultra_noise 0.1                 // Sonar noise

// Robot geometry and encoder setup
#define diaL 3.2                        // Left wheel diameter (cm)
#define diaR 3.2                        // Right wheel diameter (cm)
#define nL 12                           // Left encoder ticks/rev
#define nR 12                           // Right encoder ticks/rev
#define w 9.6                           // Distance between wheels (cm)
#define gearRatio 75                   // Motor gear ratio
#define deadreckoning false            // Disable dead reckoning mode

// PID controller parameters
#define minOutputVel -100
#define maxOutputVel 100
#define kpVel 25
#define kdVel 5

// Hardware objects
Motors motors;
Servo servo;
Encoders encoders;
PrintOLED oled;
LineSensors lineSensors;
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, deadreckoning);
PDcontroller pidVel(kpVel, kdVel, minOutputVel, maxOutputVel);

// Odometry tracking variables
int16_t deltaL = 0, deltaR = 0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10, y = 10, theta;
const float delta_checker = 10.0;
float last_x = 0, last_y = 0;
double PIDout_theta;

// Line sensor calibration and detection
int calibrationSpeed = 100;
unsigned int blueValues[5];
unsigned int lineSensorValues[5];
const int blackThreshold = 2000;
const int blueThreshold = 800;

// Wall following parameters
const double desiredDistFromWall = 10.0;
bool firstWallFollowed = false;
float wallDist;
int PIDout;
int base_speed = 100;
unsigned long lastActionTime = 0;
const unsigned long deltaTime = 2000; // ms
int wallDistFront = 10;

// Trash detection and tracking
struct TrashLocation {
  float x;
  float y;
};

const int maxTrash = 3;
TrashLocation foundTrash[maxTrash];
int trashCount = 0;
float ignoreRadius = 20.0;

// Sonar distance structure
struct Distances {
  int front;
  int left;
  int right;
};

// Sensor calibration routine
void calibrateSensors() {
  Serial.println(F("Calibrating..."));
  delay(1000);
  for (int i = 0; i < 50; i++) {
    motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    lineSensors.calibrate();
    delay(10);
  }
  for (int i = 0; i < 50; i++) {
    motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    lineSensors.calibrate();
    delay(10);
  }
  motors.setSpeeds(0, 0);
  Serial.println(F("Calibration Done"));
  delay(1000);
}

// Get sonar readings in all directions
Distances checkSurroundings() {
  Distances d;
  servo.write(90); delay(100); d.front = sonar.readDist(); delay(100);
  Serial.print("Front: "); Serial.println(d.front);

  servo.write(180); delay(100); d.left = sonar.readDist(); delay(100);
  Serial.print("Left: "); Serial.println(d.left);

  servo.write(0); delay(100); d.right = sonar.readDist(); delay(100);
  Serial.print("Right: "); Serial.println(d.right);

  servo.write(90); delay(50);  // Reset to center
  return d;
}

// Check only front sonar reading
Distances checkFront() {
  Distances f;
  servo.write(90); delay(100); f.front = sonar.readDist(); delay(100);
  servo.write(0); delay(100);
  if (firstWallFollowed) {
    servo.write(180);
  }
  Serial.print("Front: "); Serial.println(f.front);
  f.left = 0;
  f.right = 0;
  return f;
}

// Spin robot 90° left
void spinLeft() {
  motors.setSpeeds(-250, 250);
  delay(250);
  motors.setSpeeds(0, 0);
}

// Spin robot 90° right
void spinRight() {
  motors.setSpeeds(250, -250);
  delay(250);
  motors.setSpeeds(0, 0);
}

// Check if trash is already found near this location
bool isNearExistingTrash(float x, float y) {
  for (int i = 0; i < trashCount; i++) {
    float dx = x - foundTrash[i].x;
    float dy = y - foundTrash[i].y;
    float distance = sqrt(dx * dx + dy * dy);
    if (distance < ignoreRadius) {
      return true;
    }
  }
  return false;
}

// Respond to trash detection
void trashFound(float x, float y) {
  oled.print_int(trashCount);
  motors.setSpeeds(0, 0);
  buzzer.playFrequency(1000, 2000, 15); // Audible alert
  motors.setSpeeds(250, -250); delay(2000); // Spin
  motors.setSpeeds(-250, 250); delay(2000); // Spin back
  motors.setSpeeds(0, 0);
}

// Wall switching logic
void switchWalls() {
  spinLeft();
  motors.setSpeeds(100, 100);
  delay(1500);
  for (uint8_t i = 0; i < 200; i++) {
    servo.write(180);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
  }
  firstWallFollowed = true;
}

// Check for obstacles in front
void checkLocation() {
  unsigned long currentTime = millis();
  if (currentTime - lastActionTime >= deltaTime) {
    Distances f = checkFront();
    if (f.front <= wallDistFront) {
      if (firstWallFollowed) {
        spinRight();
      } else {
        spinLeft();
      }
    }
    lastActionTime = currentTime;
  }
}

// Check line sensor for trash or homebase
void checkFloor() {
  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++) {
    if (lineSensorValues[i] > blackThreshold && trashCount < maxTrash) {
      if (!isNearExistingTrash(x, y)) {
        foundTrash[trashCount] = {x, y};
        trashCount++;
        trashFound(x, y);
      }
    }
    else if (lineSensorValues[i] > blueThreshold && lineSensorValues[i] < blackThreshold && x < 15 && y < 15) {
      if (firstWallFollowed && trashCount >= 3) {
        while (true) {
          motors.setSpeeds(0, 0); // Stop robot if task is complete
        }
      } else {
        switchWalls(); // Detected homebase; switch sides
      }
    }
  }
}

// Main setup routine
void setup() {
  Serial.begin(9600);
  servo.attach(5);       // Attach servo to pin 5
  servo.write(0);        // Face sonar right
  delay(1000);
  oled.print_int(trashCount);

  // Wait for user to start via serial input
  while (Serial.available() == 0) {}

  calibrateSensors();    // Calibrate line sensors

  servo.write(0);        // Start with sonar facing right
  motors.setSpeeds(100, 100); delay(1000);

  // Initialize odometry
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}

// Main loop
void loop() {
  oled.print_int(trashCount);

  // Update encoder readings and odometry
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

  if (!firstWallFollowed) {
    // Outer wall following
    servo.write(0);
    wallDist = sonar.readDist();
    Serial.println(wallDist);
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);

    Serial.println("checkLocation");
    checkLocation();
    checkFloor();
  } else {
    // Inner wall following
    servo.write(180);
    wallDist = sonar.readDist();
    Serial.println(wallDist);
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed + PIDout;
    int rightSpeed = base_speed - PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);

    Serial.println("checkLocation");
    checkLocation();
    checkFloor();
  }
}
