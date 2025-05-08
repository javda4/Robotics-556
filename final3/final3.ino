#include <Pololu3piPlus32U4.h>  // Pololu 3pi+ robot control library
#include <Pololu3piPlus32U4Buttons.h>  // Library for button input
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"  // Particle filter for localization (not directly used here)
#include "odometry.h"  // Odometry calculations
//#include "Map.h"  // Map header (commented out, not used)
#include "PDcontroller.h"  // Proportional-Derivative control for wall following
#include <Servo.h>  // Servo control
#include "sonar.h"  // Sonar distance measurement
#include <PololuBuzzer.h>  // For playing sounds
#include "printOLED.h"  // OLED screen output

PololuBuzzer buzzer;  // Sound buzzer object

// Tune to play when trash is found (shortened here for brevity)
const char championsSnippet[] PROGMEM = "!L8 c d f8 g8 a4 g4 f8 d8 f4.";

// Constants
#define PI 3.14159
#define lenOfMap 36  // Number of tiles (if using grid map)
#define N_particles 25  // Number of particles for localization
#define move_noise 0.01  // Motion noise
#define rotate_noise 0.5
#define ultra_noise 0.1

// Robot geometry
#define diaL 3.2  // Left wheel diameter (cm)
#define diaR 3.2  // Right wheel diameter (cm)
#define nL 12     // Encoder ticks per rotation (left)
#define nR 12     // Encoder ticks per rotation (right)
#define w 9.6     // Robot width (cm)
#define gearRatio 75
#define deadreckoning false

// PID control parameters for wall following
#define minOutputVel -100
#define maxOutputVel 100
#define kpVel 25
#define kdVel 5

// Robot hardware and utilities
Motors motors;
Servo servo;
Encoders encoders;
PrintOLED oled;
LineSensors lineSensors;
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, deadreckoning);  // Initialize odometry
PDcontroller pidVel(kpVel, kdVel, minOutputVel, maxOutputVel);

// Odometry state variables
int16_t deltaL = 0, deltaR = 0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10, y = 10, theta;  // Robot's estimated position and heading
const float delta_checker = 10.0;  // Unused in this snippet
float last_x = 0, last_y = 0;

// Line sensor thresholds
int calibrationSpeed = 100;
unsigned int blueValues[5];  // Store blue color readings
unsigned int lineSensorValues[5];
const int blackThreshold = 2000;
const int blueThreshold = 800;

// Wall following
const double desiredDistFromWall = 10.0;  // Target distance from wall
bool firstWallFollowed = false;  // Indicates whether robot is navigating inner wall

// Trash detection
struct TrashLocation {
  float x;
  float y;
};

const int maxTrash = 3;
TrashLocation foundTrash[maxTrash];  // Array of found trash locations
int trashCount = 0;
float ignoreRadius = 20.0;  // Minimum distance to register new trash

// State machine for turning logic
struct TurnState {
  bool outer1_done = false;
  bool outer2_done = false;
  bool outer3_done = false;
  bool outer4_done = false;
  bool outer5_done = false;
  bool inner1_done = false;
  bool inner2_done = false;
  bool inner3_done = false;
  bool inner4_done = false;
  bool inner5_done = false;
  bool inner6_done = false;
};
TurnState turnState;

// Struct to hold sonar readings
struct Distances {
  int front;
  int left;
  int right;
};

// Line sensor calibration routine
void calibrateSensors() {
  Serial.println(F("Calibrating..."));
  delay(1000);
  // Rotate right to calibrate
  for (int i = 0; i < 50; i++) {
    motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
    lineSensors.calibrate();
    delay(10);
  }
  // Rotate left to calibrate
  for (int i = 0; i < 50; i++) {
    motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
    lineSensors.calibrate();
    delay(10);
  }
  motors.setSpeeds(0, 0);  // Stop motors
  Serial.println(F("Calibration Done"));
  delay(1000);
}

// Returns distances from front, left, and right via sonar
Distances checkSurroundings() {
  Distances d;
  servo.write(90); delay(100); d.front = sonar.readDist(); delay(100);
  servo.write(180); delay(100); d.left = sonar.readDist(); delay(100);
  servo.write(0); delay(100); d.right = sonar.readDist(); delay(100);
  servo.write(90); delay(50);  // Reset servo to front
  return d;
}

// Returns only the front distance from sonar
Distances checkfront(){
  Distances f;
  servo.write(90); delay(100); f.front = sonar.readDist(); delay(100);
  f.left = 0;
  f.right = 0;
  return f;
}

// Simple left spin for turns
void spinLeft(){
  motors.setSpeeds(-250, 250);
  delay(250);
  motors.setSpeeds(0, 0);
}

// Simple right spin for turns
void spinRight(){
  motors.setSpeeds(250, -250);
  delay(250);
  motors.setSpeeds(0, 0);
}

// Check if new trash is too close to previous trash
bool isNearExistingTrash(float x, float y) {
  for (int i = 0; i < trashCount; i++) {
    float dx = x - foundTrash[i].x;
    float dy = y - foundTrash[i].y;
    float distance = sqrt(dx * dx + dy * dy);
    if (distance < ignoreRadius) return true;
  }
  return false;
}

// Called when trash is found
void trashFound(float x, float y){
  motors.setSpeeds(0, 0);
  // Visual + audible feedback could go here
  motors.setSpeeds(250, -250); delay(1000);  // Spin for emphasis
  motors.setSpeeds(-250, 250); delay(1000);  // Spin back
  motors.setSpeeds(0, 0);
}

// Switch from following right wall to left wall
void switchWalls() {
  for (uint8_t i = 0; i < 200; i++){
    servo.write(180);  // Look left
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
  }
  firstWallFollowed = true;
}

// Check position and execute specific turns based on location
void checkLocation(){
  if (x > 69 && y < 20 && !turnState.outer1_done){ spinLeft(); turnState.outer1_done = true; }
  if (x > 60 && y > 169 && !turnState.outer2_done){ spinLeft(); turnState.outer2_done = true; }
  if (x < 18 && y > 160 && !turnState.outer3_done){ spinLeft(); turnState.outer3_done = true; }
  if (x < 20 && y > 80 && y < 92 && !turnState.outer4_done){ spinLeft(); turnState.outer4_done = true; }
  if (x < 12 && y > 20 && y < 40 && !turnState.outer5_done){ spinLeft(); turnState.outer5_done = true; }

  if (firstWallFollowed && x < 40 && x > 20 && y > 100 && y < 120 && !turnState.inner1_done){ spinRight(); turnState.inner1_done = true; }
  if (firstWallFollowed && x < 30 && x > 20 && y > 130 && y < 140 && !turnState.inner2_done){ spinRight(); turnState.inner2_done = true; }
  if (firstWallFollowed && x > 45 && x < 60 && y > 140 && y < 160 && !turnState.inner3_done){ spinRight(); turnState.inner3_done = true; }
  if (firstWallFollowed && x > 45 && x < 60 && y > 140 && y < 145 && !turnState.inner4_done){ spinRight(); turnState.inner4_done = true; }
  if (firstWallFollowed && x > 45 && x < 60 && y > 80 && y < 100 && !turnState.inner5_done){ spinRight(); turnState.inner5_done = true; }
  if (firstWallFollowed && x > 40 && x < 55 && y > 20 && y < 30 && !turnState.inner6_done){ spinRight(); turnState.inner6_done = true; }
}

// Detect trash (black) and dropoff zone (blue)
void checkFloor(){
  lineSensors.read(lineSensorValues);
  for (int i = 0; i < 5; i++){
    if (lineSensorValues[i] > blackThreshold && trashCount < maxTrash){
      if (!isNearExistingTrash(x, y)) {
        foundTrash[trashCount] = {x, y};
        trashCount++;
        trashFound(x, y);
      }
    }
    else if (lineSensorValues[i] > blueThreshold && x < 20 && y < 20){
      if (firstWallFollowed && trashCount >= 3){
        // All trash found and returned to start — stop robot
        while(true){ motors.setSpeeds(0, 0); }
      }
      else{
        // Switch to following inner wall
        switchWalls();  
      }
    }
  }
}

// Setup function
void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(0);  // Start facing right wall
  delay(1000);
  
  // Wait for user input before starting
  while (Serial.available() == 0) {}

  calibrateSensors();  // Calibrate line sensors

  // Move forward briefly to start from a known position
  motors.setSpeeds(100, 100);
  delay(1500);

  // Initialize odometry
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
}

// Main loop
void loop() {
  oled.print_int(trashCount);  // Display trash count on screen

  // Update odometry
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

  if (!firstWallFollowed){
    // Follow outer wall (right side)
    servo.write(0);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    motors.setSpeeds(base_speed - PIDout, base_speed + PIDout);
  }
  else{
    // Follow inner wall (left side)
    servo.write(180);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    motors.setSpeeds(base_speed + PIDout, base_speed - PIDout);
  }

  checkLocation();  // Determine if a corner turn is needed
  checkFloor();     // Check for trash or dropoff zone
}
