#include <Pololu3piPlus32U4.h>  // Pololu 3pi+ library for motor and encoder control
#include <Pololu3piPlus32U4Buttons.h>  // Button control library for 3pi+
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"  // Particle filter library for localization
#include "odometry.h"  // Odometry library to calculate position
//#include "Map.h"  // Commented out map library, not in use
#include "PIDcontroller.h"  // PID controller library for movement control
#include <Servo.h>  // Servo library for controlling servos (sonar sensor)
#include "sonar.h"  // Sonar sensor library for distance measurements
#include <PololuBuzzer.h>  // Buzzer library for sound feedback
#include "printOLED.h"

PololuBuzzer buzzer;  // Create a buzzer object

// Music string for playing sound (cut for brevity)
const char championsSnippet[] PROGMEM = "!L8 c d f8 g8 a4 g4 f8 d8 f4."; // Roughly matches “We are the champions”


// Constants for map size and robot parameters
#define PI 3.14159
#define lenOfMap 36  // Map length (4 rows * 9 columns)
#define N_particles 25  // Number of particles for particle filter
#define move_noise 0.01  // Noise in movement
#define rotate_noise 0.5  // Noise in rotation
#define ultra_noise 0.1  // Noise in sonar measurements

// Robot geometry and encoder specifications
#define diaL 3.2  // Diameter of the left wheel (in cm)
#define diaR 3.2  // Diameter of the right wheel (in cm)
#define nL 12  // Number of encoder ticks for the left wheel per rotation
#define nR 12  // Number of encoder ticks for the right wheel per rotation
#define w 9.6  // Width of the robot (in cm)
#define gearRatio 75  // Gear ratio of the robot's motors
#define deadreckoning false  // Whether to use dead reckoning for navigation (false = no)

#define minOutputVel -100  // Minimum output angle for PID
#define maxOutputVel 100  // Maximum output angle for PID
#define kpVel 25  // Proportional gain for PID
#define kdVel 5  // Derivative gain for PID
#define kiVel 0.25  // Integral gain for PID
#define clamp_iVel 50  // Integral clamp for PID


// Create objects for motors, servos, encoders, and odometry
Motors motors;
Servo servo;
Encoders encoders;
PrintOLED oled;
LineSensors lineSensors;  // Add this with other sensor/motor declarations
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, deadreckoning);  // Initialize odometry object

PIDcontroller pidVel(kpVel, kiVel, kdVel, minOutputVel, maxOutputVel, clamp_iVel);  // Initialize PID controller

// Odometry and encoder variables
int16_t deltaL = 0, deltaR = 0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 10; 
float y = 10; 
float theta; // Robot's position (x, y) and orientation (theta)
const float delta_checker = 20.0;
float last_x = 0;
float last_y = 0;

double PIDout_theta;  // Output of the PID controller for angular movement


//Line Sensor
unsigned int blueValues[5];  // Store blue square values
unsigned int lineSensorValues[5]; //constant looped sensor values
const int blackThreshold = 2000;
const int blueThreshold = 1;


//wall follow
const double desiredDistFromWall=10.0; // Goal distance from wall (cm)
bool firstWallFollowed = false;
bool secondWallFollowed = false;
bool switched = false;
bool wall_ahead = false;

float wallDist;
int PIDout;
int base_speed = 100;


//trash
int trashCount = 0;
const int foundTrashSpeed = 100;

// Grid dimensions and tracking
const int ROWS = 4;
const int COLS = 9;
bool gridVisited[ROWS][COLS] = {false};  // Track if a cell was visited
bool gridHasTrash[ROWS][COLS] = {false}; // Track if trash was found in a cell

int currentRow = -1, currentCol = -1;  // Current grid position





// Struct to hold distances from sonar sensor
struct Distances {
  int front;
  int left;
  int right;
};

// Function to check distances using sonar sensor
Distances checkSurroundings() {
  Distances d;
  servo.write(90); delay(50); d.front = sonar.readDist(); delay(50);
  Serial.print("Front: ");
  Serial.println(d.front);
  servo.write(180); delay(50); d.left = sonar.readDist(); delay(50);
  Serial.print("Left: ");
  Serial.println(d.left);
  servo.write(0); delay(50); d.right = sonar.readDist(); delay(50);
  Serial.print("Right: ");
  Serial.println(d.right);
  servo.write(90); delay(50);
  return d;
}



void wallFollowingSearch(){
  float _delta_x = abs(x - last_x);
  float _delta_y = abs(y - last_y);
  if (_delta_x > delta_checker || _delta_y > delta_checker){
    motors.setSpeeds(0,0);
    last_x = x;
    last_y = y;
    Distances d = checkSurroundings();
    if(d.front < 15){
      wall_ahead = true;
    }
  }
  if(firstWallFollowed){
    if (wall_ahead){
      wall_ahead = false;
      motors.setSpeeds(50,0);
      delay(250);
      motors.setSpeeds(0,0);
    }
    servo.write(180);
    delay(20);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
  }
  else{
    if (wall_ahead){
      wall_ahead = false;
      motors.setSpeeds(0,50);
      delay(250);
      motors.setSpeeds(0,0);
    }
    servo.write(0);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed + PIDout;
    int rightSpeed = base_speed - PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
  }

  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);

  lineSensors.read(lineSensorValues);

  for (int i = 0; i < 5; i++) {
    Serial.println(lineSensorValues[i]);
      int trashCol = (int)(x / 20);
      int trashRow = (int)(y / 20);
    if (lineSensorValues[i] > blackThreshold && !gridHasTrash[trashRow][trashCol]) {
      trashCount++;
      
      // ✅ Trash location logging based on current odometry position
      int trashCol = (int)(x / 20);
      int trashRow = (int)(y / 20);
      if (trashRow >= 0 && trashRow < ROWS && trashCol >= 0 && trashCol < COLS) {
        gridVisited[trashRow][trashCol] = true;
        gridHasTrash[trashRow][trashCol] = true;
        Serial.print("Trash logged at cell: (");
        Serial.print(trashRow);
        Serial.print(", ");
        Serial.print(trashCol);
        Serial.println(")");
      }

      buzzer.playFromProgramSpace(championsSnippet);
      for (int i = 0; i < 50; i++) {
        motors.setSpeeds(-foundTrashSpeed, foundTrashSpeed);
        delay(10);
      }
      for (int i = 0; i < 50; i++) {
        motors.setSpeeds(foundTrashSpeed, -foundTrashSpeed);
        delay(10);
      }
      motors.setSpeeds(0, 0);
      Serial.println(F("Trash Found"));
    }

    else if (lineSensorValues[i] > blueThreshold && firstWallFollowed) {
      secondWallFollowed = true;
    }

    else if (lineSensorValues[i] > blueThreshold) {
      firstWallFollowed = true;
    }
  }
}





void switchWalls(){
  for (uint8_t i = 0; i < 200; i++){
    servo.write(180);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist,desiredDistFromWall);
    int leftSpeed = base_speed + PIDout;
    int rightSpeed = base_speed - PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
  }
  switched = true;
  wallFollowingSearch();
}



void backToBase(){
  servo.write(180);
  wallDist = sonar.readDist();
  PIDout = pidVel.update(wallDist,desiredDistFromWall);
  int leftSpeed = base_speed + PIDout;
  int rightSpeed = base_speed - PIDout;
  motors.setSpeeds(leftSpeed, rightSpeed);
  for (int i = 0; i < 5; i++){
  if (lineSensorValues[i] > blueThreshold){
    motors.setSpeeds(0, 0);
    while(true){}
  }
  }
}












void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  servo.attach(5);  // Attach the servo to pin 5
  servo.write(90);  // Set the servo to the center position
  delay(1000);  // Wait for a second
  lineSensors.read(blueValues);  // Fills the array blue
  for (uint8_t i = 0; i < 5; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(blueValues[i]);
  }
}

// void loop() {
//   oled.print_int(trashCount);
//   // Get encoder counts and update the odometry
//   deltaL = encoders.getCountsAndResetLeft();
//   deltaR = encoders.getCountsAndResetRight();
//   encCountsLeft += deltaL;
//   encCountsRight += deltaR;
//   odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
// if (firstWallFollowed && !switched){
//     switchWalls();
//   }
//   wallFollowingSearch();
//   if(secondWallFollowed){
//     while(true){}
//   }
// }

void loop() {
  oled.print_int(trashCount);
  // Get encoder counts and update the odometry
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
  if (firstWallFollowed && !switched){
    switchWalls();
  }
  wallFollowingSearch();
  if (trashCount == 3){
    backToBase();
  }
}
