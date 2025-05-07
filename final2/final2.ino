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
float x = 10;  // Robot's position (x, y) and orientation (theta)
float y = 10; // Robot's position (x, y) and orientation (theta)
float theta; // Robot's position (x, y) and orientation (theta)
const float delta_checker = 10.0;
float last_x = 0;
float last_y = 0;

double PIDout_theta;  // Output of the PID controller for angular movement


//Line Sensor
unsigned int blueValues[5];  // Store blue square values
unsigned int lineSensorValues[5]; //constant looped sensor values
const int blackThreshold = 2000;
const int blueThreshold = 800;


//wall follow
const double desiredDistFromWall=10.0; // Goal distance from wall (cm)
bool firstWallFollowed = false;





float wallDist;
int PIDout;
int base_speed = 100;


//trash

const int maxTrash = 3;
TrashLocation foundTrash[maxTrash]; // Stores trash positions
int trashCount = 0;
float ignoreRadius = 15.0; // Distance threshold



struct TrashLocation {
  float x;
  float y;
};


struct TurnState {
  bool outer1_done = false;
  bool outer2_done = false;
  bool outer3_done = false;
  bool outer4_done = false;
  bool inner1_done = false;
  bool inner2_done = false;
  // Add more corners if needed
};

TurnState turnState; // Global instance



// Struct to hold distances from sonar sensor
struct Distances {
  int front;
  int left;
  int right;
};

// Function to check distances using sonar sensor
Distances checkSurroundings() {
  Distances d;
  servo.write(90); delay(100); d.front = sonar.readDist(); delay(100);
  Serial.print("Front: ");
  Serial.println(d.front);
  servo.write(180); delay(100); d.left = sonar.readDist(); delay(100);
  Serial.print("Left: ");
  Serial.println(d.left);
  servo.write(0); delay(100); d.right = sonar.readDist(); delay(100);
  Serial.print("Right: ");
  Serial.println(d.right);
  servo.write(90); delay(50);
  return d;
}

Distances checkfront(){
  Distances f;
  servo.write(90); delay(100); f.front = sonar.readDist(); delay(100);
  Serial.print("Front: ");
  Serial.println(f.front);
  f.left = 0;
  f.right = 0;
  return f;
}

void spinLeft(){
  motors.setSpeeds(-100,100);
  delay(250);
  motors.setSpeeds(0,0);
}



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





void trashFound(float x, float y){
  motors.setSpeeds(0,0);
  //play noise
  //.......................;               //////////////////////////////////////////////////////
  //spin robot in place
  for(int i = 0; i < 50; i++){
    motors.setSpeeds(100,-100);
    delay(10);
  }
  //spin back same amount
  for(int i = 0; i < 50; i++){
    motors.setSpeeds(-100,100);
    delay(10);
  }
  motors.setSpeeds(0,0);
}


void switchWalls(){
// may need to add here
  for (uint8_t i = 0; i < 200; i++){
    servo.write(180);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist,desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
  }
  firstWallFollowed = false;
}




void checkLocation(){
  if (x > 65 && y < 20 && !turnState.outer1_done){
    spinLeft();
    turnState.outer1_done = true;
  }
//////////////////////////////////////////////////////////////////////add locations//////////////////////////////////

}




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
        while(true){
          motors.setSpeeds(0,0);
        } //create infinite loop to stop robot running
      }
      else{
        switchWalls();  
      }
    }
  }
}






void setup() {
  Serial.begin(9600);  // Start serial communication for debugging
  servo.attach(5);  // Attach the servo to pin 5
  servo.write(0);  // Set the servo to the right wall
  delay(1000);  // Wait for a second
  // Wait until something is received
while (Serial.available() == 0) {
  // Do nothing, just wait
  }
}


void loop() {
  oled.print_int(trashCount);
  // Get encoder counts and update the odometry
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
  if (!firstWallFollowed){
    servo.write(0);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed + PIDout;
    int rightSpeed = base_speed - PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
    checkLocation();
    checkFloor();
  }
  else{
    servo.write(180);
    wallDist = sonar.readDist();
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
    checkLocation();
    checkFloor();
  }
  
}







