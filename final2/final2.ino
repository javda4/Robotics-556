#include <Pololu3piPlus32U4.h>  // Pololu 3pi+ library for motor and encoder control
#include <Pololu3piPlus32U4Buttons.h>  // Button control library for 3pi+
using namespace Pololu3piPlus32U4;
#include "particle_filter.h"  // Particle filter library for localization
#include "odometry.h"  // Odometry library to calculate position
//#include "Map.h"  // Commented out map library, not in use
#include "PDcontroller.h"  // PID controller library for movement control
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



// Create objects for motors, servos, encoders, and odometry
Motors motors;
Servo servo;
Encoders encoders;
PrintOLED oled;
LineSensors lineSensors;  // Add this with other sensor/motor declarations
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, deadreckoning);  // Initialize odometry object


PDcontroller pidVel(kpVel, kdVel, minOutputVel, maxOutputVel);

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
int calibrationSpeed = 100;
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
struct TrashLocation {
  float x;
  float y;
};

const int maxTrash = 3;
TrashLocation foundTrash[maxTrash]; // Stores trash positions
int trashCount = 0;
float ignoreRadius = 20.0; // Distance threshold






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
  // Add more corners if needed
};

TurnState turnState; // Global instance



// Struct to hold distances from sonar sensor
struct Distances {
  int front;
  int left;
  int right;
};




void calibrateSensors()
{
  //calibrate sensors function
  Serial.println(F("Calibrating..."));
    delay(1000);
    
    for (int i = 0; i < 50; i++) {
        motors.setSpeeds(-calibrationSpeed, calibrationSpeed); 
        lineSensors.calibrate();
        delay(10);
        
    }
    for (int i = 0; i < 50; i++) { // potential 200 to account for previous off set?
        motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
        lineSensors.calibrate();
        delay(10);
    }
    motors.setSpeeds(0, 0);
    
    Serial.println(F("Calibration Done"));
    delay(1000);
}







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
  motors.setSpeeds(-250,250);
  delay(250);
  motors.setSpeeds(0,0);
}

void spinRight(){
  motors.setSpeeds(250,-250);
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
    motors.setSpeeds(250,-250);
    delay(1000);
  
  //spin back same amount
    motors.setSpeeds(-250,250);
    delay(1000);
  
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
  firstWallFollowed = true;
}




void checkLocation(){
  if (x > 69 && y < 20 && !turnState.outer1_done){
    spinLeft();
    turnState.outer1_done = true;
  }
  if(x > 60 && y > 169 && !turnState.outer2_done){
    spinLeft();
    turnState.outer2_done = true;
  }
  if(x < 18 && y > 160 && !turnState.outer3_done){
    spinLeft();
    turnState.outer3_done = true;
  }
  if(x < 20 && y > 80 && y < 92 && !turnState.outer4_done){
    spinLeft();
    turnState.outer4_done = true;
  }
  if(x < 12 && y > 20 && y < 40 && !turnState.outer5_done){
    spinLeft();
    turnState.outer5_done = true;
  }
  if(firstWallFollowed && x < 40 && x > 20 && y > 100 && y < 120 && !turnState.inner1_done){
    spinRight();
    turnState.inner1_done = true;
  }
  if(firstWallFollowed && x < 30 && x > 20 && y > 130 && y < 140 && !turnState.inner2_done){
    spinRight();
    turnState.inner2_done = true;
  }
  if(firstWallFollowed && x > 45 && x < 60 && y > 140 && y < 160 && !turnState.inner3_done){
    spinRight();
    turnState.inner3_done = true;
  }
  if(firstWallFollowed && x > 45 && x < 60 && y > 140 && y < 145 && !turnState.inner4_done){
    spinRight();
    turnState.inner4_done = true;
  }
  if(firstWallFollowed && x > 45 && x < 60 && y > 80 && y < 100 && !turnState.inner5_done){
    spinRight();
    turnState.inner5_done = true;
  }
  if(firstWallFollowed && x > 40 && x < 55 && y > 20 && y < 30 && !turnState.inner6_done){
    spinRight();
    turnState.inner6_done = true;
  }
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
  calibrateSensors();
  motors.setSpeeds(100,100);
  delay(1500);
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
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
    //Serial.println("outerwall");
    Serial.println("x: ");
    Serial.println(x);
    Serial.println("y: ");
    Serial.println(y);
    servo.write(0);
    wallDist = sonar.readDist();
    Serial.println(wallDist);
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed - PIDout;
    int rightSpeed = base_speed + PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
    checkLocation();
    checkFloor();
  }
  else{
    //Serial.println("innerwall");
    servo.write(180);
    wallDist = sonar.readDist();
    Serial.println(wallDist);
    PIDout = pidVel.update(wallDist, desiredDistFromWall);
    int leftSpeed = base_speed + PIDout;
    int rightSpeed = base_speed - PIDout;
    motors.setSpeeds(leftSpeed, rightSpeed);
    checkLocation();
    checkFloor();
  }
  
}







