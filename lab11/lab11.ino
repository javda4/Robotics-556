#include <Pololu3piPlus32U4.h>
#include <Pololu3piPlus32U4Buttons.h>
using namespace Pololu3piPlus32U4;
#include "particle_filter.h" 
#include "odometry.h"
//#include "Map.h"
#include "PIDcontroller.h"
#include <Servo.h>
#include "sonar.h"
#include <PololuBuzzer.h>

PololuBuzzer buzzer;

const char fugue[] PROGMEM = "..."; // music string cut for brevity

#define PI 3.14159
#define lenOfMap 36 // 4 rows * 9 columns
#define N_particles 25
#define move_noise  0.01
#define rotate_noise 0.5
#define ultra_noise 0.1

#define diaL 3.2
#define diaR 3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define deadreckoning false

#define minOutputAng -100
#define maxOutputAng 100
#define kpAng 200
#define kdAng 0.05
#define kiAng 5
#define clamp_iAng 50
#define base_speedAng 50

Motors motors;
Servo servo;
Encoders encoders;
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, deadreckoning);
ParticleFilter particle(lenOfMap, N_particles, move_noise, rotate_noise, ultra_noise, 0);
PIDcontroller pidAng(kpAng, kiAng, kdAng, minOutputAng, maxOutputAng, clamp_iAng);

//Map map = Map();

int16_t deltaL=0, deltaR=0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x, y, theta;
double PIDout_theta;
float x_last = 0.0, y_last = 0.0, theta_last = 0.0;
const int ROWS = 4;
const int COLS = 9;
static bool visited[ROWS][COLS] = {false};

const int distance = 1500;


struct Distances {
  int front;
  int left;
  int right;
};



Distances checkSurroundings() {
  Distances d;
  servo.write(90); delay(1000); d.front = sonar.readDist(); delay(1000);
  Serial.print("Front:");
  Serial.println(d.front);
  servo.write(180); delay(1000); d.left = sonar.readDist(); delay(1000);
  Serial.print("Left:");
  Serial.println(d.left);
  servo.write(0); delay(1000); d.right = sonar.readDist(); delay(1000);
  Serial.print("Right:");
  Serial.println(d.right);
  servo.write(90); delay(1000);
  return d;
}



void rotateTo(float target_angle) {
  while (true) {
    deltaL = encoders.getCountsAndResetLeft();
    deltaR = encoders.getCountsAndResetRight();
    encCountsLeft += deltaL;
    encCountsRight += deltaR;
    odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
    PIDout_theta = pidAng.update(theta, target_angle);
    motors.setSpeeds(PIDout_theta, -PIDout_theta);
    if (abs(target_angle - theta) < 0.1) {
      motors.setSpeeds(0, 0);
      break;
    }
  }
}



void moveForward(int duration, int speed) {
  motors.setSpeeds(speed, speed);
  delay(duration);
  motors.setSpeeds(0, 0);
}



void printAccessibleSquares() {
  Serial.println("Accessible squares:");
  for (int i = 0; i < ROWS; ++i) {
    for (int j = 0; j < COLS; ++j) {
      if (visited[i][j]) {
        Serial.print("("); Serial.print(i); Serial.print(", "); Serial.print(j); Serial.println(")");
      }
    }
  }
}




int i = 0;
void exploreDFS() {
  static int stack[lenOfMap][2];
  static int top = -1;

  int currX = (int)(x);
  int currY = (int)(y);

  if (currX < 0 || currX >= ROWS || currY < 0 || currY >= COLS || visited[currX][currY]) return;
  Serial.print(i);
  i+=1;
  visited[currX][currY] = true;
  stack[++top][0] = currX;
  stack[top][1] = currY;

  Distances d = checkSurroundings();

  // Move forward in the current direction and increase the column (X) if front is clear
  if (d.front > 24 && currY + 1 < COLS && !visited[currX][currY + 1]) {
    moveForward(distance, 100); // Move forward in the current direction
    exploreDFS(); // Explore further
    rotateTo(theta + PI); moveForward(distance, 100); rotateTo(theta + PI); // Backtrack
  }

  // Turn right 90 degrees (clockwise) and move forward to increase row (Y)
  if (d.left > 24 && currX + 1 < ROWS && !visited[currX + 1][currY]) {
    rotateTo(theta + (PI / 2)-0.4); // Turn right 90 degrees
    moveForward(distance, 100);   // Move forward after turning
    exploreDFS();             // Explore further
    rotateTo(theta + PI); moveForward(distance, 100); rotateTo(theta + PI); // Backtrack
  }

  // Turn left 90 degrees (counterclockwise) and move forward to increase column (X) again
  if (d.right > 24 && currX - 1 >= 0 && !visited[currX - 1][currY]) {
    Serial.println("ENTERED RIGHT!!!");
    moveForward(distance,100);
    moveForward(distance,-100);
    rotateTo(theta - (PI / 2)); // Turn left 90 degrees
    moveForward(distance, 100);   // Move forward after turning
    exploreDFS();             // Explore further
    rotateTo(theta + PI); moveForward(distance, 100); rotateTo(theta + PI); // Backtrack
  }

  top--;
}








void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90);
  delay(1000);
}



void loop() {
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
  float dx = x - x_last, dy = y - y_last, dtheta = theta - theta_last;
  x_last = x; y_last = y; theta_last = theta;

  exploreDFS();
  printAccessibleSquares();
  Serial.print("HOWDY!!!!!!!");
  delay(1000);
}
