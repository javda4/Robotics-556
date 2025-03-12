#include <Pololu3piPlus32U4.h>
#include "printOLED.h"
#include "PIDcontroller.h"
#include "PDcontroller.h"
#include "odometry.h" // Uncomment if using odometry

using namespace Pololu3piPlus32U4;

Motors motors;
Encoders encoders;


// Odometry Parameters
#define diaL 3.2
#define diaR  3.2
#define nL 12
#define nR 12
#define w 9.6
#define gearRatio 75
#define DEAD_RECKONING false

// Tune these based on testing
#define minOutput -100
#define maxOutput 100
#define kp 200 // Adjust Kp
#define kd 0.05 // Adjust Kd
#define ki 5.5  // Adjust Ki
#define clamp_i 10  // Adjust integral clamp
#define base_speed 50

//PID - kp 200 , kd 00.05, ki 5.5  //We got 3.13  //Was quick to get to goal state (for pi ) (for pi/2) (for -pi/2)
//PID - kp 50. kd 0.05, ki 5.5 // We got 3.16     ///Appraoched goal state slowly and over shot a little
//PID - kp 150, kd 5, ki 5       We got 3.12       //Approached goal state quickly slight slow down as it got close 

//PD - kp 200, kd 0.05 we get 3.07 //smooth through out. did not make it completely to goal state but lcose enough , excess output power
//PD - kp 50, kd 0.05 we get 2.8 //slower throughout, further from goal state than last. noitcable slowddown toward end. 
//PD - kp 150 , kd 5 we got 3.02 // due to kd being higher we did not achieve as much as we did in the first one , smoothher than the one before


// PD and PID Controllers
PDcontroller pdcontroller(kp, kd, minOutput, maxOutput);
//PIDcontroller pidcontroller(kp, ki, kd, minOutput, maxOutput, clamp_i);
Odometry odometry(diaL, diaR, w, nL, nR, gearRatio, DEAD_RECKONING);
// Given goals in radians
const float goal_theta = 3.14 /-2;  // Example: PI radians

// Odometry variables
int16_t deltaL = 0, deltaR = 0;
int16_t encCountsLeft = 0, encCountsRight = 0;
float x = 0, y = 0 , theta = 0;

// Controller output variables
double PDout, PIDout;

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Read encoder data
  deltaL = encoders.getCountsAndResetLeft();
  deltaR = encoders.getCountsAndResetRight();
  encCountsLeft += deltaL;
  encCountsRight += deltaR;

  // update odometry
  odometry.update_odom(encCountsLeft, encCountsRight, x, y, theta);
  Serial.print("Before PD Out");
  Serial.print("Theta: ");
  Serial.print(theta);
  Serial.print(" PD Output: ");
  Serial.println(PDout);

  //* TASK 2.2 
  double PDout = pdcontroller.update(theta, goal_theta);
  motors.setSpeeds(PDout, -PDout);  // Adjust motor direction if needed
  Serial.print("After PD Out");
  Serial.print("Theta: ");
  Serial.print(theta);
  Serial.print(" PD Output: ");
  Serial.println(PDout);
  //*/



  // /* TASK 3.2 - Utilize PID Controller */

  // double PIDout = pidcontroller.update(theta, goal_theta);
  // motors.setSpeeds(PIDout, -PIDout);  // Adjust motor direction if needed

  // // Print data for debugging
  // Serial.print("Theta: ");
  // Serial.print(theta);
  // Serial.print(" PID Output: ");
  // Serial.println(PIDout);



  delay(100);  // Small delay for stability
}
