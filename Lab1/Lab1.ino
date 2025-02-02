#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;


MyRobot robot;
void led(){
  for (int i = 1; i <= 5; i++) {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(50);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(50);
  }                    // wait for a second
}

void setup() {
    Serial.begin(9600);
    delay(15);
    pinMode(LED_BUILTIN, OUTPUT);
}
//test the movements of the robot
void loop() {
  //create speed outputs in both positive and negative directions
  int speed = 300;
  int duration = 3000;
  int distance = 1;

  //delay to give time to put robot down before it starts
  delay(duration);
  
  //move forward
    robot.forward(distance, speed);
    led();
    delay(3000);
  //spin right
    robot.spin_right(duration, speed);
    led();
    delay(3000);
  //spin left
    robot.spin_left(duration, speed);
    led();
    delay(3000);
  //move backward
    robot.backward(distance, speed);
    led();
    delay(3000);
  //move forward and right
    robot.turn_right(duration, speed);
    led();
    delay(3000);
  //move forward and right
    robot.turn_left(duration, speed);
    led();
    delay(3000);
  //stop robot motion
    robot.halt();
    led();
    led();
    led();
    delay(1000);

  // //Initial Step Using INO file and set speeds before modularizing 
  // Motors motors;
  // int left_speed = 75;
  // int right_speed = 75;
  // int delay_time_ms = 4000;
  // int default_speed = 75;
  // int speed_offset = 75;
  // int halt_speed = 0;
  
  // //delay initial action to have time to put robot down
  // delay(delay_time_ms);

  // //move forward
  // motors.setSpeeds(left_speed, right_speed); 
  // delay(delay_time_ms); //have move forward execute for delay_time_ms duration

  // //move backwards
  // motors.setSpeeds(left_speed * -1, right_speed * -1); 
  // delay(delay_time_ms);
  // left_speed = default_speed;
  // right_speed = default_speed; // reset left and right speed to its default value for next operation

  // //spin left 
  // motors.setSpeeds(left_speed * -1, right_speed);
  // delay(delay_time_ms);
  // left_speed = default_speed; // reset left speed to its default value for next operation

  // //spin right
  // motors.setSpeeds(left_speed, right_speed * -1);
  // delay(delay_time_ms);
  // right_speed = default_speed; // reset right speed to its default value for the next operation

  // //move forward veer left 
  // motors.setSpeeds(left_speed, right_speed + speed_offset); 
  // delay(delay_time_ms);
  // right_speed = default_speed; //reset right speed to its default value for next operation

  // // move forward veer right 
  // motors.setSpeeds(left_speed + speed_offset, right_speed); 
  // delay(delay_time_ms);
  // left_speed = default_speed; // resert left speed to its default value for next operation

  // // halt
  // motors.setSpeeds(halt_speed, halt_speed);
  // delay(delay_time_ms);
}



