#include <Pololu3piPlus32U4.h>
#include "my_robot.h"
using namespace Pololu3piPlus32U4;

Motors motors;
int MyRobot::obtain_time(int distance, int speed){
  float actual_speed = speed / 1000.0;  // Convert speed input to m/s
  int time_ms = (distance / actual_speed) * 1000;  // Convert to milliseconds
  delay(10000);
  Serial.println(time_ms);
  while(true);
  return time_ms;
}

long MyRobot::move_time(long start, int time_ms){
  long delta_time = 0;
  long time_ms_long = (long)time_ms;
  while (time_ms_long > delta_time){
      long current = millis();
      delta_time = current - start;
      Serial.println("Start: ");
      Serial.println(start);
      Serial.println("Current: ");
      Serial.println(current);
      Serial.println("Delta_time: ");
      Serial.println(delta_time);
    }
    Serial.println("This is the end of the move time function");
}


MyRobot::MyRobot() {}
//defin moving forward
  void MyRobot::forward(int distance, int speed) {
    int time_ms = obtain_time(distance, speed);
    long start = millis();
//set motor speeds to move forward
    motors.setSpeeds(speed, speed);
    move_time(start, time_ms);
    //long time_ms_long = (long)time_ms;
    // while (time_ms_long > delta_time){
    //   long current = millis();
    //   delta_time = current - start;
    // }
    halt();
}

//define backing up
void MyRobot::backward(int distance, int speed) {
    int time_ms = obtain_time(distance, speed);
  //set motor speeds to back up
    motors.setSpeeds(-speed, -speed);
    delay(time_ms);
    halt();
}

//define spinning left
void MyRobot::spin_left(int duration, int speed) {
    motors.setSpeeds(-speed, speed);
    delay(duration);
    halt();
}
//define spinning right
void MyRobot::spin_right(int duration, int speed) {
    motors.setSpeeds(speed, -speed);
    delay(duration);
    halt();
}
//define turning left
void MyRobot::turn_left(int duration, int speed) {
    motors.setSpeeds(speed, speed + offset);
    delay(duration);
    halt();
}
//define turning right
void MyRobot::turn_right(int duration, int speed) {
    motors.setSpeeds(speed + offset, speed);
    delay(duration);
    halt();
}
//define stopping 
void MyRobot::halt() {
    motors.setSpeeds(0, 0);
}

