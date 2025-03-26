#include <Pololu3piPlus32U4.h>
#include "PDcontroller.h" //Uncomment after you import your PDcontroller files

using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 0.25 // tune 0.4   0.25    
#define kd_line 1 // tune 0.005   0.01    

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);

//Recommended Variables

//Calibration
int calibrationSpeed = 100; 
unsigned int lineSensorValues[5];

//Line Following
int lineCenter;
int16_t robotPosition;


void calibrateSensors()
{
  //TASK 2.1a
  //Implement calibration for IR Sensors
  //Hint: Have your robot turn to the left and right to calibrate sensors.

   
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

void followLine() {
    robotPosition = lineSensors.readLineBlack(lineSensorValues);
    Serial.println("Robot Position sensor values:");
    Serial.println(robotPosition);
    lineCenter = 2000; 
    double correction = pd_line.update(robotPosition, lineCenter);
    
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    
    
    motors.setSpeeds(leftSpeed, rightSpeed);
}

void setup() {
  Serial.begin(9600);
  delay(2000);

  calibrateSensors();
  // Serial.println(F("Press any key to start..."));
  //   while (!Serial.available()) {
  //       // Wait for user input
  //   }
  //   Serial.read(); // Clear the input buffer
    delay(10);
  
}

void loop(){

  //Task 2.1b
  //Implement Controller and logic for line following
  //Hint: The actual structure should be similar to you wall following code, 
  //      but instead of sonar you are using the Line Sensors class.
  //Consider making this its own function for easier use in the next lab
  followLine();
    
}
