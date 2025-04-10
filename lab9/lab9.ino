#include <Pololu3piPlus32U4.h>
#include <Servo.h>
#include "sonar.h"
#include "PDcontroller.h" //Uncomment after importing your PDcontroller files
#include "printOLED.h"


using namespace Pololu3piPlus32U4;

LineSensors lineSensors;
Motors motors;
Servo servo;
PrintOLED oled;

Sonar sonar(4);

#define minOutput -100
#define maxOutput 100
#define baseSpeed 100
#define kp_line 0.25 //0.25
#define kd_line 1 // 1
#define kp_obs 25 //25
#define kd_obs 5 //5

PDcontroller pd_line(kp_line, kd_line, minOutput, maxOutput);
PDcontroller pd_obs(kp_obs, kd_obs, minOutput, maxOutput);

//Recommended Variables

//Calibration
int calibrationSpeed = 100;
unsigned int lineSensorValues[5];
unsigned int lineDetectionValues[5];

//Line Following
int lineCenter = 2000;
int16_t robotPosition;
bool isOnBlack;

//Wall Following
int PDout;
float wallDist;
int distFromWall = 10;


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

void setup() {
  Serial.begin(9600);
  servo.attach(5);
  servo.write(90); // turn servo forward
  delay(2000);

  calibrateSensors();
  
}

void loop(){
//start by calculating dist to wall
  wallDist = sonar.readDist();
  oled.print_float(wallDist);

  //if wall is within range start following wall
  if (wallDist<10.0){
    wallFollowing();
  }
  //else follow line
  lineFollowing();
 
    
}


void lineFollowing()
{

  ////////////////////////////////////////prep linefollow state/////////////////////////
  motors.setSpeeds(0,0);
  delay(100);
  servo.write(90);
  /////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////line following state///////////////////////
  while (true){
  robotPosition = lineSensors.readLineBlack(lineSensorValues);
    Serial.println("Robot Position sensor values:");
    Serial.println(robotPosition);
    lineCenter = 2000; 
    double correction = pd_line.update(robotPosition, lineCenter);
    
    int leftSpeed = baseSpeed - correction;
    int rightSpeed = baseSpeed + correction;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
    wallDist = sonar.readDist();
    oled.print_float(wallDist);

    if (wallDist<10.0){
    wallFollowing();
    }
    ////////////////////////////////////////////////////////////////////////////////////
  }
}



// void wallFollowing(){
//     motors.setSpeeds(0,0);
//     delay(1000);
//     servo.write(180);
//     //spin left real quick to find obstacle
//     motors.setSpeeds(50,-50);
//     delay(1500);
//     motors.setSpeeds(50, 50);
//     delay(500);
//     while (true){
//     wallDist = sonar.readDist();
//     double pdout = pd_obs.update(wallDist, distFromWall);
//     int leftSpeed = baseSpeed + pdout;
//     int rightSpeed = baseSpeed - pdout;
    
//     motors.setSpeeds(leftSpeed, rightSpeed);
//     oled.print_float(wallDist);
//     detectBlackLine();
//     }
// }

///////////////////////////////////////wall following prep state////////////////////////////////
void initialWallFollowing(){
  int i;
  for(i=0; i<1000; i++){
    oled.print_int(i);
    wallDist = sonar.readDist();
    double pdout = pd_obs.update(wallDist, distFromWall);
    int leftSpeed = baseSpeed + pdout;
    int rightSpeed = baseSpeed - pdout;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
    //oled.print_float(wallDist);
  }
}

void wallFollowing(){
    motors.setSpeeds(0,0);
    delay(1000);
    servo.write(180);
    //spin left real quick to find obstacle
    motors.setSpeeds(50,-50);
    delay(1500);
    motors.setSpeeds(0,0);
    delay(500);
    initialWallFollowing();
  //////////////////////////////////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////Wall following state/////////////////////////////////////
    while (true){
    wallDist = sonar.readDist();
    double pdout = pd_obs.update(wallDist, distFromWall);
    int leftSpeed = baseSpeed + pdout;
    int rightSpeed = baseSpeed - pdout;
    
    motors.setSpeeds(leftSpeed, rightSpeed);
    //oled.print_float(wallDist); 
    detectBlackLine();
    }
}
  ///////////////////////////////////////////////////////////////////////////////////////////////


void detectBlackLine(){
  lineSensors.read(lineDetectionValues);

    // Threshold value to detect black (adjust based on calibration)
    const int blackThreshold = 2000; 

    // Check if the robot is on a black square
    for (int i = 0; i < 5; i++) {
        Serial.println(lineDetectionValues[i]);
        if (lineDetectionValues[i] > blackThreshold) {
          /////////////////////////////////////////////Detect Line State////////////////////////
            servo.write(90);
            //#TODO If using this function, decide what to do 
            //      if black line is detected again
            motors.setSpeeds(150, 20); // 50, -50
            delay(1000);
            motors.setSpeeds(0,0);
            delay(100);
            lineFollowing();
          /////////////////////////////////////////////////////////////////////////////////////
        }
    }
}

