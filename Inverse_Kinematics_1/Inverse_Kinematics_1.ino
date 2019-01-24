#include <Servo.h>
#include <math.h>

Servo servo1;  // create servo object to control a servo
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servosArray[6] = {servo6, servo5, servo4, servo3, servo2, servo1};

// distance in mm
const int LENGTH_UPPER_ARM = 105;
const int LENGTH_FOREARM = 148;
const int LENGTH_HAND = 180;//not precise
const int LENGTH_GROUND_TO_UPPER_ARM = 190;//not precise, on wooden box

double mapValueIn = 0;
double mapValueOut = 0;
int delayInterval = 20;
int delayMin = 10;
int delayMax = 40;
int cosInterval = 0;//COS movement

void setup() {
  servo1.attach(8);
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);
  servo5.attach(12);
  servo6.attach(13);

  // starting servo positions for resting arm
//  armPos(90, 60, 13, 165, 90, 115);//turn-ready position
  armPos(88, 65, 0, 180, 90, 170);//stand-by position

  Serial.begin(9600);
//  Serial.println( (int)calcServo6_Angle(3, -1) );
//  Serial.println( (int)calcServo6_Angle(3, 0) );
//  Serial.println( (int)calcServo6_Angle(3, 1) );
  
//  Serial.println( calcServo5_Angle(4, 4, 0.9122836) );
//  Serial.println( ( atan2( 4, 4 ) 
//                  - atan2( ( 3 * sin(0.9122836) )
//                  ,( 2 + 3 * cos(0.9122836) ) ) 
//                ) * 180 / M_PI);
//  Serial.println( calcServo4_Angle(197, 210) );
//  Serial.println(  acos(
//                        (
//                        sqrt(pow(4,2)+pow(4,2))
//                        - pow(2,2) 
//                        - pow(3,2)
//                        )
//                        /(2*2*3) 
//                ) * 180 / M_PI );
//  Serial.println( -cos((double) 5 * M_PI / 10) + 1);
}

void loop() {
//  armPos(90, 70, 20, 170, 90, 115);//stand-by position

  delay(1200);
  armTravel(170, 50, 0, 100, 90, 115);//ready to dip
  armTravel(170, 145, 20, 20, 90, 115);//curl up
  armTravel(170, 155, 20, 15, 90, 115);//dip down
  
  armTravel(170, 172, 24, 18, 87, 115);//camera position
  delay(2200);
  armTravel(170, 172, 24, 18, 87, 165);//camera position
  delay(2200);
  
  armTravel(170, 155, 20, 15, 90, 165);//reverse dip
  armTravel(170, 145, 20, 20, 90, 165);//curl up reverse
  armTravel(170, 50, 0, 100, 90, 165);//return
  
  armTravel(88, 0, 0, 150, 90, 165);// travel/carry position
  delay(3200);

  // deposit item
  armTravel(10, 50, 0, 100, 90, 165);//return
  armTravel(10, 145, 20, 20, 90, 165);//curl up reverse
  armTravel(10, 155, 20, 15, 90, 165);//reverse dip
  armTravel(10, 172, 24, 18, 87, 165);//camera position
  delay(2200);
  armTravel(10, 172, 24, 18, 87, 115);//open claw
  delay(2200);

  //return to start empty
  armTravel(10, 155, 20, 15, 90, 115);//reverse dip
  armTravel(10, 145, 20, 20, 90, 115);//curl up reverse
  armTravel(10, 50, 0, 100, 90, 115);//return

  armTravel(88, 65, 20, 180, 90, 115);
  armTravel(88, 65, 0, 180, 90, 170);// REST position
  delay(2200);


  // Movement Demo
//  delay(1000);
//  armTravel(135, 30, 20, 45, 45, 170);
//  delay(1000);
//  armTravel(90, 90, 13, 155, 90, 115);
//  delay(1000);
//  armTravel(45, 30, 20, 90, 90, 170);
//  delay(1000);
//  armTravel(90, 90, 13, 155, 90, 115);

//  delay(1000);
//  armTravel((int)calcServo6_Angle(3,-4), 60, 13, 165, 90, 115);
////  delay(1000);
////  armTravel((int)calcServo6_Angle(3,0), 60, 13, 165, 90, 115);
//  delay(1000);
//  armTravel((int)calcServo6_Angle(3,4), 60, 13, 165, 90, 115);
////  delay(1000);
////  armTravel((int)calcServo6_Angle(3,0), 60, 13, 165, 90, 115);
}

// INVERSE KINEMATICS
// - calculate movement angles of motors based on distance given

// Base rotation
double calcServo6_Angle(double posY, double posX) {
  return atan2( posY, posX ) * 180 / M_PI;  
}

// TODO: TEST FORMULAS
double calcServo4_Angle(double posX, double posY) {
  double angle4 = 90;

  // answer is in radians
  angle4 = acos( 
                ( 
                  sqrt( pow(posX,2) + pow(posY,2) ) 
                  - pow(LENGTH_UPPER_ARM,2) 
                  - pow(LENGTH_FOREARM,2) 
                )
                /
                ( 2*LENGTH_UPPER_ARM*LENGTH_FOREARM ) 
                );
  // convert radians to degrees...optional " 180 - "
  angle4 = angle4 * 180 / M_PI;
  return angle4;
}
// Angle4 has to be in RADIANS
double calcServo5_Angle(double posX, double posY, double angle4) {
  double angle5 = 90;
  angle5 = atan2( posY, posX ) 
          - atan2( ( LENGTH_FOREARM * sin(angle4) )
          ,( LENGTH_UPPER_ARM + LENGTH_FOREARM * cos(angle4) ) );
  return angle5;        
}

// DONE ? // keep gripper parallel to ground 
int calcServo3_Angle(int angle4, int angle5) {
  int angle3 = 90;
  
  if(angle5 + angle4 <= 90){
    angle3 = 180 - angle4 - angle5;  
  }else if (angle5 + angle4 > 90){
    angle3 = -90 + angle4 + angle5;  
  }
return angle3;
}

// FORWARD KINEMATICS
// - takes angles and moves motors to that position
void armTravel(int a6, int a5, int a4, int a3, int a2, int a1){
  
  int newAnglesArray[6] = {a6, a5, a4, a3, a2, a1};// array for new angles
  int biggestAngle = 0;
  int biggestIndex = 0;
  int jointTravel = 0;
  
  for(int n = 0; n < 6; n++){
    // take absolute difference of new and old angles
    jointTravel = abs( newAnglesArray[n] - servosArray[n].read() );
//    Serial.print("jointTravel: ");
//    Serial.print(n);
//    Serial.print(" - ");
//    Serial.print(jointTravel );
//    Serial.println(" ");
    
    if ( jointTravel > biggestAngle ){
//        biggestAngle = newAnglesArray[n];
        biggestAngle = jointTravel;
//        Serial.print("biggestAngle: ");
//        Serial.println(biggestAngle);
        biggestIndex = n; 
    }
  }
  // finalize jointTravel
  jointTravel = abs( newAnglesArray[biggestIndex] - servosArray[biggestIndex].read() );
  
//  Serial.print("FINAL jointTravel: ");
//  Serial.println(jointTravel);
//  Serial.println(" ");

  for(int i = 0; i <= jointTravel; i++){

//    Serial.print("Joint Travel count: ");
//    Serial.println(i);
    
    
      for (int servo = 0; servo < 6; servo++){
        if(newAnglesArray[servo] < servosArray[servo].read() ){
          servosArray[servo].write( servosArray[servo].read() - 1 );  
        } else if (newAnglesArray[servo] > servosArray[servo].read() ){
          servosArray[servo].write( servosArray[servo].read() + 1 );
        }// else change nothing  
      }     
      
    // COS movement    
    mapValueIn = i * 2*M_PI / jointTravel;    
    // inverted cosine translated up by 1
    cosInterval = cos(mapValueIn) + 1 ;
    mapValueOut = cosInterval * (delayMax - delayMin) / 2 + delayMin;  
    delay((int)mapValueOut);
//    delay(delayInterval);
    
  }//end for jointTravel
}
 


// MOVE MOTORS DIRECTLY TO GIVEN POSITION
void armPos(int a6, int a5, int a4, int a3, int a2, int a1){
  servo1.write(a1);
  servo2.write(a2);
  servo3.write(a3);
  servo4.write(a4);
  servo5.write(a5);
  servo6.write(a6);
}
