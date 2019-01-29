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
const int LENGTH_GROUND_TO_UPPER_ARM = 190;// on wooden box

double mapValueIn = 0;
double mapValueOut = 0;
int delayInterval = 20;
int delayMin = 10;
int delayMax = 40;
int cosInterval = 0;//COS movement
int feedbackAngles[6];

void setup() {
  Serial.begin(9600);

  // set pin 13 as output
  pinMode(5, OUTPUT);
  digitalWrite(13, LOW);
  
  // starting servo positions for resting arm
  analogFeedbackAngles();// put current angles in the array

  //print angles to console
  for(int i=0;i<6;i++){
    Serial.println(feedbackAngles[i]);
  }

  // write servos to whatever the current position is
  armPos(
    feedbackAngles[5],
    feedbackAngles[4],
    feedbackAngles[3],
    feedbackAngles[2],
    feedbackAngles[1],
    feedbackAngles[0]
    );

  // initialize the servos
  servo1.attach(8);
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);
  servo5.attach(12);
  servo6.attach(13);

  //move the servos to STANDBY position
  armTravel(90, 115, 0, 180, 90, 42);

  

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
//analogFeedbackAngles();

  // turn on the lasers
  digitalWrite(5, HIGH);
  delay(4000);
  digitalWrite(5, LOW);
  delay(2000);


//  delay(500);
//  armTravel(90, 115, 0, 180, 90, 42);
//  delay(500);
//  armTravel(90, 115, 0, 180, 90, 92);

//  delay(1500);

}

void analogFeedbackAngles(){

  // 500 readings ~ 100ms
  int num = 500;
  int analogReadings[num];

  long int voltage = 0;
  int angle = 0;
  
  for(int n = 1; n < 7; n++){
    
    for(int i = 0; i < num; i++){
      analogReadings[i] = analogRead(n);
    }

    for(int k = 0; k < num; k++){
      voltage = voltage + analogReadings[k];
  //    Serial.println(analogReadings[k]);   
    }
    voltage = voltage / num;
    
    // convert voltage to angle using manually calibrated values
    if(n==1){
      angle = map(voltage, 218, 326, 42, 92);
    }else if(n==2){
      angle = map(voltage, 145, 497, 10, 180);
    }else if(n==3){
      angle = map(voltage, 161, 492, 15, 170);
    }else if(n==4){
      angle = map(voltage, 56, 608, 0, 180);
    }else if(n==5){
      angle = map(voltage, 160, 514, 33, 150);
    }else if(n==6){
      angle = map(voltage, 159, 488, 15, 170);
    }

    feedbackAngles[n-1] = angle;

    Serial.print("Servo #");
    Serial.print(n);
    Serial.print("   Voltage: ");
    Serial.print(voltage);
    Serial.print("   Angle: ");
    Serial.print(angle);
    Serial.println(" ");
  }
  
  Serial.println(" ");
}

void demoMove(){
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
