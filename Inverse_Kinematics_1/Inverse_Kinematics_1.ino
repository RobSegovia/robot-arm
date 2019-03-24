#include <Servo.h>
#include <math.h>
#include <Wire.h>

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

boolean gripFast = false;
boolean isHorizontal = true;

int wristAngle = 80; // horizontal wrist
int gripAngle = 36; // 36 = fully open; 92 = fully closed


void setup() {

  Serial.begin(9600);

  // starting servo positions for resting arm
  // ** ANGLEs will print in console 1 -> 6, but must be entered 6 -> 1
  analogFeedbackAngles();// put current angles in the array

  // write servos to whatever the current position is
  // ** comment this out for ANGLE reading...make sure ground is connected when using USB
  armPos(
    feedbackAngles[5],
    feedbackAngles[4],
    feedbackAngles[3],
    feedbackAngles[2],
    feedbackAngles[1],
    feedbackAngles[0]
  );

  // initialize the servos
  // ** comment this out for ANGLE reading
  servo1.attach(8);
  servo2.attach(9);
  servo3.attach(10);
  servo4.attach(11);
  servo5.attach(12);
  servo6.attach(13);


  //move the servos to STANDY then REST position
  armTravel(90, 120, 34, 180, wristAngle, gripAngle);

  //  Wire.begin(3); // Setup MEGA with address 3 for I2C
  //  Wire.onReceive(receiveEvent); // ready to register events

}

void receiveEvent(int howMany) {
  char receivedChar;
  while (Wire.available() > 0) {
    receivedChar = Wire.read();
    // TODO - add input, maybe armPaths( receivedChar );

    //    armPaths(receivedChar);


  }
}

// ---- GRIP ITEM ---- gripper starts to close...if it's not advancing (angles are the same)
// then back up a few degrees and stop trying to close the gripper...sensitivity can be set: 2, 3, 4, etc
void gripItem(int sensitivity) {

  int angleArray[56];
  int temp = 0;

  for (int i = 0; i < 56; i++) {

    Serial.println("Index: ");
    Serial.print(i);
    Serial.print("  -  Grip angle: ");
    Serial.print(gripAngle);
    Serial.println("");

    servo1.write(i + 36); //start closing gripper
    gripAngle = i + 36; // set current angle

    gripperAnalogFeedbackAngle(); // read current angle and put in array
    angleArray[i] = feedbackAngles[0]; // add actual feedback angle of gripper to local array

    // exit function if the detected angle is at fully 'closed'
//    if (feedbackAngles[0] >= 92) {
//      gripAngle = 92 - (92 - gripAngle); // back up by some degrees
//      servo1.write(gripAngle);
//
//      Serial.print("exit >=92 >> gripAngle = ");
//      Serial.print(gripAngle);
//
//      return;
//    }

    // check the most recent angle values in the array
    if (i > 8 && i < 56) {

      // check angle difference to 2 angles ago
      temp = angleArray[i] - angleArray[i - 2];

      // if the difference is greater >= 4 then stop squeezing
      if ( temp >= sensitivity ) {

//        gripAngle = gripAngle - sensitivity;

        if (gripAngle < 36) // ensure 'open' angle is not out of bounds
          gripAngle = 36;

        servo1.write(gripAngle);

        return; // stop trying to close the gripper

      }
    }
  }
  return;
}

// open the gripper fully
int ungripItem() {
  gripAngle = 36;
  servo1.write(gripAngle);
}

void armPaths( /* char inByte */ ) {

  // comment out next 3 lines for Wire.h functionality
  if (Serial.available() > 0) {
    char inByte;
    inByte = Serial.read();

    // --------------------------- set the wrist angle VERTICAL or HORIZONTAL
    if (inByte == 'b') { // B for BOTTLE - Horizontal grip
      wristAngle = 80;
      isHorizontal = true;
    }
    else if (inByte == 'm') {// M for MUG - Vertical grip
      wristAngle = 162;
      isHorizontal = false;
    }

    // --------------------------- OPEN or CLOSE the gripper
    if (inByte == 'g') { // CLOSE
      if (isHorizontal) {
        gripItem(4);
      } else {
        gripAngle = 92;
        servo1.write(gripAngle);
      }
    } else if (inByte == 'u') // OPEN
      ungripItem();

    if (inByte == 's') // STANDY
      armTravel(90, 110, 34, 180, wristAngle, gripAngle);//standby (off platform)
    else if (inByte == 'n') // NAP
      armTravel(90, 100, 24, 180, wristAngle, gripAngle);// rest on platform

    else if (inByte == 'c') // CARRY
      armTravel(90, 144, 15, 130, wristAngle, gripAngle);//high carrying GOOD
      
    else if (inByte == 'z') // CARRY L
      armTravel(169, 135, 15, 107, wristAngle, gripAngle);//ready to reach down, RIGHT
    else if (inByte == 'x') // CARRY R
      armTravel(15, 135, 15, 107, wristAngle, gripAngle);//ready to reach down, LEFT

    // ----------------  Positions for picking up object

    // Closest position to robot - 0cm = ~16.5cm from phone
    else if (inByte == '0') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 58, 20, 18, wristAngle, gripAngle);
      } else {
        armTravel(169, 58, 20, 28, wristAngle, gripAngle);
      }

    }
    else if (inByte == '1') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 56, 27, 22, wristAngle, gripAngle);
      } else {
        armTravel(169, 56, 27, 32, wristAngle, gripAngle);
      }

    }
    else if (inByte == '2') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 54, 34, 26, wristAngle, gripAngle);
      } else {
        armTravel(169, 54, 34, 36, wristAngle, gripAngle);
      }

    }
    else if (inByte == '3') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 52, 41, 30, wristAngle, gripAngle);
      } else {
        armTravel(169, 52, 41, 40, wristAngle, gripAngle);
      }

    }

    // MIDDLE pos 5cm
    else if (inByte == '4') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 50, 55, 34, wristAngle, gripAngle);
      } else {
        armTravel(169, 50, 55, 44, wristAngle, gripAngle);
      }

    }

    else if (inByte == '5') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 48, 62, 37, wristAngle, gripAngle);
      } else {
        armTravel(169, 48, 62, 47, wristAngle, gripAngle);
      }

    }
    else if (inByte == '6') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 46, 69, 40, wristAngle, gripAngle);
      } else {
        armTravel(169, 46, 69, 50, wristAngle, gripAngle);
      }

    }
    else if (inByte == '7') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 44, 76, 43, wristAngle, gripAngle);
      } else {
        armTravel(169, 44, 76, 53, wristAngle, gripAngle);
      }

    }
    else if (inByte == '8') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 43, 83, 46, wristAngle, gripAngle);
      } else {
        armTravel(169, 43, 83, 56, wristAngle, gripAngle);
      }

    }

    else if (inByte == '9') {
      if (isHorizontal) {
        armTravel(169, 135, 20, 60, wristAngle, gripAngle);
        armTravel(169, 42, 90, 49, wristAngle, gripAngle);
      } else {
        armTravel(169, 42, 90, 59, wristAngle, gripAngle);
      }

    }

    // DROP position for bottle
    else if (inByte == 'd') {
      if (isHorizontal) {
        armTravel(15, 44, 76, 43, wristAngle, gripAngle);
      } else {
        armTravel(15, 44, 76, 53, wristAngle, gripAngle);
      }

    }

  }// end serial

}


void loop() {

  armPaths(); // Keyboard input for Debugging

  //  armPaths('h');//set horz for bottle
  //  armPaths('z');
  //  armPaths('0');
  //  armPaths('g');
  //  armPaths('z');
  //  armPaths('c');
  //  armPaths('x');
  //  armPaths('D');
  //  armPaths('u');
  //  armPaths('x');
  //  armPaths('s');
  //  delay(2000);
  //
  //  armPaths('v');//set horz for bottle
  //  armPaths('z');
  //  armPaths('0');
  //  gripFast = true;
  //  armPaths('g');
  //  gripFast = false;
  //  armPaths('z');
  //  armPaths('c');
  //  armPaths('x');
  //  armPaths('d');
  //  armPaths('u');
  //  armPaths('x');
  //  armPaths('s');
  //  delay(2000);


  //  analogFeedbackAngles();
  //  delay(2000);

}


// FORWARD KINEMATICS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// - takes angles and moves motors to that position
void armTravel(int a6, int a5, int a4, int a3, int a2, int a1) {

  int newAnglesArray[6] = {a6, a5, a4, a3, a2, a1};// array for new angles
  int biggestAngle = 0;
  int biggestIndex = 0;
  int jointTravel = 0;

  for (int n = 0; n < 6; n++) {
    // take absolute difference of new and old angles
    jointTravel = abs( newAnglesArray[n] - servosArray[n].read() );

    if ( jointTravel > biggestAngle ) {
      biggestAngle = jointTravel;
      biggestIndex = n;
    }
  }
  // finalize jointTravel
  jointTravel = abs( newAnglesArray[biggestIndex] - servosArray[biggestIndex].read() );

  for (int i = 0; i <= jointTravel; i++) {

    for (int servo = 0; servo < 6; servo++) {
      if (newAnglesArray[servo] < servosArray[servo].read() ) {
        servosArray[servo].write( servosArray[servo].read() - 1 );
      } else if (newAnglesArray[servo] > servosArray[servo].read() ) {
        servosArray[servo].write( servosArray[servo].read() + 1 );
      }// else change nothing
    }

    // COSINE movement
    mapValueIn = i * 2 * M_PI / jointTravel;
    // inverted cosine wave translated up by 1
    cosInterval = cos(mapValueIn) + 1 ;
    mapValueOut = cosInterval * (delayMax - delayMin) / 2 + delayMin;
    delay((int)mapValueOut);

  }//end for jointTravel
}
//  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~





void analogFeedbackAngles() {

  // 500 readings ~ 100ms
  int num = 500;
  int analogReadings[num];

  long int voltage = 0;
  int angle = 0;

  for (int n = 1; n < 7; n++) {

    for (int i = 0; i < num; i++) {
      analogReadings[i] = analogRead(n);
    }

    for (int k = 0; k < num; k++) {
      voltage = voltage + analogReadings[k];
    }
    voltage = voltage / num;

    // convert voltage to angle using manually calibrated values
    if (n == 1) {
      angle = map(voltage, 218, 326, 42, 92);
    } else if (n == 2) {
      angle = map(voltage, 145, 497, 10, 180);
    } else if (n == 3) {
      angle = map(voltage, 161, 492, 15, 170);
    } else if (n == 4) {
      angle = map(voltage, 56, 608, 0, 180);
    } else if (n == 5) {
      angle = map(voltage, 160, 514, 33, 150);
    } else if (n == 6) {
      angle = map(voltage, 159, 488, 15, 170);
    }

    feedbackAngles[n - 1] = angle;

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

void gripperAnalogFeedbackAngle() {

  // 500 readings ~ 100ms
  int num = 2000;
  int analogReadings[num];

  long int voltage = 0;
  int angle = 0;

  // take num samples
  for (int i = 0; i < num; i++) {
    analogReadings[i] = analogRead(1);
  }

  //take average of all samples
  for (int k = 0; k < num; k++) {
    voltage = voltage + analogReadings[k];
  }
  voltage = voltage / num;

  // convert voltage to angle using manually calibrated values
  angle = map(voltage, 218, 326, 42, 92);

  //store angle in global angle array
  feedbackAngles[0] = angle;

  Serial.print("Gripper --   Voltage: ");
  Serial.print(voltage);
  Serial.print("   Angle: ");
  Serial.print(angle);
  Serial.println(" ");

}

// MOVE MOTORS DIRECTLY TO GIVEN POSITION
void armPos(int a6, int a5, int a4, int a3, int a2, int a1) {
  servo1.write(a1);
  servo2.write(a2);
  servo3.write(a3);
  servo4.write(a4);
  servo5.write(a5);
  servo6.write(a6);
}




// -----------------------  INVERSE KINEMATICS
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
               sqrt( pow(posX, 2) + pow(posY, 2) )
               - pow(LENGTH_UPPER_ARM, 2)
               - pow(LENGTH_FOREARM, 2)
             )
             /
             ( 2 * LENGTH_UPPER_ARM * LENGTH_FOREARM )
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
                    , ( LENGTH_UPPER_ARM + LENGTH_FOREARM * cos(angle4) ) );
  return angle5;
}

// DONE ? // keep gripper parallel to ground
int calcServo3_Angle(int angle4, int angle5) {
  int angle3 = 90;

  if (angle5 + angle4 <= 90) {
    angle3 = 180 - angle4 - angle5;
  } else if (angle5 + angle4 > 90) {
    angle3 = -90 + angle4 + angle5;
  }
  return angle3;
}
