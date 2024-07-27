const byte mainSpeed = 36;

#include <Servo.h>
Servo name_servo;

int servo_position = 0;


const byte FrontLIR = A5;
const byte FrontMIR = A4;
const byte FrontRIR = A2;
const byte SideLIR = A3;
const byte SideRIR = 11;

bool LFrontIR;
bool MFrontIR;
bool RFrontIR;
bool LeftIR;
bool RightIR;


const byte rightMotorP = 6;
const byte rightMotor1 = 5;
const byte rightMotor2 = 4;
const byte leftMotorP = 3;


void moveForward(int Speed = 0) {

  digitalWrite(rightMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);


  analogWrite(leftMotorP, Speed);
  analogWrite(rightMotorP, Speed);
}
void moveBackward(int Speed = 0) {

  digitalWrite(rightMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);

  analogWrite(leftMotorP, Speed);
  analogWrite(rightMotorP, Speed);
}






void Debugging() {

  Serial.print("RFrontIR: ");
  Serial.print(RFrontIR);
  Serial.print(" LFrontIR: ");
  Serial.print(LFrontIR);
  Serial.print(" MFrontIR: ");
  Serial.print(MFrontIR);

  Serial.print(" RightIR: ");
  Serial.print(RightIR);
  Serial.print(" LeftIR: ");
  Serial.print(LeftIR);

  ///////////

  Serial.println("----------------------------------------------------------------");
}



char value;
void setup() {
  name_servo.attach(9);
  Serial.begin(9600);
  pinMode(rightMotorP, OUTPUT);
  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotorP, OUTPUT);


  pinMode(FrontLIR, INPUT_PULLUP);
  pinMode(SideLIR, INPUT_PULLUP);

  pinMode(FrontMIR, INPUT_PULLUP);

  pinMode(FrontRIR, INPUT_PULLUP);
  pinMode(SideRIR, INPUT_PULLUP);





  name_servo.write(70);
  delay(1000);

  Serial.begin(9600);
}
void loop() {


  LFrontIR = digitalRead(FrontLIR);
  MFrontIR = digitalRead(FrontMIR);
  RFrontIR = digitalRead(FrontRIR);
  LeftIR = digitalRead(SideLIR);
  RightIR = digitalRead(SideRIR);


  // Debugging();

  if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == LOW && LeftIR == HIGH && RightIR == HIGH) {  // Sides Only
    moveForward(mainSpeed);
    name_servo.write(90);

  } else if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == HIGH && LeftIR == HIGH && RightIR == HIGH) {  // front R + Side R
    moveForward(mainSpeed);
    name_servo.write(120);

  } else if (LFrontIR == HIGH && MFrontIR == LOW && RFrontIR == LOW && LeftIR == HIGH && RightIR == HIGH) {  // front L + Side L
    moveForward(mainSpeed);
    name_servo.write(30);

  } else if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == LOW && LeftIR == HIGH && RightIR == LOW) {  // Side L
    moveForward(mainSpeed);
    name_servo.write(35);

  } else if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == LOW && LeftIR == LOW && RightIR == HIGH) {  // Side R
    moveForward(mainSpeed);
    name_servo.write(130);

  } else if (LFrontIR == HIGH && MFrontIR == LOW && RFrontIR == LOW && LeftIR == HIGH && RightIR == LOW) {  // front L + front M
    moveForward(mainSpeed);
    name_servo.write(10);

  } else if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == HIGH && LeftIR == LOW && RightIR == HIGH) {  // front R + front M
    moveForward(mainSpeed);
    name_servo.write(150);

  } else if (LFrontIR == HIGH && MFrontIR == HIGH && RFrontIR == HIGH && LeftIR == LOW && RightIR == LOW) {  // front R + front M + front L
    moveForward(0);
    name_servo.write(90);

  } 
  // else if (LFrontIR == LOW && MFrontIR == HIGH && RFrontIR == HIGH && LeftIR == LOW && RightIR == HIGH) {  // front R + front M + side R
  //   moveForward(mainSpeed);
  //   name_servo.write(160);

  // } else if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == LOW && LeftIR == LOW && RightIR == LOW) {  // front L + front M + side L
  //   moveForward(mainSpeed);
  //   name_servo.write(10);

  // } else if (LFrontIR == HIGH && MFrontIR == HIGH && RFrontIR == HIGH && LeftIR == HIGH && RightIR == HIGH) {  // All Reading
  //   moveForward(0);
  //   name_servo.write(90);

  // } else if (LFrontIR == LOW && MFrontIR == LOW && RFrontIR == LOW && LeftIR == LOW && RightIR == LOW) {  // All Not Reading
  //   moveForward(mainSpeed);
  //   name_servo.write(90);
  // }

  // value = Serial.read();
  //   // }

  // if (LFrontIR == HIGH) {
  //    moveForward(45);
  //    name_servo.write(30);
  // }
  // else{
  //    moveForward(45);
  //    name_servo.write(120);
  // }

  //   // if (Serial.available() > 0) {
  //     value = Serial.read();
  //   // }

  //   if (value == 'F') {
  //      moveForward(30);
  //   }
  //   else if (value == 'B') {
  //      moveBackward(30);
  //   } else if (value == 'R') {
  //     name_servo.write(10);
  //   } else if (value == 'L') {
  //     name_servo.write(150);
  //   }
  //   else if (value == 'I') {
  //     name_servo.write(10);
  //     moveForward(30);
  //     delay(5);
  //     name_servo.write(90);
  //   }
  //   else if (value == 'G') {
  //     name_servo.write(150);
  //     moveForward(30);
  //     delay(5);
  //     name_servo.write(90);
  //   }
  // //   else if (value == 'J') {
  // //     name_servo.write(60);
  // //     motor4.run(BACKWARD);
  // //     motor1.run(BACKWARD);
  // //     motor2.run(BACKWARD);
  // //     motor3.run(BACKWARD);
  // //     delay(5);
  // //     name_servo.write(110);
  // //   }
  // //   else if (value == 'H') {
  // //     name_servo.write(160);
  // //     motor4.run(BACKWARD);
  // //     motor1.run(BACKWARD);
  // //     motor2.run(BACKWARD);
  // //     motor3.run(BACKWARD);
  // //     delay(5);
  // //     name_servo.write(110);
  // //   }


  //    else if (value == 'H'){
  //     name_servo.write(90);
  //     moveForward(0);


  //  }

  // moveForward(85);
  // name_servo.write(90);
  // delay(6000);
  //     moveBackward(30);
  // name_servo.write(150);
  // delay(1500);
}
