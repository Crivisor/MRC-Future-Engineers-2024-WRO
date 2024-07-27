const byte mainSpeed = 25;

#include <Servo.h>
Servo steeringServo;

const byte servoDefaultAngle = 90;
short servoAngle = servoDefaultAngle;
const byte errorMultiplyer = 40;
float error = 0;
bool reading = HIGH;


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
  Serial.println(LeftIR);

  Serial.print("Error: ");
  Serial.println(error);

  ///////////

  Serial.println("----------------------------------------------------------------");
  delay(100);
}

void PID() {
  if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR == reading) {  // Sides Only
    error = 0;
  } else if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR != reading && RightIR != reading) {  // All not reading
    error = 0;
  } else if (LFrontIR != reading && MFrontIR != reading && RFrontIR == reading && LeftIR == reading && RightIR == reading) {  // front R + Side R + Side L
    error = 1;
  } else if (LFrontIR == reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR == reading) {  // front L + Side L + Side R
    error = -1;
  } else if (LFrontIR != reading && MFrontIR != reading && RFrontIR == reading && LeftIR != reading && RightIR == reading) {  // front R + Side R
    error = 1;
  } else if (LFrontIR == reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR != reading) {  // front L + Side L
    error = -2;
  } else if (LFrontIR != reading && MFrontIR == reading && RFrontIR == reading && LeftIR != reading && RightIR == reading) {  // front R + Side R + front M
    error = 2;
  } else if (LFrontIR == reading && MFrontIR == reading && RFrontIR != reading && LeftIR == reading && RightIR != reading) {  // front L + Side L + front M
    error = -2;
  } else if (MFrontIR == reading && RFrontIR == reading && LeftIR != reading && RightIR == reading) {  // front L + front R + front M + Side R
    error = 3;
  } else if (LFrontIR == reading && MFrontIR == reading && LeftIR == reading && RightIR != reading) {  // front L + front R + front M + Side L
    error = -3;
  } else if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR != reading && RightIR == reading) {  // Side R
    error = 3;
  } else if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR != reading) {  // Side L
    error = -3;
  }
  // else if (LFrontIR != reading && MFrontIR == reading && RFrontIR != reading && LeftIR != reading && RightIR != reading) {  // front M
  //   short randomNum = random(0, 1);
  //   if (randomNum == 0) {
  //     randomNum = -1;
  //   }
  //   error = randomNum*2;
  // } else if (LFrontIR == reading && MFrontIR == reading && RFrontIR != reading && LeftIR == reading && RightIR != reading) {  // front M + Side L + front L
  //  short randomNum = random(0, 1);
  //   if (randomNum == 0) {
  //     randomNum = -1;
  //   }
  //   error = randomNum*2;

  // }
  else if (LFrontIR == reading && MFrontIR == reading && RFrontIR != reading && LeftIR == reading && RightIR != reading) {  // front M + Side L
    error = -2;
  } else if (LFrontIR != reading && MFrontIR == reading && RFrontIR == reading && LeftIR != reading && RightIR == reading) {  // front M + Side R + front R
    error = -1;
  } else if (LFrontIR != reading && MFrontIR == reading && RFrontIR != reading && LeftIR != reading && RightIR == reading) {  // front M + Side R
    error = -2;
  }
  // else if (MFrontIR == reading && LeftIR != reading) {  // front M + ! Side L
  //   error = -3;
  // } else if (MFrontIR == reading && RightIR != reading) {  // front M + !Side R
  //   error = 3;
  // }

  // left 160
  // right 20
  servoAngle = round(servoDefaultAngle + (errorMultiplyer * error));
  moveForward(mainSpeed);
  steeringServo.write(servoAngle);
}

char value;
void setup() {
  steeringServo.attach(9);
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





  steeringServo.write(70);
  delay(1000);

  Serial.begin(9600);
}

void loop() {


  LFrontIR = digitalRead(FrontLIR);
  MFrontIR = digitalRead(FrontMIR);
  RFrontIR = digitalRead(FrontRIR);
  LeftIR = digitalRead(SideLIR);
  RightIR = digitalRead(SideRIR);



  PID();

  // Debugging();

  /*
  value = Serial.read();
    }

  if (LFrontIR == HIGH) {
     moveForward(45);
     steeringServo.write(30);
  }
  else{
     moveForward(45);
     steeringServo.write(120);
  }

    if (Serial.available() > 0) {
      value = Serial.read();
    }

    if (value == 'F') {
       moveForward(30);
    }
    else if (value == 'B') {
       moveBackward(30);
    } else if (value == 'R') {
      steeringServo.write(10);
    } else if (value == 'L') {
      steeringServo.write(150);
    }
    else if (value == 'I') {
      steeringServo.write(10);
      moveForward(30);
      delay(5);
      steeringServo.write(90);
    }
    else if (value == 'G') {
      steeringServo.write(150);
      moveForward(30);
      delay(5);
      steeringServo.write(90);
    }
    else if (value == 'J') {
      steeringServo.write(60);
      motor4.run(BACKWARD);
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      delay(5);
      steeringServo.write(110);
    }
    else if (value == 'H') {
      steeringServo.write(160);
      motor4.run(BACKWARD);
      motor1.run(BACKWARD);
      motor2.run(BACKWARD);
      motor3.run(BACKWARD);
      delay(5);
      steeringServo.write(110);
    }


     else if (value == 'H'){
      steeringServo.write(90);
      moveForward(0);


   }

  moveForward(85);
  steeringServo.write(90);
  delay(6000);
      moveBackward(30);
  steeringServo.write(150);
  delay(1500);
  */
}