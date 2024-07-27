### Overview

My Arduino code controls a robot equipped with IR sensors and a servo motor for steering. It is designed to move the robot forward or backward based on IR sensor inputs, using a PID-like approach for steering correction.

### Global Variables and Constants

1. **`const byte mainSpeed = 25;`**
   - This constant defines the speed at which the robot moves forward or backward. It is used as the default speed setting for the motors.

2. **Servo and Angle Settings:**
   - **`#include <Servo.h>`**: This line includes the Servo library, which provides functions to control a servo motor.
   - **`Servo steeringServo;`**: This creates an instance of the Servo class to control the robot’s steering mechanism.
   - **`const byte servoDefaultAngle = 90;`**: The default angle for the servo when the robot is moving straight. The servo angle ranges from 0 to 180 degrees.
   - **`short servoAngle = servoDefaultAngle;`**: The current angle of the servo, initialized to the default angle.
   - **`const byte errorMultiplyer = 40;`**: This multiplier affects how much the `error` value will adjust the servo angle. It determines the sensitivity of the steering correction.
   - **`float error = 0;`**: The error value used to calculate the necessary steering adjustment.
   - **`bool reading = HIGH;`**: The expected state of the IR sensors when no obstacle is detected. HIGH means no obstacle detected, LOW means obstacle detected.

3. **IR Sensor Pins:**
   - **`const byte FrontLIR = A5;`**
   - **`const byte FrontMIR = A4;`**
   - **`const byte FrontRIR = A2;`**
   - **`const byte SideLIR = A3;`**
   - **`const byte SideRIR = 11;`**
   - These constants define the analog and digital pins connected to the IR sensors. The pins are used to read sensor values that determine if obstacles are present.

4. **IR Sensor States:**
   - **`bool LFrontIR;`**
   - **`bool MFrontIR;`**
   - **`bool RFrontIR;`**
   - **`bool LeftIR;`**
   - **`bool RightIR;`**
   - These boolean variables store the current readings from the IR sensors.

5. **Motor Control Pins:**
   - **`const byte rightMotorP = 6;`**
   - **`const byte rightMotor1 = 5;`**
   - **`const byte rightMotor2 = 4;`**
   - **`const byte leftMotorP = 3;`**
   - These constants define the pins connected to the motor driver. The pins control the direction and speed of the motors.

### Functions

1. **`void moveForward(int Speed = 0)`**
   - This function sets the robot to move forward.
   - **`digitalWrite(rightMotor2, HIGH);`** and **`digitalWrite(rightMotor1, LOW);`**: Sets the direction for the right motor.
   - **`analogWrite(leftMotorP, Speed);`** and **`analogWrite(rightMotorP, Speed);`**: Sets the speed for both motors using PWM.

2. **`void moveBackward(int Speed = 0)`**
   - This function sets the robot to move backward.
   - **`digitalWrite(rightMotor2, LOW);`** and **`digitalWrite(rightMotor1, HIGH);`**: Reverses the direction for the right motor.
   - **`analogWrite(leftMotorP, Speed);`** and **`analogWrite(rightMotorP, Speed);`**: Sets the speed for both motors using PWM.

3. **`void Debugging()`**
   - This function outputs the current state of the IR sensors and the `error` value to the Serial Monitor.
   - **`Serial.print()`** statements print the status of each IR sensor and the calculated `error` to help with monitoring and debugging.
   - **`delay(100);`**: Adds a small delay to avoid flooding the Serial Monitor with data.

4. **`void PID()`**
   - This function calculates the `error` based on IR sensor readings and adjusts the steering angle of the servo.
   - **Error Scenarios:**
     - **`if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR == reading)`**: All front sensors are inactive (no obstacle detected), and side sensors are active. Sets `error` to 0.
     - **`else if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR != reading && RightIR != reading)`**: All sensors inactive (obstacle detected all around). Sets `error` to 0.
     - **`else if (LFrontIR != reading && MFrontIR != reading && RFrontIR == reading && LeftIR == reading && RightIR == reading)`**: Front right and side sensors active. Sets `error` to 1, indicating a need for right steering.
     - **`else if (LFrontIR == reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR == reading)`**: Front left and side sensors active. Sets `error` to -1, indicating a need for left steering.
     - **`else if (LFrontIR != reading && MFrontIR != reading && RFrontIR == reading && LeftIR != reading && RightIR == reading)`**: Front right and side right sensors active. Sets `error` to 1.
     - **`else if (LFrontIR == reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR != reading)`**: Front left, side left sensors active. Sets `error` to -2.
     - **`else if (LFrontIR != reading && MFrontIR == reading && RFrontIR == reading && LeftIR != reading && RightIR == reading)`**: Front right, middle, and side right sensors active. Sets `error` to 2.
     - **`else if (LFrontIR == reading && MFrontIR == reading && RFrontIR != reading && LeftIR == reading && RightIR != reading)`**: Front left, middle, and side left sensors active. Sets `error` to -2.
     - **`else if (MFrontIR == reading && RFrontIR == reading && LeftIR != reading && RightIR == reading)`**: Front middle, right, and side right sensors active. Sets `error` to 3.
     - **`else if (LFrontIR == reading && MFrontIR == reading && LeftIR == reading && RightIR != reading)`**: Front middle, left, and side left sensors active. Sets `error` to -3.
     - **`else if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR != reading && RightIR == reading)`**: Only side right sensor active. Sets `error` to 3.
     - **`else if (LFrontIR != reading && MFrontIR != reading && RFrontIR != reading && LeftIR == reading && RightIR != reading)`**: Only side left sensor active. Sets `error` to -3.
   - **`servoAngle = round(servoDefaultAngle + (errorMultiplyer * error));`**: Calculates the new angle for the servo based on the `error` and `errorMultiplyer`. The `round()` function ensures the angle is an integer.
   - **`moveForward(mainSpeed);`**: Commands the robot to move forward at the default speed.
   - **`steeringServo.write(servoAngle);`**: Sets the servo to the calculated angle for steering adjustment.

### Setup and Loop Functions

1. **`void setup()`**
   - **`steeringServo.attach(9);`**: Attaches the servo control to pin 9.
   - **`Serial.begin(9600);`**: Initializes serial communication at 9600 baud rate for debugging.
   - **`pinMode(rightMotorP, OUTPUT);`**, **`pinMode(rightMotor1, OUTPUT);`**, **`pinMode(rightMotor2, OUTPUT);`**, **`pinMode(leftMotorP, OUTPUT);`**: Sets the motor control pins as outputs.
   - **`pinMode(FrontLIR, INPUT_PULLUP);`**, **`pinMode(SideLIR, INPUT_PULLUP);`**, **`pinMode(FrontMIR, INPUT_PULLUP);`**, **`pinMode(FrontRIR, INPUT_PULLUP);`**, **`pinMode(SideRIR, INPUT_PULLUP);`**: Sets the IR sensor pins as inputs with pull-up resistors.
   - **`steeringServo.write(70);`**: Moves the servo to an initial angle of 70 degrees.
   - **`delay(1000);`**: Waits for one second.

2. **`void loop()`**
   - **`LFrontIR = digitalRead(FrontLIR);`**
   - **`MFrontIR = digitalRead(FrontMIR);`**
   - **`RFrontIR = digitalRead(FrontRIR);`**
   - **`LeftIR = digitalRead(SideLIR);`**
   -

 **`RightIR = digitalRead(SideRIR);`**
   - These lines read the current state of each IR sensor and store the values in their respective boolean variables.
   - **`PID();`**: Calls the `PID()` function to calculate the error and adjust steering based on sensor inputs.
   - **`// Debugging();`**: Commented out; if uncommented, it would print sensor states and error values to the Serial Monitor.
   - The code within the `loop()` function is primarily responsible for continuously checking sensor values and adjusting the robot’s movement and steering accordingly.

### Summary

My Arduino code controls a robot by reading input from IR sensors and using this data to adjust the robot's movement and steering. The `PID()` function uses these readings to determine the direction the robot should steer to correct its path. The robot moves forward or backward based on the current state of its sensors, and the servo motor adjusts its steering angle to help navigate around obstacles. The debugging section provides a way to monitor the robot’s behavior and sensor readings in real-time.

--- 

This explanation covers the key aspects of My code, breaking down its functionality and how each part contributes to the robot’s operation.
