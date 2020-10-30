/*
   Credit to:
 
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com

   Improving the Beginner’s PID
   by Brett Beauregard, http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
*/

#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter

//// DC Motor variables /////////////////////////////////////////////////////////////////////////////////////////

// Pinouts based on Arduino Pro Micro
const byte hallApin = 6, hallBpin = 5;    // Hall effect sensors on motor encoder
const byte gate1 = 8, gate2 = 9;          // H-bridge gate controls for switching motor polarity (direction)
const byte motorPWM = 7;                  // Pulse width modulated signal for motor speed
const byte Vmax = 12;                     // Max voltage being sent to the motor

//// PID variables ////

unsigned long now, lastTime;
double input, output, setpoint = 0;
double error, lastErr, dErr, errSum;
double P, I, D; 
double kp = 1, ki = 0, kd = 2;     
double dT;
int sampleTime = 50; // units: ms
const byte maxOutput = Vmax;
const byte minOutput = 0;

//// IMU variables ////

Kalman kalmanY;
const int MPU = 0x68; // MPU6050 I2C address
double AccX, AccY, AccZ;
double GyroX, GyroY, GyroZ;
double accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ, kalAngleY;
double roll, pitch, yaw;
double AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
double elapsedTime, currentTime, previousTime;
int c = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  
  Serial.begin(9600);
  pinMode(hallApin, INPUT_PULLUP);
  pinMode(hallBpin, INPUT_PULLUP);
  pinMode(gate1, OUTPUT);
  pinMode(gate2, OUTPUT);

  // IMU setup
  Serial.begin(9600);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        // end the transmission

  delay(100);                        
  
  // Calculate IMU error
  calculate_IMU_error(); 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Note(s):
//  - output: output of the PID Controller
//  - setpoint: desired system output (roll angle of 0)
//  - input: actual system output, which is fed back into the controller

// PID control loop
void loop() {
  
  now = millis();
  dT = (double)(now - lastTime);
  
  // PID control occurs at a regular interval based on the sampleTime
  if (dT >= sampleTime){
   
    // Compute error
    input = getIMUangle();
    error = setpoint - input;
    dErr = (error - lastErr) / dT;
    
    // View IMU angle in the Serial Plotter
    Serial.println(input);
    
    // Compute PID Output
    P = kp * error;
    I += ki * (error * dT);
    D = kd * dErr; 

    // Integrator Anti-windup:
    // Clamp if the integrated term exeeds motor limits
    if (I > maxOutput) I = maxOutput;
    else if (I < minOutput) I = minOutput;

    output = P + I + D;
    
    // Update variables for next iteration
    lastErr = error;
    lastTime = now;

    // Convert PID output to PWM signal
    WriteDriverVoltage(output, Vmax);
  
  }
}

//// IMU code /////////////////////////////////////////////////////////////////////////////////////////////////////

// Calculates the accelerometer and gyro data error
void calculate_IMU_error() {

  // Read accelerometer values 200 times
  while (c < 200) {
    
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);                 // Starting address for AccX, AccY, AccZ
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);   // Request 6 bytes (48 bits) from the MPU6050

    // Note(s):
    //  - Each acceleration (AccX, AccY, AccZ) is stored as a 16-bit value
    //  - Wire.read() << 8 | Wire.read() takes the first byte and shifts it 8 bits (1 byte) to the left 
    //  - It then appends the second byte, giving us the full 16-bit value
      
    //  - For a default range of +/- 2g, the accelerometer sensitivity is 16384 LSB/g
    //  - We divide our raw values by 16384 to get the acceleration in g's    
     
    AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ; 
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    
    // Sum all readings
    // AccErrorX is the error in roll angle
    // AccErrorY is the error in pitch angle
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Repeat the same process for the gyro error 
  // Read gyro values 200 times
  while (c < 200) {
    
    Wire.beginTransmission(MPU);
    Wire.write(0x43);                       // Starting address for GyroX, GyroY, GyroZ
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);

    // The LSB sensitivity of the gyro is 131 LSB/deg/s at a range of +/- 250 deg/s
    // Thus, we divide the raw values to get angular velocity in deg/s
    GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
    GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
    
    // Sum all readings
    GyroErrorX = GyroErrorX + GyroX;
    GyroErrorY = GyroErrorY + GyroY;
    GyroErrorZ = GyroErrorZ + GyroZ;
    c++;
  }
  
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
}

// Returns the roll angle of the MPU 6050 sensor
double getIMUangle(){

  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 
  
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; 
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + AccErrorY;
  
  // === Read gyroscope data === //
  previousTime = currentTime;                           // Previous time is stored before the actual time read
  currentTime = millis();                               // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000;    // Divide by 1000 to get seconds
  
  Wire.beginTransmission(MPU);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); 

  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; 
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX;
  GyroY = GyroY - GyroErrorY; 
  GyroZ = GyroZ - GyroErrorZ;   
  
  // Currently the raw values are in degrees per seconds, deg/s
  // We need to multiply by seconds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime;
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  
  // Complementary filter - combine acceleromter and gyro angle values 
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;

  // Apply Kalman filter for optimal estimate of angular position
  kalAngleY = kalmanY.getAngle(gyroAngleY, GyroY, dT);
  
  return kalAngleY;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Converts the controller output to a PWM signal (% duty cycle)
void WriteDriverVoltage(double output, const byte Vmax) {

  int PWMval = int(255 * abs(output) / Vmax);
  
  if (PWMval > 255) {
    PWMval = 255;
  }

  // Set direction of DC motor //
  
  // for CW tilt:
  // input is pos (+)
  // error is neg (-)
  // spin motor CW
  if (output > 0) {
    digitalWrite(gate1, HIGH);
    digitalWrite(gate2, LOW);
  }

  // for CCW tilt:
  // input is neg (-)
  // error is pos (+)
  // spin motor CCW
  else if (output < 0) {
    digitalWrite(gate1, LOW);
    digitalWrite(gate2, HIGH);
  }
  
  else {
    digitalWrite(gate1, LOW);
    digitalWrite(gate2, LOW);
  }

  // Write PWM signal to motorPWM pin
  analogWrite(motorPWM, PWMval);
  
}
