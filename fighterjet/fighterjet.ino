#include <MPU6050.h>

#include <Wire.h>
#include <Servo.h>

MPU6050 mpu;
Servo xservo;
Servo yservo;
// Timers
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

void setup() {


  // put your setup code here, to run once:
  xservo.attach(A1);
  yservo.attach(A3);
  Serial.begin(9600);
  xservo.write(90);
  yservo.write(90);
  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
//    // Calibrate gyroscope. The calibration must be at rest.
//  // If you don't want calibrate, comment this line.
//  mpu.calibrateGyro();
//
//  // Set threshold sensivty. Default 3.
//  // If you don't want use threshold, comment this line or set 0.
//  mpu.setThreshold(3);
}

void loop() {
//  // put your main code here, to run repeatedly:
   Vector normAccel = mpu.readNormalizeAccel();
 

   int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

 


  // Output raw
  Serial.print(" normAccel.XAxis = ");
  Serial.print(normAccel.XAxis);

//  Serial.print(" Yaw = ");
//  Serial.println(yaw1);
  int mappedVal = map(normAccel.XAxis,-10,4,0,100);
//  
  // Wait to full timeStep period
  // Output raw
  Serial.print(" mappedVal = ");
  Serial.print(mappedVal);

//  
//   xservo.write(mappedVal);
   Serial.print(" normAccel.ZAxis = ");
  Serial.print(normAccel.ZAxis);
//xservo.write(0);
////  delay(2000);
////
Serial.println();
  delay(200);
}
