/* Taken from reddit comment by creator of 2-axis balancing stick: 
 * The program runs at 100Hz. This is the basic tasks it performs each cycle for one axis (one motor):

Calculates speed of motor
Calculates pitch angle of the stick and the speed it is moving in that direction
Calculates desired torque using a PD controller (P and D are two constants you have to set). Desired_torque = P * pitch_angle + D * pitch_speed
Calculates what voltage to apply to achieve the desired torque (B is a constant found through experimentation of the specific motor). Applied_Voltage = Desired_Torque + B * Motor_Speed
Command the new voltage to the motors.
Repeat!
 */



// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PID_v1.h>
#include <Servo.h>

Servo motor;

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;

int motorus = 1500;

double angle = 0;

int16_t ax, ay, az;
int16_t gx, gy, gz;

double Setpoint, Input, Output;
double Kp = .1, Ki = 0, Kd = .05;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
  Serial.begin(38400);
  Serial.println("Initialize motor controller");
  motor.writeMicroseconds(1500);
  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // use the code below to change accel/gyro offset values

  Serial.println("Updating internal sensor offsets...");
  // -76	-2359	1688	0	0	0
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  accelgyro.setXGyroOffset(200);
  accelgyro.setYGyroOffset(135);
  accelgyro.setZGyroOffset(-81);
  Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
  Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print("\n");
  Setpoint = 0;
  //turn the PID on
  motor.attach(9);
  myPID.SetOutputLimits(-500, 500);
  myPID.SetMode(AUTOMATIC);

  delay(5000);
}

void loop() {
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  if(abs(gx) > 1){ //attempts to reduce drift
    angle += (gx/200.0);
  }
  Input = angle;
  myPID.Compute();
  motorus += Output*(1 + angle/100.0);
  motor.writeMicroseconds(motorus);
  /*Serial.print(angle); Serial.print(", "); */Serial.println(motorus);
  delay(10);
}
