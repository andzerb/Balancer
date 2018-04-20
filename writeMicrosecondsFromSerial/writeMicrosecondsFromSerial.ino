#include <Servo.h>

Servo motor;
int val;
void setup()
{
  Serial.begin(38400);
  motor.attach(9);
  motor.writeMicroseconds(1500);
}
void loop()
{
  //waiting for input
  while (Serial.available() > 0){
    val = Serial.parseInt(); //read int or parseFloat for ..float...
    Serial.println(val);
  }
  motor.writeMicroseconds(val);
}
