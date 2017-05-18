#include <Wire.h> 
#include <Servo.h>
Servo motor;
Servo servo;
float lastmillis = 0;
float rpm;
float revs = 0;
float curSpeed = 0;
int motorSpeed;
String inAngle = "";
int angle;

void setup() {
  Serial.begin(115200);
  motor.attach(9);
  servo.attach(10);
  pinMode(2, INPUT_PULLUP);
  attachInterrupt(0, rpm_fan, FALLING);
  lastmillis = millis()/1000.0;
  for (motorSpeed = 1500;motorSpeed <= 1680;motorSpeed  = motorSpeed + 10)
    {
      motor.writeMicroseconds(motorSpeed);
      delay(100);
    }
    delay(1000);
}

void loop() {
  if (millis() - lastmillis*1000 >= 200){ //Uptade every one second, this will be like reading frecuency
      detachInterrupt(0);
      rpm = 5*revs * 60 / 5;  
      curSpeed = (2*3.1415926536*0.03*rpm*60)/1000.0;
      revs = 0; 
      lastmillis = millis()/1000.0;
      attachInterrupt(0, rpm_fan, FALLING);
      if(curSpeed < 2){
      	motorSpeed = motorSpeed + 1;
      	motor.writeMicroseconds(motorSpeed);
      }
	  if(curSpeed > 2){
      	motorSpeed = motorSpeed - 1;
      	motor.writeMicroseconds(motorSpeed);
      }
  }   
}
void rpm_fan(){
  revs++;
}
void serialEvent()
{
   while(Serial.available()){
      char inChar = Serial.read(); 
      if(inChar != '.'){
      	inAngle = inAngle + inChar;
      }
      else{
        angle = -inAngle.toInt();
        if(angle < 45 && angle > -45){
          angle = map(angle,-90,90,0,180);
          servo.write(angle);
        }
        inAngle = "";
      }
   }
}