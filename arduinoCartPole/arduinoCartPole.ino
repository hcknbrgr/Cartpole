  //TODO: CENTER THE CART IN THE ENVIRONMENT TO ENSURE MOTOR CONTROL WORKS

#include <MPU6050.h>
#include <Wire.h>

/*
  Ultrasonic sensor pins:
    VCC: +5VDC
    Trig : Trigger (Input) - Pin11
    Echo: Echo (OUtput) - Pin12
    GND: GND
  
  MPU 6050 gyroscope pins:
    VCC: +5VDC
    GND: GND
    SCL: SCL
    SDA: SDA

  Motor pinout
    Motor1
      OUT1: Pin1
      OUT2: Pin2
      ENA: PWMPin3
    Motor2
      OUT3: Pin7
      OUT4: Pin8
      ENB: PWMPin6
*/

// MOTOR PINS
int motor1pin1 = 1;
int motor1pin2 = 2;
int motor1PWM = 3;
int motor2pin1 = 7;
int motor2pin2 = 8;
int motor2PWM = 6;
int boardLED = 13;

// ULTRASONIC SENSOR PINS
int trigPin = 11;   // Trigger
int echoPin = 12;   // Echo

MPU6050 mpu;

int speed = 0;  // 0 - 255
int motorDirection = 0; // Serial Input to determine which direction to send the motor in. 
  // 0 = move towards sensor
  // 1 = stationary
  // 2 = move away from sensor

long duration;

void setup() {
  // Serial Port begin
  Serial.begin (115200);
  digitalWrite(boardLED, LOW);  
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println(0);
    delay(500);
  }
  pinMode(boardLED, OUTPUT);
  digitalWrite(boardLED, HIGH);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor1PWM, OUTPUT); 
}

void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);
  Serial.println(duration);  
  //cm = (duration/2) / 29.1;   lets do this math on the laptop
  //send over serial and let python do the math

  delay(25); // delay to let serial communication happen
  
  if(Serial.available()>0) {
    motorDirection = Serial.read(); 
  }
  // 0 = move towards sensor
  // 1 = stationary
  // 2 = move away from sensor

  // if we want to use the PWM to modify the speed, we'll use: analogWrite(motor1PWM, speed);
  // we need to assign speed
  if(motorDirection == 0)  // 0 = move towards sensor
  {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  } 
  else if(motorDirection == 2)
  {
    digitalWrite(motor1pin1, HIGH);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, HIGH);
  }
  else if(motorDirection == 3)  // 3 = this is set ready to go!
  {
    digitalWrite(boardLED, LOW);  // turn off the light that was turned on when the gyroscope was online
  }
  else  // 1 = stationary
  {
    digitalWrite(motor1pin1, LOW);
    digitalWrite(motor1pin2, LOW);
    digitalWrite(motor2pin1, LOW);
    digitalWrite(motor2pin2, LOW);
  }

/*
  // *********** Code for pitch and roll
  // Read normalized values
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & ROll by Korneliusz Jarzebski
  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  Serial.println(duration);

  //int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  
  Serial.println(pitch);
*/  

  delay(100);
  
}
