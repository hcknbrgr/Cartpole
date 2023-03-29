/*

  Ultrasonic sensor pins:
    VCC: +5VDC
    Trig : Trigger (Input) - Pin11
    Echo: Echo (OUtput) - Pin12
    GND: GND
  

*/

int trigPin = 11;   // Trigger
int echoPin = 12;   // Echo
int LEDPin = 13;    // LED
int validDistance = 0; // Serial Input to determine valid

long duration, cm, inches;

void setup() {
  // put your setup code here, to run once:
  // Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LEDPin, OUTPUT);
  digitalWrite(LEDPin, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the signal from the sensor: a HGIH pulse whose duration is the time from the sending of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  // Convert the time into a distance
  cm = (duration/2) / 29.1;   
  inches = (duration/2) / 74;

  Serial.println(cm);
  if(Serial.available()>0){
    validDistance = Serial.read();
  }
  if(validDistance == 1)
  {
    digitalWrite(LEDPin, HIGH);
  }
  else
    digitalWrite(LEDPin, LOW);




  delay(250);
  
}
