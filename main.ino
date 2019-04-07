#include "PID_v1.h"
#include "max6675.h"
#include "Servo.h"

// ----------
Servo ServoMotor;
const uint8_t servoPin = 6;

// ----------
int thermoDO = 12;
int thermoCS = 10;
int thermoCLK = 13;

MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
int vccPin = 3;
int gndPin = 2;

// ----------
double Setpoint, Input, Output;

double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// ----------
void setup()
{
  Serial.begin(9600);

  Input = 0;
  Setpoint = 120;

  myPID.SetMode(AUTOMATIC);

  pinMode(vccPin, OUTPUT); digitalWrite(vccPin, HIGH);
  pinMode(gndPin, OUTPUT); digitalWrite(gndPin, LOW);

  ServoMotor.attach(servoPin);
  delay(100);
  ServoMotor.write(0);
  delay(400);
}

int outputCount = 0;
void loop()
{
  delay(250);
  double in = thermocouple.readCelsius();
  if (in == NAN)
    return;

  Input = in;
  myPID.Compute();

  int setAngle = mapRange(Output);
  ServoMotor.write(setAngle);

  outputCount++;
  if(outputCount >= (4*5)) {
    outputCount=0;
    Serial.print(Input);
    Serial.print("\t");
    Serial.println(setAngle);
  }
}

int mapRange(double setting)
{
  return (int)map(setting, 0, 255, 0, 89);
}

