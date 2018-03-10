#include <Wire.h>
#include <MPU6050.h>
#include <SoftwareSerial.h>

int VIBRATION_MOTOR = 3;

// the setup function runs once when you press reset or power the board
SoftwareSerial mySerial(7, 8); // RX, TX
// Connect HM10      Arduino Uno
//     Pin 1/TXD          Pin 7
//     Pin 2/RXD          Pin 8

MPU6050 mpu;
//Serial Read

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Timers
extern volatile unsigned long timer0_millis;
unsigned long new_value = 0;
unsigned long timer = 0;
float timeStep = 0.01;

// Pitch, Roll and Yaw values
float pitch = 0;
float roll = 0;
float yaw = 0;

//Initial Values
float pitch_init = 0;
float roll_init = 0;
float yaw_init = 0;

void setup()
{
  pinMode(VIBRATION_MOTOR, OUTPUT);
  Serial.begin(115200);
  mySerial.begin(115200);

  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // reserve 200 bytes for the inputString:
  inputString.reserve(200);

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  //mpu.setThreshold(3);
}

void loop()
{

  if (stringComplete||mySerial.read() == 'c')
  {
    // Calibrate gyroscope. The calibration must be at rest.
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    //mpu.setThreshold(3);
    stringComplete = false;
    setMillis(new_value);
    pitch = 0.0;
    roll = 0.0;
    yaw = 0.0;
  }
  timer = millis();

  // Read normalized values
  Vector norm = mpu.readNormalizeGyro();

  // Calculate Pitch, Roll and Yaw
  pitch = pitch + norm.YAxis * timeStep;
  roll = roll + norm.XAxis * timeStep;
  yaw = yaw + norm.ZAxis * timeStep;

  // Output raw
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  Serial.print(" Yaw = ");
  Serial.println(yaw);
//  mySerial.print(" Pitch = ");
//  mySerial.print(pitch);
//  mySerial.print(" Roll = ");
//  mySerial.print(roll);
//  mySerial.print(" Yaw = ");
//  mySerial.println(yaw);

if(pitch < -10.00 || pitch > 10.00)
{
//  digitalWrite(VIBRATION_MOTOR, HIGH);
//  delay(500);
//  digitalWrite(VIBRATION_MOTOR, LOW);
  Serial.println("Incorrect Posture");
  mySerial.println('i');
  
}

  // Wait to full timeStep period
  delay((timeStep * 1000) - (millis() - timer));

}

void setMillis(unsigned long new_millis)
{
  uint8_t oldSREG = SREG;
  cli();
  timer0_millis = new_millis;
  SREG = oldSREG;
}

void serialEvent()
{
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is 'c', set a flag so the main loop can
    // do something about it:
    if (inChar == 'c') {
      stringComplete = true;
    }
  }
}
