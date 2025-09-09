#include <Servo.h>
#include <I2Cdev.h>
#include <Buzzer.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <BMP085.h>
#include <MPU6050_light.h>

Buzzer buzzer(3, 13);
int CS_pin = 4;
int state = 0;
int aState = 0;
int bstate = 0;
int cstate = 0;

MPU6050 mpu(Wire);
Servo servoX;
Servo servoY;
int16_t AX, AY, AZ;
int16_t AngleX, AngleY, AngleZ;
int16_t GX, GY, GZ;

double PIDX, PIDY, errorX, errorY, previous_errorX, previous_errorY, pwmX, pwmY, gyroXX, gyroYY;
 
int desired_angleX = 90;//servoY
int desired_angleY = 0;//servoX

int servoX_offset = 90;
int servoY_offset = 90;

int servoXstart = 0;
int servoYstart = 0;

float servo_gear_ratio = 2;

BMP085 barometer;
float temperature;
float pressure;
int32_t altitude;
int time = 500;
int lastAltitude;
 
int groundAltitude; // Stores starting altitude, before launch
int maxHeight;

int apogeeTemp;

int apogee;
int current_altitude;
int previous_altitude;
int apogee_temperature;

double dt, currentTime, previousTime;

float pidX_p = 0;
float pidX_i = 0;
float pidX_d = 0;
float pidY_p = 0;
float pidY_i = 0;
float pidY_d = 0;

float gyroAngleX;
float gyroAngleY;
int32_t accAngleX;
int32_t accAngleY;
float pitch; 
float yaw;

int pos;
int buzr = 3;
long timer = 0;

double kp = 0.9;
double ki = 0.2;
double kd = 0.0255;
int RLED = 9;
int GLED = 8;
int BLED = 7;
//SimpleKalmanFilter kalman(0.001, 0.003, 0.03);
int Mosfet = 2;
File myFile;

void setup() {
  servoX.attach(5);
  servoY.attach(6);
  Serial.begin(9600);
  pinMode(buzr, OUTPUT); 
  pinMode(RLED, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(BLED, OUTPUT); 
  pinMode(CS_pin, OUTPUT);
  pinMode(Mosfet, OUTPUT);
   
   buzzer.begin(0);
   buzzer.sound(NOTE_A5, time/2);
   buzzer.sound(NOTE_D3, time/2);
   buzzer.sound(NOTE_A5, time/2); 
   buzzer.end(0);   
  if (!SD.begin()) {
    Serial.println(F("initialization failed!"));
    //return;
  }
  Serial.println(F("initialization done."));
    
    myFile = SD.open("data.txt", FILE_WRITE);
    myFile.close();
    delay(50);
  Wire.begin();
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  Serial.println(F("Calculating offsets, do not move MPU6050"));
 
  //mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done !"); 
   // initialize device
  Serial.println("Initializing I2C devices...");
  barometer.initialize(); 
  Serial.println("Testing device connections...");
  Serial.println(barometer.testConnection() ? "BMP085 connection successful" : "BMP085 connection failed");
  digitalWrite(RLED, HIGH);
  delay(500);
  digitalWrite(GLED, HIGH);
  digitalWrite(RLED, LOW);
  delay(500); 
  servoX.write(90);
  servoY.write(90); 
  myFile = SD.open("data.txt", FILE_WRITE);
  delay(1000);
  myFile.print("ground_temperature : ");myFile.println(barometer.getTemperatureC());
  myFile.print("ground_altitude : ");myFile.println(barometer.getAltitude(pressure));
  myFile.print("AX");myFile.print(":");myFile.print("AY");myFile.print(":");myFile.print("AZ");myFile.print(":");myFile.print("GX");myFile.print(":");myFile.print("GY");myFile.print(":");myFile.print("GZ");myFile.print(":");myFile.print("AngleX");myFile.print(":");myFile.print("AngleZ");myFile.print(":");myFile.print("Temperature");myFile.print(":");myFile.print("Pressure");myFile.print(":");myFile.println("Altitude");
  myFile.close();
  delay(1000);
 
}
 

void loop() {
    previousTime = currentTime;        
    currentTime = millis();            
    dt = (currentTime - previousTime) / 1000; 
    previousTime = currentTime; 
    
    mpu.update();
     barometer.setControl(BMP085_MODE_TEMPERATURE);
    // read calibrated temperature value in degrees Celsius
    temperature = barometer.getTemperatureC();

    // request pressure (3x oversampling mode, high detail, 23.5ms delay)
    barometer.setControl(BMP085_MODE_PRESSURE_3);

    // read calibrated pressure value in Pascals (Pa)
    pressure = barometer.getPressure();
    altitude = barometer.getAltitude(pressure);
    delay(10);
//    Serial.println(AngleX);Serial.print("\t");Serial.print(AngleZ);Serial.print("\t");
    //Serial.println(AY);
    launchdetect();
    
    
}
void Apogee() {
  if (millis() - timer > 1000) {
      Serial.println(millis() - timer);
//    Serial.println(current_altitude - previous_altitude);
    current_altitude = altitude;
   if (current_altitude - previous_altitude < -1){
        myFile = SD.open("data.txt", FILE_WRITE);
        myFile.println("Deploy Parachutes");myFile.print("Apogee_Temperature : ");
        myFile.println(temperature);
        myFile.print("Apogee Altitude : ");myFile.println(altitude - 1);
        myFile.close();
        buzzer.sound(NOTE_A5, time);
        digitalWrite(Mosfet, HIGH);
        delay(1000);
        digitalWrite(Mosfet, LOW);
          bstate = 1;
          cstate = 1;
          
  }
    timer = millis();
    previous_altitude = current_altitude;
 } 
   after_apogee();
}
void accel_degrees() {
  
    AX = mpu.getAccX();
    AY = mpu.getAccY();
    AZ = mpu.getAccZ();
    AngleX = mpu.getAngleX();
    AngleY = mpu.getAngleY();
    AngleZ = mpu.getAngleZ();
    GX = mpu.getGyroX();
    GY = mpu.getGyroY();
    GZ = mpu.getGyroZ();
    accAngleX = mpu.getAccAngleX();
    accAngleY = mpu.getAccAngleY();
//    Serial.print(AY);
    if (cstate == 0) {
      pid_compute();
    }

}
void pid_compute()  {
   
  errorX = AngleX - desired_angleX;
  errorY = AngleZ - desired_angleY;

  //Defining "P" 
  pidX_p = kp*errorX;
  pidY_p = kp*errorY;
  
  //Defining "D"
  pidX_d = kd*((errorX - previous_errorX)/dt);
  pidY_d = kd*((errorY - previous_errorY)/dt);
  
  //Defining "I"
  pidX_i = ki * (pidX_i + errorX * dt);
  pidY_i = ki * (pidY_i + errorY * dt);

  previous_errorX = errorX;
  previous_errorY = errorY;
  
  PIDX = pidX_p+ pidX_i + pidX_d;
  PIDY = pidY_p+ pidY_i + pidY_d;
  
  pwmX = ((PIDX * servo_gear_ratio) + servoX_offset);
  pwmY = ((PIDY * servo_gear_ratio) + servoY_offset);
  servowrite();
}
void servowrite() {
  
  servoX.write(pwmX);
  servoY.write(pwmY); 
  launch_abort();
  
}
void launchdetect () {
   if ( mpu.getAccY() > 1.5) {
   state = 1;

 }
 if (state == 1) {
    digitalWrite(RLED, HIGH);
    delay(10);
    digitalWrite(GLED, HIGH);
    delay(10);
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, LOW);
  data();
  accel_degrees();
  Apogee();
 }
}
void data() {
  myFile = SD.open("data.txt", FILE_WRITE);
  myFile.print(AX);myFile.print(":");myFile.print(AY);myFile.print(":");myFile.print(AZ);myFile.print(":");myFile.print(GX);myFile.print(":");myFile.print(GY);myFile.print(":");myFile.print(GZ);myFile.print(":");myFile.print(AngleX);myFile.print(":");myFile.print(AngleZ);myFile.print(":");myFile.print(temperature);myFile.print(":");myFile.print(pressure);myFile.print(":");myFile.println(altitude);
  myFile.close();
}
void launch_abort() {
  if ((AngleZ < -45 || AngleZ > 45) || (AngleX < 45 || AngleX > 135)){
    aState = 1;
  }
  if (aState == 1) {
    myFile = SD.open("data.txt", FILE_WRITE);
    myFile.println("Launch abort");
    myFile.close();
    digitalWrite(RLED, HIGH);
    delay(1000);
    digitalWrite(RLED, LOW);
    exit(0);
  }
}

void after_apogee() {
  if (bstate == 1){
    if ( GX == 0 & GY == 0 & GZ == 0 & AX == 0 & AY == 0 & AZ == 0){
      buzzer.sound(NOTE_A4, time);
      digitalWrite(RLED, HIGH);
      delay(500);
      digitalWrite(RLED, LOW);
      myFile = SD.open("data.txt", FILE_WRITE);
      myFile.println("Rocket Landed");
      myFile.close();
      delay(1000);
//      Serial.println(dt);
      exit(0);
    }
  }
}
