#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <utility/imumaths.h>
#include <time.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>

// Pin definitions:
// SD definitions:
#define SD_CS 17
#define SD_SI 19
#define SD_SO 16
#define SD_CLK 18

// BMP388/BNO definitions:
#define SENSOR_SDA 4
#define SENSOR_SCL 5
#define SEALEVELPRESSURE_HPA (1013.25)


// Set up servos
Servo axis1;
Servo axis2;
Servo axis3;
Servo claw;

// Set up motor
#define MOTOR_AIN1 8
#define MOTOR_AIN2 7
#define MOTOR_PWMA 6

// Servo pins
#define AXIS_1_PWM 15
#define AXIS_2_PWM 14
#define AXIS_3_PWM 13
#define CLAW_PWM 12


Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Logging
File dataFile;
#define LOG_INTERVAL_MS 100
#define FLUSH_INTERVAL_MS 1000
unsigned long lastLogTime = 0;
unsigned long lastFlushTime = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(3000); //Initialize sensor delay

  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);


   if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  pinMode(MOTOR_AIN1, OUTPUT);
  pinMode(MOTOR_AIN2, OUTPUT);
  pinMode(MOTOR_PWMA, OUTPUT);

  digitalWrite(MOTOR_AIN1, LOW);
  digitalWrite(MOTOR_AIN2, LOW);
  analogWrite(MOTOR_PWMA,0);

  axis1.attach(AXIS_1_PWM);
  axis2.attach(AXIS_2_PWM);
  axis3.attach(AXIS_3_PWM);
  claw.attach(CLAW_PWM);

  SPI.setRX(SD_SO);
  SPI.setTX(SD_SI);
  SPI.setSCK(SD_CLK);
  SPI.begin();


  if (!SD.begin(SD_CS)) {
    Serial.println("SD card init failed, logging disabled");
  } else {
    bool fileExists = SD.exists("data.csv");
    
    // Open file persistently in write mode
    dataFile = SD.open("data.csv", FILE_WRITE);
    
    if (dataFile && !fileExists) {
      dataFile.println("Time_ms,Heading_deg,Pitch_deg,Roll_deg,Temp_C,Pressure_hPa,Altitude_m");
      dataFile.flush();
    }
  }


}

void loop() {
  // put your main code here, to run repeatedly:

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    

  }

   unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL_MS) {
    lastLogTime = currentMillis;

    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

    bool bmpSuccess = bmp.performReading();
    float temp = bmpSuccess ? bmp.temperature : 0.0;
    float pressure = bmpSuccess ? (bmp.pressure / 100.0) : 0.0;
    float altitude = bmpSuccess ? bmp.readAltitude(SEALEVELPRESSURE_HPA) : 0.0;

    // Fast append without re-opening the file system cluster maps
    if (dataFile) {
      dataFile.print(currentMillis); dataFile.print(",");
      dataFile.print(orientationData.orientation.x); dataFile.print(",");
      dataFile.print(orientationData.orientation.y); dataFile.print(",");
      dataFile.print(orientationData.orientation.z); dataFile.print(",");
      dataFile.print(temp); dataFile.print(",");
      dataFile.print(pressure); dataFile.print(",");
      dataFile.println(altitude);
    }
  }
  if (currentMillis - lastFlushTime >= FLUSH_INTERVAL_MS) {
    lastFlushTime = currentMillis;
    if (dataFile) {
      dataFile.flush(); // Forces writing data to physical SD card blocks seamlessly
    }
  }
}
