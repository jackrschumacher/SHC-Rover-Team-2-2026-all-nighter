#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "Adafruit_BMP3XX.h"
#include <utility/imumaths.h>
#include <time.h>
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>

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

// Servo easing: each servo steps 1 degree toward its target every stepMs
// (10 ms = ~100 deg/s). Raise stepMs to slow a servo down.
#define SERVO_STEP_MS 10
#define CLAW_STEP_MS 15
#define AXIS_START_DEG 90
#define CLAW_START_DEG 30
#define NUM_SERVOS 4
#define SERVO_MAX_DEG 180
#define CLAW_MAX_DEG 60

struct EasedServo {
  Servo *servo;
  int current;
  int target;
  unsigned long stepMs;
  unsigned long lastStepTime;
};

EasedServo easedServos[NUM_SERVOS] = {
  { &axis1, AXIS_START_DEG, AXIS_START_DEG, SERVO_STEP_MS, 0 },
  { &axis2, AXIS_START_DEG, AXIS_START_DEG, SERVO_STEP_MS, 0 },
  { &axis3, AXIS_START_DEG, AXIS_START_DEG, SERVO_STEP_MS, 0 },
  { &claw, CLAW_START_DEG, CLAW_START_DEG, CLAW_STEP_MS, 0 },
};

// Last servo pose is saved to flash so boot resumes where the arm was left
// instead of jumping. Saved only after the arm has been still for
// SAVE_DELAY_MS, to limit flash wear.
#define POSE_MAGIC 0xA55A
#define SAVE_DELAY_MS 2000

struct SavedPose {
  uint16_t magic;
  uint8_t deg[NUM_SERVOS];
};

uint8_t savedDeg[NUM_SERVOS];
unsigned long lastServoMoveTime = 0;

// Call before attaching servos
void loadSavedPose() {
  EEPROM.begin(sizeof(SavedPose));
  SavedPose pose;
  EEPROM.get(0, pose);
  if (pose.magic != POSE_MAGIC) return;  // Nothing saved yet, keep start defaults

  for (int i = 0; i < NUM_SERVOS; i++) {
    int maxDeg = (i == NUM_SERVOS - 1) ? CLAW_MAX_DEG : SERVO_MAX_DEG;
    if (pose.deg[i] > maxDeg) return;  // Corrupt data, keep start defaults
  }
  for (int i = 0; i < NUM_SERVOS; i++) {
    easedServos[i].current = easedServos[i].target = pose.deg[i];
    savedDeg[i] = pose.deg[i];
  }
}

void savePoseIfSettled() {
  bool changed = false;
  for (int i = 0; i < NUM_SERVOS; i++) {
    if (easedServos[i].current != easedServos[i].target) return;  // Still moving
    if (easedServos[i].current != savedDeg[i]) changed = true;
  }
  if (!changed || millis() - lastServoMoveTime < SAVE_DELAY_MS) return;

  SavedPose pose;
  pose.magic = POSE_MAGIC;
  for (int i = 0; i < NUM_SERVOS; i++) {
    pose.deg[i] = savedDeg[i] = easedServos[i].current;
  }
  EEPROM.put(0, pose);
  EEPROM.commit();
}

// Call every loop; moves each servo at most 1 degree per its step interval
void updateServos() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_SERVOS; i++) {
    EasedServo &s = easedServos[i];
    if (s.current == s.target || now - s.lastStepTime < s.stepMs) continue;
    s.lastStepTime = now;
    lastServoMoveTime = now;
    s.current += (s.target > s.current) ? 1 : -1;
    s.servo->write(s.current);
  }
  savePoseIfSettled();
}


Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// Logging
File dataFile;
#define LOG_INTERVAL_MS 100
#define FLUSH_INTERVAL_MS 1000
unsigned long lastLogTime = 0;
unsigned long lastFlushTime = 0;
#define SERIAL_PRINT_INTERVAL_MS 1000
unsigned long lastSerialPrintTime = 0;
#define CSV_HEADER "Time_ms,Heading_deg,Pitch_deg,Roll_deg,Temp_C,Pressure_hPa,Altitude_m"

// Writes one CSV row to any output (SD file or Serial)
void printCsvRow(Print &out, unsigned long timeMs, sensors_event_t &orientation,
                 float temp, float pressure, float altitude) {
  out.print(timeMs);
  out.print(",");
  out.print(orientation.orientation.x);
  out.print(",");
  out.print(orientation.orientation.y);
  out.print(",");
  out.print(orientation.orientation.z);
  out.print(",");
  out.print(temp);
  out.print(",");
  out.print(pressure);
  out.print(",");
  out.println(altitude);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  // Attach servos first so they get a holding signal right away instead of
  // floating through the sensor startup delay. Writing before attach makes the
  // very first pulse the start angle (otherwise the library sends 90 deg first).
  loadSavedPose();
  for (int i = 0; i < NUM_SERVOS; i++) {
    easedServos[i].servo->write(easedServos[i].current);
  }
  axis1.attach(AXIS_1_PWM);
  axis2.attach(AXIS_2_PWM);
  axis3.attach(AXIS_3_PWM);
  claw.attach(CLAW_PWM);

  delay(3000);  //Initialize sensor delay

  if (!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("No BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
      ;
  }
  bno.setExtCrystalUse(true);


  if (!bmp.begin_I2C()) {  // hardware I2C mode, can pass in address & alt Wire
                           //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
                           //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1)
      ;
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
  analogWrite(MOTOR_PWMA, 0);

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
      dataFile.println(CSV_HEADER);
      dataFile.flush();
    }
  }

  Serial.println(CSV_HEADER);
}

// Command parsing:
// Expects "motor,axis1,axis2,axis3,claw\n", each value 0-99
#define NUM_VALUES 5
#define CMD_BUF_LEN 32
#define MOTOR_DEADBAND 10  // PWM below this is treated as stop

char cmdBuf[CMD_BUF_LEN];
int cmdLen = 0;
bool cmdOverflow = false;

// 0 = full reverse, ~49/50 = stop, 99 = full forward
void driveMotor(int value) {
  int speed = map(value, 0, 99, -255, 255);
  if (abs(speed) < MOTOR_DEADBAND) {
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, LOW);
    analogWrite(MOTOR_PWMA, 0);
  } else if (speed > 0) {
    digitalWrite(MOTOR_AIN1, HIGH);
    digitalWrite(MOTOR_AIN2, LOW);
    analogWrite(MOTOR_PWMA, speed);
  } else {
    digitalWrite(MOTOR_AIN1, LOW);
    digitalWrite(MOTOR_AIN2, HIGH);
    analogWrite(MOTOR_PWMA, -speed);
  }
}

// Returns false (and moves nothing) if the line is malformed
bool parseCommand(char *line) {
  int values[NUM_VALUES];
  int count = 0;

  char *token = strtok(line, ",");
  while (token != NULL) {
    if (count >= NUM_VALUES) return false;

    char *end;
    long v = strtol(token, &end, 10);
    while (*end == ' ' || *end == '\r') end++;
    if (end == token || *end != '\0' || v < 0 || v > 99) return false;

    values[count++] = (int)v;
    token = strtok(NULL, ",");
  }
  if (count != NUM_VALUES) return false;

  driveMotor(values[0]);
  // Servos ease toward these in updateServos()
  easedServos[0].target = map(values[1], 0, 99, 0, SERVO_MAX_DEG);
  easedServos[1].target = map(values[2], 0, 99, 0, SERVO_MAX_DEG);
  easedServos[2].target = map(values[3], 0, 99, 0, SERVO_MAX_DEG);
  easedServos[3].target = map(values[4], 0, 99, 0, CLAW_MAX_DEG);
  return true;
}

// Non-blocking so sensor logging keeps running between commands
void readSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      cmdBuf[cmdLen] = '\0';
      if (cmdOverflow || (cmdLen > 0 && !parseCommand(cmdBuf))) {
        Serial.println("Bad command, expected 5 values 0-99: m,a1,a2,a3,claw");
      }
      cmdLen = 0;
      cmdOverflow = false;
    } else if (cmdLen < CMD_BUF_LEN - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = true;  // Drop the rest of this line
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  readSerialCommands();
  updateServos();

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
      printCsvRow(dataFile, currentMillis, orientationData, temp, pressure, altitude);
    }

    if (currentMillis - lastSerialPrintTime >= SERIAL_PRINT_INTERVAL_MS) {
      lastSerialPrintTime = currentMillis;
      printCsvRow(Serial, currentMillis, orientationData, temp, pressure, altitude);
    }
  }
  if (currentMillis - lastFlushTime >= FLUSH_INTERVAL_MS) {
    lastFlushTime = currentMillis;
    if (dataFile) {
      dataFile.flush();  // Forces writing data to physical SD card blocks seamlessly
    }
  }
}
