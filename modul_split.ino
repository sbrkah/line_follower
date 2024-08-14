// Deklarasi penggunaan librari yang dipakai
#include "Wire.h"
#include "l298n_functions.h"
#include "menu_functions.h"
#include "LiquidCrystal_I2C.h"
#include "QMC5883LCompass.h"
#include "SparkFunLSM6DSO.h"
#include "HCSR04.h"
#include "Servo.h"

// Setup Pin
// Pin Motor
#define motor1Pin1 A1
#define motor1Pin2 A0
#define motor1PWM 5
#define motor2Pin1 A2
#define motor2Pin2 A3
#define motor2PWM 6
// Pin Servo
#define servo1Pin 9
#define servo2Pin 10
// Pin Ultrasonic
#define ultrasonicTrigger 3
#define ultrasonicEcho 2
// Pin Pushbutton
#define prevButtonPin 11
#define nextButtonPin 12
#define stopButtonPin 8

// Data Pengaturan
float gyroZOffset = -0.08; // Masukkan nilai hasil kalibrasi IMU sebelumnya
const int calibrationSamples = 100;
const float alpha = 1;
int batasBedaJarak = 40;
float batasBedaSudut = 2.0;
unsigned long intervalBacaSensor = 5;
unsigned long intervalBacaPushButton = 100;
const float startSpeed = 180;
const float maxSpeed = 180;
const float minSpeed = 120;
const float Kp = 2.0;

// Setup Variables
const char* MENU[] = {"TITIK A", "TITIK B", "TITIK C", "TITIK D"};
const int TITIK[] = {50, 90, -50, -90};
float YAW, lsmGx = 0, lsmGy, lsmGz, lsmYaw;
float batasBawahGyro = gyroZOffset * 1.5;
int avgCount = 0;
int avgSet = 10;
float gyMx, gyMy, gyYaw = 0.0;
float totalGyYaw, avgGyYaw = 0.0;
float jarakUltrasonic = 999;
int sudutPutarKap = 90;
int prevButtonState, prevLastButtonState, nextButtonState;
int rodaState, rodaPwm = maxSpeed, rodaSTOP = 999;
int jalanMaju = 1, jalanMundur = 2, putarKanan = 3, putarKiri = 4;
String robotState = "BERHENTI", robotOperasi = "", robotSTOP = "BERHENTI", TARGET = "", sAMBIL = "AMBIL OBJEK";
String SAMPAI = "PENURUNAN OBJ", MULAI = "MENUJU TITIK", PULANG = "MENUJU BASE", ARAHTITIK = "MENCARI TITIK", ARAHBALIK = "MENCARI BASE";
float targetArah = 0.0;
unsigned long lastTimePos, lastTimeGyro, interval, lastInterval, lastBacaSensor;
unsigned long lastBacaPushButton, nextLastButtonState, stopButtonState, stopLastButtonState;

// Inisiasi Objek Library
LSM6DSO myIMU;
l298n motor(motor1PWM, motor1Pin1, motor1Pin2, motor2Pin1, motor2Pin2, motor2PWM);
LiquidCrystal_I2C lcd(0x27,  16, 2);
QMC5883LCompass compass;
UltraSonicDistanceSensor HC04(ultrasonicTrigger, ultrasonicEcho);
Servo servo1;
Servo servo2;

void printLCD(String text = "", int x = -1, int y = -1) {
  if (x > -1 && y > -1) {
    lcd.setCursor(x, y);
  }
  lcd.print(text);
}

void statePrinter(int state) {
  String _state = String(state) + " pwm:" + String(rodaPwm);
  //  Serial.println(_state);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  motor.init();
  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);
  servo1.write(2);
  servo2.write(190);
  pinMode(prevButtonPin, INPUT_PULLUP);
  pinMode(nextButtonPin, INPUT_PULLUP);
  pinMode(stopButtonPin, INPUT_PULLUP);
  printLCD("Memulai Kompas", 0, 1);
  compass.init();
  compass.setSmoothing(20, true);
  delay(500);
  compass.setCalibrationOffsets(-22.00, -97.00, 1222.00); // Masukkan nilai kalibrasi dari hasil kalibrasi sebelumnya
  compass.setCalibrationScales(0.88, 4.94, 0.60);         // Masukkan nilai kalibrasi dari hasil kalibrasi sebelumnya
  lcd.clear();
  printLCD("Memulai IMU", 0, 1);
  if ( myIMU.begin() ) {
    myIMU.initialize(BASIC_SETTINGS);
    delay(500);
    lcd.clear();
    printLCD("OK", 0, 1);
    delay(500);
  } else {
    printLCD("IMU ERROR!   ", 0, 1);
    while (1);
  }
  setupMenu(MENU, TITIK, 4);
}

// Fungsi print lcd dengan tambahan spasi yang dapat ditentukan
String fixPrint(String huruf, int len, bool sesudah = false) {
  int kurang = len - huruf.length();
  String spasi = "";
  for (int i = 0; i < kurang; i++) {
    spasi += " ";
  }
  if (sesudah) {
    return huruf + spasi;
  }
  else {
    return spasi + huruf;
  }
}

// Buka gripper dengan 2 servo
void gripperBuka() {
  for (int pos = 0; pos <= sudutPutarKap; pos++) {
    // servo 1 kondisi awal pada 0
    servo1.write(pos);
    delay(10);
    // servo 2 kondisi mirror, sehingga nilainya mulai dari 180
    servo2.write(180 - pos);
    delay(10);
  }
}

// Tutup gripper dengan 2 servo
void gripperTutup() {
  for (int pos = sudutPutarKap; pos >= 0; pos--) {
    // servo 1 kondisi awal pada 0
    servo1.write(pos);
    delay(10);
    // servo 2 kondisi mirror, sehingga nilainya mulai dari 180
    servo2.write(180 - pos);
    delay(10);
  }
}

// Fungsi untuk membaca nilai gyro dari sensor IMU LSM6DS
void bacaGyro() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTimeGyro) / 1000.0;
  lastTimeGyro = currentTime;
  lsmGz = myIMU.readFloatGyroZ() * -1 - gyroZOffset;
  float calcGz = lsmGz * dt;
  if (abs(calcGz) > abs(batasBawahGyro)) {
    lsmYaw += calcGz;
    if (lsmYaw >= 360.0) {
      lsmYaw -= 360.0;
    } else if (lsmYaw < 0.0) {
      lsmYaw += 360.0;
    }
  }
}

// Fungsi untuk membaca nilai kompas dari sensor GY-271
void bacaKompas() {
  compass.read();
  gyMx = compass.getX();
  gyMy = compass.getY();
  gyYaw = atan2(gyMy, gyMx) * 180.0 / PI;

  if (gyYaw >= 360.0) {
    gyYaw -= 360.0;
  } else if (gyYaw < 0.0) {
    gyYaw += 360.0;
  }
  if (avgCount < avgSet) {
    avgCount += 1;
    totalGyYaw += gyYaw;
    avgGyYaw = totalGyYaw / avgCount;
  } else {
    avgCount = 0;
    totalGyYaw = 0.0;
  }
}

// Fungsi untuk membaca nilai ultrasonik dari sensor HCSR04
void bacaUltrasonic() {
  jarakUltrasonic = HC04.measureDistanceCm();
  delay(50);
  if (jarakUltrasonic < 0) {
    jarakUltrasonic = 999;
  }
}

void cariArah(float tujuan) {
  rodaPwm = startSpeed;
  if (tujuan > 0) {
    rodaState = putarKanan;
  } else {
    rodaState = putarKiri;
  }

  targetArah = YAW + tujuan;

  if (targetArah < 0) {
    targetArah += 360;
  }
  if (targetArah > 360) {
    targetArah -= 360;
  }
}

void startRobot() {
  robotState = ARAHTITIK;
}

void stopRobot() {
  motor.STOP();
  robotState = robotSTOP;
  rodaState = rodaSTOP;
}

void putarRobot(float targetYaw, int batasEkstra = 0) {
  rodaPwm = startSpeed;
  gerakMotor();
  delay(50);
  float error = abs(targetYaw - YAW);
  float minError = error;
  while (abs(error) > batasBedaSudut + batasEkstra) {
    bacaSensor();
    float controlSignal = Kp * minError;
    if (controlSignal > maxSpeed) {
      controlSignal = maxSpeed;
    } else if (controlSignal < -maxSpeed) {
      controlSignal = -maxSpeed;
    }
    if (controlSignal > 0) {
      statePrinter(19);
      rodaPwm = controlSignal;
      statePrinter(20);
    }
    if (abs(rodaPwm) < minSpeed) {
      statePrinter(21);
      rodaPwm = (rodaPwm > 0) ? minSpeed : -minSpeed;
      statePrinter(22);
    }
    gerakMotor();
    pilihMenu();
    delay(3);
    error = abs(targetYaw - YAW);
    if(error < minError){
      minError = error;
    }

    if (robotState == robotSTOP) {
      stopRobot();
      break;
    }
  }
}

void resetRobot(String newState, bool slowDown = false) {
  if (robotState == robotSTOP) {
    return;
  }

  if (slowDown) {
    motor.setMotor1(1, minSpeed * 1.5);
    motor.setMotor2(1, minSpeed * 1.5);
    delay(125);
    motor.setMotor1(1, minSpeed);
    motor.setMotor2(1, minSpeed);
    delay(125);
  }
  motor.STOP();
  rodaPwm = startSpeed;
  robotState = newState;
}


void pilihMenu() {
  unsigned long currentTime = millis();
  if (currentTime - lastBacaPushButton > intervalBacaPushButton) {
    prevButtonState = digitalRead(prevButtonPin);
    nextButtonState = digitalRead(nextButtonPin);
    stopButtonState = digitalRead(stopButtonPin);

    //    printLCD((String(prevButtonState) + String(nextButtonState) + String(stopButtonState)), 0, 0);

    if (stopButtonState == LOW && stopLastButtonState != LOW) {
      stopRobot();
    }
    if (prevButtonState == LOW && prevLastButtonState != LOW && robotState == robotSTOP) {
      nextMenu();
    }
    if (nextButtonState == LOW && nextLastButtonState != LOW && robotState == robotSTOP) {
      statePrinter(1);
      startRobot();
      statePrinter(2);
    }
    lastBacaPushButton = currentTime;
  }
  prevLastButtonState = prevButtonState;
  nextLastButtonState = nextButtonState;
  stopLastButtonState = stopButtonState;
}

// Pembacaan Semua Sensor
void bacaSensor() {
  unsigned long currentTime = millis();
  if (currentTime - lastBacaSensor >= intervalBacaSensor) {
    bacaGyro();
    bacaKompas();
    YAW = alpha * lsmYaw + (1 - alpha) * avgGyYaw;
    printLCD("Arah:" + fixPrint(String(int(YAW)), 3), 8, 1);
    printLCD("Selisih:" + fixPrint(String(int(YAW) - targetArah), 4), 4, 0);
    lcdPrint("Arah:", 8, 1);
    lcdPrint(String(int(YAW)), -1, -1, 3);
    lcdPrint("Arah:", 4, 0);
    lcdPrint(String(int(YAW)-targetArah), -1, -1, 4);
    lastBacaSensor = currentTime;
  }
}

// Fungsi step menuju titik tuju
void oteweTarget() {
  // Step 1, pada pencarian sudut, apabila sudut sesuai, berhenti berputar
  if (robotState == ARAHTITIK) {
    cariArah(currentTarget());
    putarRobot(targetArah);
    if (robotState != robotSTOP) {
      resetRobot(MULAI);
      delay(3000);
      jarakUltrasonic = 999;
      rodaState = jalanMaju;
    }
  }

  // Step 2 gerak menuju titik target
  else if (robotState == MULAI) {
    // Jarak titik diketahui dengan sensor ultrasonik
    // Apabila sudah dekat, robot akan berhenti
    if (jarakUltrasonic < batasBedaJarak) {
      resetRobot(SAMPAI, true);
      gripperBuka();
      delay(2000);
      gripperTutup();
      robotState = ARAHBALIK;
    } else {
      // Membaca ultrasonik sampai jarak sesuai
      bacaUltrasonic();
    }
  }

  // Step 3 mencari titik sudut arah awal
  else if (robotState == ARAHBALIK) {
    cariArah(180);
    putarRobot(targetArah, 4);
    if(robotState != robotSTOP){
      resetRobot(PULANG);
      jarakUltrasonic = 999;
      rodaState = jalanMaju;
    }
  }

  // Step 4 kembali ke titik awal
  else if (robotState == PULANG) {
    if (jarakUltrasonic < batasBedaJarak) {
      motor.STOP();
      rodaState = rodaSTOP;
      robotState = robotSTOP;
    } else {
      bacaUltrasonic();
    }
  }
}

// Kendali motor sesuai arah state robot, maju, putar, mundur
void gerakMotor() {

  /////////////////////NORMALISASI GERAK LURUS
  float copyYaw = YAW;
  int copyTarget = currentTarget();
  int rodaPwmKanan = rodaPwm, rodaPwmKiri = rodaPwm;
  int fixPwm = 5;
  if (rodaState == jalanMaju) {
    // Normalisasi nilai arah
    if (copyTarget > 180 && copyYaw < 90) {
      copyYaw += 360;
    }
    if (copyYaw > 180 && copyTarget < 90) {
      copyTarget += 360;
    }
    // Fix kecepatan apabila terdapat eror
    if (copyYaw > copyTarget) {
      rodaPwmKanan += fixPwm;
    } else if (copyYaw < copyTarget) {
      rodaPwmKiri += fixPwm;
    }
    /////////////////////NORMALISASI GERAK LURUS

    motor.setMotor1(1, rodaPwmKanan);
    motor.setMotor2(1, rodaPwmKiri);
  } else if (rodaState == jalanMundur) {
    motor.setMotor1(0, rodaPwmKanan);
    motor.setMotor2(0, rodaPwmKiri);
  } else if (rodaState == putarKanan) {
    motor.setMotor1(0, rodaPwm + 2 - fixPwm);
    motor.setMotor2(1, rodaPwm - fixPwm);
  } else if (rodaState == putarKiri) {
    motor.setMotor1(1, rodaPwm + 2 - fixPwm);
    motor.setMotor2(0, rodaPwm - fixPwm);
  } else {
    motor.STOP();
  }
}


// Perulangan Utama
void loop() {
  bacaSensor();
  pilihMenu();
  oteweTarget();
  gerakMotor();
  delay(5);
}
