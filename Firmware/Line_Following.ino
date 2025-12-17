#include <EEPROM.h>
#include <Arduino.h> 

// =======================================================
// 1. DEFINISI PIN & KONSTANTA UTAMA
// =======================================================

// --- Pin Motor (H-Bridge) ---
#define pin_pwm_motor_L 5   // D6 -> INPUT1/PWM Kiri
#define pin_dir_motor_L 11  // D10 -> INPUT2/DIR Kiri
#define pin_pwm_motor_R 6   // D5 -> INPUT3/PWM Kanan
#define pin_dir_motor_R 10  // D11 -> INPUT4/DIR Kanan

// --- Pin Sensor (MUX CD4051) ---
#define MUX_OUT_PIN A0      // Output MUX (Sensor DAT) ke Pin Analog A0
#define S0_PIN 2            // Pin Kontrol S0 (DAT_C)
#define S1_PIN 3            // Pin Kontrol S1 (DAT_B)
#define S2_PIN 4            // Pin Kontrol S2 (DAT_A)
const int NUM_SENSORS = 8;

// --- Pin Tombol (INPUT_PULLUP / Active LOW) ---
#define BUTTON_START_PIN 7  // Pin PB1 (D7) -> START Line Follower
#define BUTTON_WHITE_PIN A1 // Pin PB2 (A1) -> KALIBRASI PUTIH
#define BUTTON_BLACK_PIN A2 // Pin PB3 (A2) -> KALIBRASI HITAM

// --- Konstanta PID & Kecepatan ---
float Kp = 11;   
float Ki = 0;     
float Kd = 15;   
int BASE_SPEED = 200; 
#define MAX_PWM 255

// --- Konstanta Kalibrasi ---
const int CALIBRATION_TIME_MS = 3000; 
const int CALIBRATION_DELAY_MS = 1000; 

// --- Variabel Global ---
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
float PID_output = 0;

int sensorValues[NUM_SENSORS];
int minValues[NUM_SENSORS];   
int maxValues[NUM_SENSORS];   
int thresholdValues[NUM_SENSORS]; 

// --- Variabel untuk State Machine ---
bool robotIsRunning = false; // Status robot (true = jalan, false = diam)


// =======================================================
// 2. FUNGSI KONTROL MOTOR
// =======================================================

void setMotor(int target_PWM_L, int target_PWM_R) {
  // Batasi nilai PWM agar tidak melebihi batas
  target_PWM_L = constrain(target_PWM_L, -MAX_PWM, MAX_PWM);
  target_PWM_R = constrain(target_PWM_R, -MAX_PWM, MAX_PWM);

  // --- Motor Kanan ---
  if (target_PWM_R >= 0) { 
    digitalWrite(pin_dir_motor_R, LOW); // Maju
    analogWrite(pin_pwm_motor_R, target_PWM_R);
  } else { 
    digitalWrite(pin_dir_motor_R, HIGH); // Mundur
    analogWrite(pin_pwm_motor_R, abs(target_PWM_R));
  }
  
  // --- Motor Kiri ---
  if (target_PWM_L >= 0) { 
    digitalWrite(pin_dir_motor_L, LOW); // Maju
    analogWrite(pin_pwm_motor_L, target_PWM_L);
  } else { 
    digitalWrite(pin_dir_motor_L, HIGH); // Mundur
    analogWrite(pin_pwm_motor_L, abs(target_PWM_L));
  }
}

void stopMotorHard() {
  analogWrite(pin_pwm_motor_L, 0);
  analogWrite(pin_pwm_motor_R, 0);
}


// =======================================================
// 3. FUNGSI SENSOR & LOGIKA POSISI
// =======================================================

void selectMuxChannel(int channel) {
  digitalWrite(S0_PIN, bitRead(channel, 0));
  digitalWrite(S1_PIN, bitRead(channel, 1));
  digitalWrite(S2_PIN, bitRead(channel, 2));
}

void readSensors() {
  for (int i = 0; i < NUM_SENSORS; i++) {
    selectMuxChannel(i);
    delayMicroseconds(10); 
    sensorValues[i] = analogRead(MUX_OUT_PIN);
  }
}

int calculateLinePosition() {
  long weightedSum = 0;
  int lineSensorCount = 0;
  
  // Urutan bobot untuk menentukan posisi garis
  int weights[] = {7000, 5000, 3000, 1000, -1000, -3000, -5000, -7000}; 

  readSensors();
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    // Jika > Threshold = 1 (Garis), Jika < Threshold = 0 (Lantai)
    int value = (sensorValues[i] > thresholdValues[i]) ? 1 : 0;
    
    if (value == 1) {
      weightedSum += (long)weights[i];
      lineSensorCount++;
    }
  }

  // --- [UPDATE] LOGIKA BERHENTI DI GARIS FINISH (8 SENSOR HITAM) ---
  // Kita pakai >= 7 untuk toleransi jika 1 sensor agak meleset
  if (lineSensorCount >= 7) { 
     stopMotorHard();        // Rem mendadak
     robotIsRunning = false; // Mematikan status jalan (STOP TOTAL)
     return 0;               // Return 0 agar tidak error
  }

  // --- LOGIKA GARIS PUTUS-PUTUS ---
  if (lineSensorCount == 0) {
    // Jika sebelumnya error kecil (robot stabil), anggap garis putus-putus -> Lurus
    if (abs(lastError) < 2000) { 
      return 0; 
    }
    // Jika sebelumnya belok tajam, anggap keluar jalur -> Belok Patah
    if (lastError > 0) return 8000; 
    else return -8000; 
  }
  
  return weightedSum / lineSensorCount; 
}


// =======================================================
// 4. FUNGSI KALIBRASI
// =======================================================

void calibration() {
  Serial.println("\n--- MODE KALIBRASI ---");
  stopMotorHard();
  
  for(int i = 0; i < NUM_SENSORS; i++) {
    minValues[i] = 1023;
    maxValues[i] = 0;     
  }
  
  // 1. KALIBRASI PUTIH
  Serial.println("1. Tekan tombol A1 (Putih). Pindahkan robot di atas bidang putih.");
  while (digitalRead(BUTTON_WHITE_PIN) == HIGH); 
  delay(200); 
  Serial.println("   Mulai Kalibrasi PUTIH...");
  unsigned long startTime = millis();
  while ((millis() - startTime) < CALIBRATION_TIME_MS) {
    readSensors();
    for(int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] < minValues[i]) minValues[i] = sensorValues[i];
    }
  }
  Serial.println("   Selesai Putih.");
  delay(CALIBRATION_DELAY_MS); 

  // 2. KALIBRASI HITAM
  Serial.println("2. Tekan tombol A2 (Hitam). Pindahkan robot di atas garis hitam.");
  while (digitalRead(BUTTON_BLACK_PIN) == HIGH); 
  delay(200); 
  Serial.println("   Mulai Kalibrasi HITAM...");
  startTime = millis();
  while ((millis() - startTime) < CALIBRATION_TIME_MS) {
    readSensors();
    for(int i = 0; i < NUM_SENSORS; i++) {
      if (sensorValues[i] > maxValues[i]) maxValues[i] = sensorValues[i];
    }
  }
  Serial.println("   Selesai Hitam.");
  delay(CALIBRATION_DELAY_MS); 

  // 3. SIMPAN
  Serial.println("3. Tekan Tombol D7 (Start) untuk Simpan...");
  while (digitalRead(BUTTON_START_PIN) == HIGH); 
  delay(200); 
  
  for(int i = 0; i < NUM_SENSORS; i++) {
    thresholdValues[i] = (minValues[i] + maxValues[i]) / 2;
    EEPROM.put(i * sizeof(int), thresholdValues[i]); 
  }
  Serial.println("Tersimpan! Siap Start.");
  delay(2000);
}

void loadThreshold() {
  bool is_calibrated = true;
  for(int i = 0; i < NUM_SENSORS; i++) {
    EEPROM.get(i * sizeof(int), thresholdValues[i]);
    if (thresholdValues[i] < 50 || thresholdValues[i] > 950) {
      thresholdValues[i] = 512; 
      is_calibrated = false;
    }
  }
  if (!is_calibrated) Serial.println("PERINGATAN: Belum Kalibrasi!");
}


// =======================================================
// 5. FUNGSI PID
// =======================================================

void calculatePID() {
  // calculateLinePosition sekarang bisa mengubah status robotIsRunning jadi false
  int position = calculateLinePosition();
  
  // Jika robot sudah disuruh berhenti oleh sensor, reset error
  if (!robotIsRunning) {
      error = 0;
      lastError = 0;
      PID_output = 0;
      return;
  }

  error = position; 
  
  float P = Kp * error; 
  
  integral += error;
  integral = constrain(integral, -3000, 3000); 
  float I = Ki * integral;
  
  derivative = error - lastError;
  float D = Kd * derivative;
  
  PID_output = P + I + D;
  lastError = error;
}

void applyPID() {
  calculatePID();

  // --- [UPDATE] PENCEGAHAN MOTOR JALAN SETELAH FINISH ---
  // Jika calculatePID mendeteksi finish line, robotIsRunning jadi false.
  // Kita harus langsung return agar setMotor di bawah tidak dijalankan.
  if (!robotIsRunning) return; 

  float turn = PID_output; 
  
  int power_L = BASE_SPEED + (int)turn;
  int power_R = BASE_SPEED - (int)turn;
  
  setMotor(power_L, power_R);
}


// =======================================================
// 6. SETUP & LOOP
// =======================================================

void setup() {
  Serial.begin(9600);
  
  pinMode(pin_pwm_motor_L, OUTPUT);
  pinMode(pin_dir_motor_L, OUTPUT);
  pinMode(pin_pwm_motor_R, OUTPUT);
  pinMode(pin_dir_motor_R, OUTPUT);
  
  pinMode(S0_PIN, OUTPUT);
  pinMode(S1_PIN, OUTPUT);
  pinMode(S2_PIN, OUTPUT);
  
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_WHITE_PIN, INPUT_PULLUP);
  pinMode(BUTTON_BLACK_PIN, INPUT_PULLUP);

  loadThreshold();
  stopMotorHard(); 
  
  Serial.println("Robot Siap. Tekan D7 untuk Start.");
}

void loop() {
  // 1. Cek Kalibrasi
  if (digitalRead(BUTTON_WHITE_PIN) == LOW || digitalRead(BUTTON_BLACK_PIN) == LOW) {
    stopMotorHard(); 
    robotIsRunning = false; 
    calibration();
    delay(500); 
  }
  
  // 2. Cek Start/Stop Manual (Tombol)
  if (digitalRead(BUTTON_START_PIN) == LOW) {
    delay(200); 
    robotIsRunning = !robotIsRunning; 
    
    if (robotIsRunning) {
      Serial.println("START!");
      lastError = 0; 
      integral = 0;
    } else {
      Serial.println("STOP.");
      stopMotorHard(); 
    }
    while(digitalRead(BUTTON_START_PIN) == LOW); 
  }

  // 3. Jalankan Robot
  if (robotIsRunning) {
    applyPID();
  } else {
    stopMotorHard();
  }
}