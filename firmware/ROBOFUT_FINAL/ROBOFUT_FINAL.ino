#include <Adafruit_NeoPixel.h>

// ─── PIN DEFINITIONS ────────────────────────────────────────────────────────
#define BNO08X_CS 10
#define BNO08X_INT 5
#define BNO08X_RESET -1
#define SDA0_Pin 47  
#define SCL0_Pin 48

#define PUSH 45
#define BATTERY 9
#define LED 03

#define MOTOR_BACK_RIGTH_PWM_P 17
#define MOTOR_BACK_RIGTH_PWM_N 18
#define MOTOR_BACK_RIGTH_A 36
#define MOTOR_BACK_RIGTH_B 8

#define MOTOR_FRONT_RIGTH_PWM_P 6
#define MOTOR_FRONT_RIGTH_PWM_N 7
#define MOTOR_FRONT_RIGTH_A 4
#define MOTOR_FRONT_RIGTH_B 5

#define MOTOR_BACK_LEFT_PWM_P 37
#define MOTOR_BACK_LEFT_PWM_N 38
#define MOTOR_BACK_LEFT_A 39
#define MOTOR_BACK_LEFT_B 40

#define MOTOR_FRONT_LEFT_PWM_P 01
#define MOTOR_FRONT_LEFT_PWM_N 02
#define MOTOR_FRONT_LEFT_A 41
#define MOTOR_FRONT_LEFT_B 42

#define FREQUENCY_PWM 4000
#define RESOLUTION_PWM 8

// ─── PARÁMETROS DE POTENCIA Y ZONA MUERTA ──────────────────────────────────
#define PWM_MAX 255

// MAGIA: Este es el empujón mínimo para vencer la fricción. 
// Si los motores zumban pero no arrancan, sube este número a 80 o 90.
#define MIN_PWM 70  

Adafruit_NeoPixel pixels(1, LED, NEO_GRB + NEO_KHZ800);

// ─── REMOTEXY ───────────────────────────────────────────────────────────────
#define REMOTEXY_MODE__ESP32CORE_BLE
#include <BLEDevice.h>
#define REMOTEXY_BLUETOOTH_NAME "OmniFut"
#include <RemoteXY.h>

#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =  
  { 255, 4, 0, 0, 0, 41, 0, 19, 0, 0, 0, 0, 31, 2, 106, 200, 200, 84, 1, 1,
    2, 0, 5, 210, 12, 143, 143, 25, 8, 60, 60, 32, 2, 26, 31, 5, 7, 19, 143, 143,
    118, 8, 60, 60, 32, 2, 26, 31 };

struct {
  int8_t joystick_01_x;  
  int8_t joystick_01_y;  
  int8_t joystick_02_x;  
  int8_t joystick_02_y;  
  uint8_t connect_flag;  
} RemoteXY;
#pragma pack(pop)

// ─── VARIABLES DE VISIÓN (CÁMARA) ──────────────────────────────────────────
String inputString = "";
bool stringComplete = false;
int x = 0;
int y = 0;

// Constantes de cámara (ASUMIENDO RESOLUCIÓN 320x240)
const int CENTER_X = 160; 
const int CENTER_Y = 120; 

// Variables del Controlador PD
float errorAnteriorX = 0;
float errorAnteriorY = 0;

// ────────────────────────────────────────────────────────────────────────────
// SETUP
// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // Conexión a la cámara
  inputString.reserve(50);
  RemoteXY_Init();  
  delay(500);

  pinMode(PUSH, INPUT_PULLUP);

  ledcAttach(MOTOR_BACK_RIGTH_PWM_P, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_BACK_RIGTH_PWM_N, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcWrite(MOTOR_BACK_RIGTH_PWM_P, 0);
  ledcWrite(MOTOR_BACK_RIGTH_PWM_N, 0);

  ledcAttach(MOTOR_FRONT_RIGTH_PWM_P, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_FRONT_RIGTH_PWM_N, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcWrite(MOTOR_FRONT_RIGTH_PWM_P, 0);
  ledcWrite(MOTOR_FRONT_RIGTH_PWM_N, 0);

  ledcAttach(MOTOR_BACK_LEFT_PWM_P, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_BACK_LEFT_PWM_N, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcWrite(MOTOR_BACK_LEFT_PWM_P, 0);
  ledcWrite(MOTOR_BACK_LEFT_PWM_N, 0);

  ledcAttach(MOTOR_FRONT_LEFT_PWM_P, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcAttach(MOTOR_FRONT_LEFT_PWM_N, FREQUENCY_PWM, RESOLUTION_PWM);
  ledcWrite(MOTOR_FRONT_LEFT_PWM_P, 0);
  ledcWrite(MOTOR_FRONT_LEFT_PWM_N, 0);

  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.show();
  
  while (digitalRead(PUSH));
  
  for (int k = 0; k < 4; k++) {
    pixels.setPixelColor(0, pixels.Color(150, 150, 0));
    pixels.show(); delay(500);
    pixels.clear(); pixels.show(); delay(500);
  }
}

// ────────────────────────────────────────────────────────────────────────────
// LOOP PRINCIPAL
// ────────────────────────────────────────────────────────────────────────────
void loop() {
  readSerial();
  parseString();
  RemoteXYEngine.handler();

  float Vx = 0.0;
  float Vy = 0.0;
  float W = RemoteXY.joystick_02_x / 100.0; // Rotación manual en el joystick

  // Si la cámara ve la pelota (asumiendo que manda 0,0 si la pierde)
  if (x != 0 && y != 0) {
    
    // --- CONTROLADOR PD (Proporcional - Derivativo) ---
    // Si persigue la pelota muy lento, sube kpX y kpY a 0.005 o 0.01
    float kpX = 0.003; 
    float kdX = 0.001;
    float kpY = 0.003; 
    float kdY = 0.001;

    // Calculamos el error en pixeles
    float errorX = CENTER_X - x; 
    float errorY = CENTER_Y - y; 

    // Calculamos el cambio de error (Derivativo)
    float dErrorX = errorX - errorAnteriorX;
    float dErrorY = errorY - errorAnteriorY;

    // Generamos potencia en rango [-1.0 a 1.0]
    Vx = (kpX * errorX) + (kdX * dErrorX);
    Vy = (kpY * errorY) + (kdY * dErrorY);

    // Si el robot se aleja de la pelota en lugar de acercarse, descomenta estas líneas:
    // Vx = -Vx; 
    // Vy = -Vy;

    // Límite de velocidad: 0.4 significa 40% de su velocidad máxima al perseguir
    float maxTrackingSpeed = 0.4; 
    Vx = constrain(Vx, -maxTrackingSpeed, maxTrackingSpeed);
    Vy = constrain(Vy, -maxTrackingSpeed, maxTrackingSpeed);

    errorAnteriorX = errorX;
    errorAnteriorY = errorY;

  } else {
    // MODO MANUAL: Si no ve la pelota, obedece al Joystick
    Vx = -RemoteXY.joystick_01_x / 100.0;
    Vy = RemoteXY.joystick_01_y / 100.0;
    
    errorAnteriorX = 0;
    errorAnteriorY = 0;
  }

  // Única llamada a motores
  omniDrive(Vx, Vy, W);
}

// ────────────────────────────────────────────────────────────────────────────
// FUNCIONES DE MOTORES (CON ZONA MUERTA INCORPORADA)
// ────────────────────────────────────────────────────────────────────────────
void omniDrive(float Vx, float Vy, float W) {
  // Cinemática inversa para tus Omniwheels
  float m1 = Vx - Vy - W; // Front Left
  float m2 = Vx + Vy + W; // Front Right
  float m3 = Vx - Vy + W; // Back Right
  float m4 = Vx + Vy - W; // Back Left

  // Normalización (Asegura que los valores se mantengan entre -1.0 y 1.0)
  float maxVal = max({abs(m1), abs(m2), abs(m3), abs(m4)});
  if (maxVal > 1.0) {
    m1 /= maxVal;
    m2 /= maxVal;
    m3 /= maxVal;
    m4 /= maxVal;
  }

  // Convertir a PWM Teórico
  int pwmMotor1 = (int)(m1 * PWM_MAX);
  int pwmMotor2 = (int)(m2 * PWM_MAX);
  int pwmMotor3 = (int)(m3 * PWM_MAX);
  int pwmMotor4 = (int)(m4 * PWM_MAX);

  setMotors(pwmMotor4, pwmMotor3, pwmMotor1, pwmMotor2);
}

void setMotors(int vMotorBackLeft, int vMotorBackRigth, int vMotorFrontLeft, int vMotorFrontRigth) {
  setMotorBackLeft(-vMotorBackLeft);
  setMotorBackRigth(vMotorBackRigth);
  setMotorFrontkLeft(vMotorFrontLeft);
  setMotorFrontRigth(-vMotorFrontRigth);
}

void setMotorBackRigth(int v)  { setMotor(v, MOTOR_BACK_RIGTH_PWM_P, MOTOR_BACK_RIGTH_PWM_N); }
void setMotorFrontRigth(int v) { setMotor(v, MOTOR_FRONT_RIGTH_PWM_P, MOTOR_FRONT_RIGTH_PWM_N); }
void setMotorBackLeft(int v)   { setMotor(v, MOTOR_BACK_LEFT_PWM_P, MOTOR_BACK_LEFT_PWM_N); }
void setMotorFrontkLeft(int v) { setMotor(v, MOTOR_FRONT_LEFT_PWM_P, MOTOR_FRONT_LEFT_PWM_N); }

void setMotor(int v, int motorPositive, int motorNegative) {
  v = constrain(v, -255, 255);
  int pwm = 0;
  
  // MAPEO DE ZONA MUERTA (Evita que el motor se quede zumbando sin arrancar)
  if (v != 0) {
    pwm = map(abs(v), 1, 255, MIN_PWM, 255);
  }

  if (v > 0) {
    ledcWrite(motorPositive, pwm);
    ledcWrite(motorNegative, 0);
  } else if (v < 0) {
    ledcWrite(motorPositive, 0);
    ledcWrite(motorNegative, pwm);
  } else {
    ledcWrite(motorPositive, 0);
    ledcWrite(motorNegative, 0);
  }
}

// ────────────────────────────────────────────────────────────────────────────
// FUNCIONES DE SERIAL / CÁMARA
// ────────────────────────────────────────────────────────────────────────────
void readSerial() {
  while (Serial1.available()) {
    char inChar = (char)Serial1.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseString() {
  if (stringComplete) {
    if (inputString.startsWith("(") && inputString.endsWith(")\n")) {
      int commaIndex = inputString.indexOf(',');
      if (commaIndex > 0) {
        String x_str = inputString.substring(1, commaIndex);
        String y_str = inputString.substring(commaIndex + 1, inputString.length() - 2);
        x = x_str.toInt();
        y = y_str.toInt();
      }
    }
    inputString = "";
    stringComplete = false;
  }
}