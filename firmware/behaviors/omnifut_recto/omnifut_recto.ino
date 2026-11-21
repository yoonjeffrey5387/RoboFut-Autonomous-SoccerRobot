/*  ===========================================================================
 *  OmniFut - PERSEGUIR PELOTA EN LINEA RECTA  (ESP32-S3)
 *  Logica nueva y minima. Solo conserva los pines de tu PCB.
 *
 *  Comportamiento:
 *    - Ve la pelota  -> avanza recto hacia ella, corrigiendo apenas el rumbo
 *                       para mantenerla centrada (de frente).
 *    - No la ve       -> se queda quieto.
 *
 *  Arduino IDE -> Tools:
 *    Board "ESP32S3 Dev Module" | PSRAM "Disabled" | Flash "16MB (128Mb)"
 *    USB Mode "Hardware CDC and JTAG"
 *  =========================================================================== */

#include <Adafruit_NeoPixel.h>

// ===== MODO =================================================================
//  1 = CALIBRAR motores (ruedas al aire). 0 = PERSEGUIR.
#define TEST_MODE 1

// ===== PINES (de tu PCB) ====================================================
#define PUSH   45
#define LEDPIN  3

// Cada motor: pin P y pin N (doble PWM)
#define FL_P 1
#define FL_N 2
#define FR_P 6
#define FR_N 7
#define BL_P 37
#define BL_N 38
#define BR_P 17
#define BR_N 18

#define CAM_RX 15      // RX del ESP32 <- TX (P4) de la camara
#define CAM_TX -1

#define PWM_FREQ 4000
#define PWM_RES  8
#define MIN_PWM  60    // duty minimo para que la rueda arranque (sube si no gira)
#define DEADBAND 4

// ===== VISION (igual que la camara: QQVGA 160x120) ==========================
#define CENTER_X 80

// ===== AJUSTES QUE VAS A TUNEAR =============================================
const int   FWD_SPEED = 150;    // velocidad de avance 0..255
const float KP_TURN   = 0.6f;   // correccion de rumbo (sube si no centra, baja si tiembla)
const int   STOP_AREA = 3000;   // si la pelota se ve mas grande que esto, ya llego -> frena
const unsigned long BALL_TIMEOUT_MS = 250;   // si no llegan datos frescos, pelota perdida

// ===== NeoPixel (solo para ver el estado) ===================================
Adafruit_NeoPixel pixel(1, LEDPIN, NEO_GRB + NEO_KHZ800);
void led(uint8_t r, uint8_t g, uint8_t b) { pixel.setPixelColor(0, pixel.Color(r, g, b)); pixel.show(); }

// ===== MOTORES  (indices: 0=FL 1=FR 2=BL 3=BR) ==============================
const int   wP[4]  = { FL_P, FR_P, BL_P, BR_P };
const int   wN[4]  = { FL_N, FR_N, BL_N, BR_N };
int         inv[4] = {  +1,   +1,   +1,   +1 };   // <-- pon -1 al que gire al reves
const char* wName[4] = { "FL", "FR", "BL", "BR" };

void wheel(int i, int v) {                 // v: -255..255
  v = constrain(v * inv[i], -255, 255);
  if (abs(v) < DEADBAND) { ledcWrite(wP[i], 0); ledcWrite(wN[i], 0); return; }
  int pwm = map(abs(v), DEADBAND, 255, MIN_PWM, 255);
  if (v > 0) { ledcWrite(wP[i], pwm); ledcWrite(wN[i], 0); }
  else       { ledcWrite(wP[i], 0);   ledcWrite(wN[i], pwm); }
}
void stopAll() { for (int i = 0; i < 4; i++) wheel(i, 0); }

// Avance recto + giro suave (mezcla diferencial sobre la base omni).
//   fwd  = adelante (+)
//   turn = girar (+ = hacia un lado)
void drive(int fwd, int turn) {
  int left  = fwd + turn;   // ruedas izquierdas: FL, BL
  int right = fwd - turn;   // ruedas derechas:   FR, BR
  wheel(0, left);   // FL
  wheel(2, left);   // BL
  wheel(1, right);  // FR
  wheel(3, right);  // BR
}

// ===== UART camara:  "found,x,area\n" =======================================
char buf[48]; uint8_t len = 0;
int  bFound = 0, bX = 0, bArea = 0;
unsigned long lastBall = 0;

void readCam() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      buf[len] = '\0';
      int f, x, a;
      if (sscanf(buf, "%d,%d,%d", &f, &x, &a) == 3) {
        bFound = f; bX = x; bArea = a;
        if (f) lastBall = millis();
      }
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else len = 0;
  }
}

// ===== CALIBRACION ==========================================================
void motorTest() {
  led(0, 0, 40);
  for (int i = 0; i < 4; i++) {       // confirma el sentido de giro de cada rueda
    Serial.printf("Rueda %s -> adelante\n", wName[i]);
    wheel(i, 160); delay(1500); wheel(i, 0); delay(800);
  }
  Serial.println("ADELANTE (todas)"); drive(150, 0); delay(2000); stopAll(); delay(1000);
  Serial.println("GIRO");             drive(0, 150); delay(2000); stopAll(); delay(1500);
}

// ===== SETUP ================================================================
void setup() {
  Serial.begin(115200);
  Serial1.setRxBufferSize(512);
  Serial1.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
  pinMode(PUSH, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) { ledcAttach(wP[i], PWM_FREQ, PWM_RES); ledcAttach(wN[i], PWM_FREQ, PWM_RES); }
  stopAll();
  pixel.begin(); led(0, 40, 0);

  Serial.println("\nOmniFut - perseguir recto");
  if (TEST_MODE) { Serial.println(">> MODO CALIBRACION <<"); return; }

  Serial.println("Pulsa START para arrancar...");
  while (digitalRead(PUSH)) delay(10);
  led(0, 0, 0);
}

// ===== LOOP =================================================================
void loop() {
  if (TEST_MODE) { motorTest(); return; }

  readCam();
  bool ballValid = bFound && (millis() - lastBall < BALL_TIMEOUT_MS);

  if (ballValid) {
    if (bArea >= STOP_AREA) {                 // ya llegamos a la pelota
      stopAll(); led(0, 0, 150);              // azul
    } else {
      int err  = bX - CENTER_X;               // + = pelota a la derecha
      int turn = (int)(KP_TURN * err);        // gira para centrarla
      drive(FWD_SPEED, turn);                 // avanza recto corrigiendo rumbo
      led(0, 120, 0);                         // verde
    }
  } else {
    stopAll(); led(40, 0, 0);                 // sin pelota -> quieto (rojo)
  }

  static unsigned long t = 0;
  if (millis() - t > 200) { t = millis();
    Serial.printf("f=%d x=%3d a=%5d valid=%d\n", bFound, bX, bArea, ballValid);
  }
}
