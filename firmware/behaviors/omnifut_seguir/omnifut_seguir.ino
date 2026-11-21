/*  ===========================================================================
 *  OmniFut - SEGUIR LA PELOTA  (ESP32-S3)
 *    Ve la pelota  -> avanza hacia ella corrigiendo el rumbo (de frente).
 *    No la ve       -> se queda quieto.
 *    SIN stop_area: nunca frena por distancia, sigue empujando mientras la vea.
 *
 *  Arduino IDE -> Tools:
 *    Board "ESP32S3 Dev Module" | PSRAM "Disabled" | Flash "16MB (128Mb)"
 *    USB Mode "Hardware CDC and JTAG"
 *  =========================================================================== */

#include <Adafruit_NeoPixel.h>

// 0 = SEGUIR pelota   |   1 = CALIBRAR motores (ruedas al aire)
#define TEST_MODE 0

// ===== PINES (de tu PCB) ====================================================
#define PUSH   45
#define LEDPIN  3

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
#define MIN_PWM  60    // duty minimo para que la rueda arranque (sube si alguna no gira)
#define DEADBAND 4
#define CENTER_X 80    // la camara es 160 de ancho -> centro 80

// ===== AJUSTES QUE PUEDES TUNEAR ===========================================
const int   FWD_SPEED = 150;    // velocidad de avance 0..255
const float KP_TURN   = 0.6f;   // <-- si gira al lado CONTRARIO de la pelota, ponlo NEGATIVO (-0.6f)
const unsigned long BALL_TIMEOUT_MS = 250;

// ===== NeoPixel =============================================================
Adafruit_NeoPixel pixel(1, LEDPIN, NEO_GRB + NEO_KHZ800);
void led(uint8_t r, uint8_t g, uint8_t b) { pixel.setPixelColor(0, pixel.Color(r, g, b)); pixel.show(); }

// ===== MOTORES (0=FL 1=FR 2=BL 3=BR) ========================================
const int   wP[4]  = { FL_P, FR_P, BL_P, BR_P };
const int   wN[4]  = { FL_N, FR_N, BL_N, BR_N };
int         inv[4] = {  -1,   -1,   -1,   -1 };   // todas invertidas (tu robot iba al reves)
const char* wName[4] = { "FL", "FR", "BL", "BR" };

void wheel(int i, int v) {                  // v: -255..255
  v = constrain(v * inv[i], -255, 255);
  if (abs(v) < DEADBAND) { ledcWrite(wP[i], 0); ledcWrite(wN[i], 0); return; }
  int pwm = map(abs(v), DEADBAND, 255, MIN_PWM, 255);
  if (v > 0) { ledcWrite(wP[i], pwm); ledcWrite(wN[i], 0); }
  else       { ledcWrite(wP[i], 0);   ledcWrite(wN[i], pwm); }
}
void stopAll() { for (int i = 0; i < 4; i++) wheel(i, 0); }

// avance recto + giro suave
void drive(int fwd, int turn) {
  int left  = fwd + turn;   // FL, BL
  int right = fwd - turn;   // FR, BR
  wheel(0, left);  wheel(2, left);
  wheel(1, right); wheel(3, right);
}

// ===== UART camara: "found,x,area\n" (el area se lee pero ya NO se usa) ======
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
    } else if (len < sizeof(buf) - 1) buf[len++] = c;
    else len = 0;
  }
}

// ===== CALIBRACION ==========================================================
void motorTest() {
  led(0, 0, 40);
  for (int i = 0; i < 4; i++) {
    Serial.printf("Rueda %s -> adelante\n", wName[i]);
    wheel(i, 160); delay(1500); wheel(i, 0); delay(800);
  }
  Serial.println("ADELANTE"); drive(150, 0); delay(2000); stopAll(); delay(1000);
  Serial.println("GIRO");     drive(0, 150); delay(2000); stopAll(); delay(1500);
}

// ===== SETUP ================================================================
void setup() {
  Serial.begin(115200);
  Serial1.setRxBufferSize(512);
  Serial1.begin(115200, SERIAL_8N1, CAM_RX, CAM_TX);
  pinMode(PUSH, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) { ledcAttach(wP[i], PWM_FREQ, PWM_RES); ledcAttach(wN[i], PWM_FREQ, PWM_RES); }
  stopAll();
  pixel.begin();

  if (TEST_MODE) { led(0, 0, 40); return; }

  // espera el boton START (LED parpadea amarillo = vivo y esperando)
  while (digitalRead(PUSH)) { led(60, 40, 0); delay(150); led(0, 0, 0); delay(150); }
  led(0, 0, 0);
}

// ===== LOOP =================================================================
void loop() {
  if (TEST_MODE) { motorTest(); return; }

  readCam();
  bool ballValid = bFound && (millis() - lastBall < BALL_TIMEOUT_MS);

  if (ballValid) {
    int err  = bX - CENTER_X;            // + = pelota a la derecha
    int turn = (int)(KP_TURN * err);     // corrige para centrarla
    drive(FWD_SPEED, turn);              // avanza hacia ella
    led(0, 120, 0);                      // verde = persiguiendo
  } else {
    stopAll();
    led(40, 0, 0);                       // rojo = sin pelota
  }
}
