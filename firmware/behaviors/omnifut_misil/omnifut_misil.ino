/*  ===========================================================================
 *  OmniFut - MISIL LENTO  (ESP32-S3)
 *    Persigue la pelota como sumo: siempre avanza hacia ella, lento y constante.
 *    La pierde -> gira hacia el ultimo lado donde la vio hasta recuperarla.
 *
 *  PWM DIRECTO (el numero que pones = el que sale). Sin el viejo map() que inflaba.
 *  Arduino IDE -> Tools:
 *    Board "ESP32S3 Dev Module" | PSRAM "Disabled" | Flash "16MB (128Mb)"
 *    USB Mode "Hardware CDC and JTAG"
 *  =========================================================================== */

#include <Adafruit_NeoPixel.h>

// 0 = MISIL (perseguir/buscar)   |   1 = CALIBRAR motores (ruedas al aire)
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
#define CAM_RX 15
#define CAM_TX -1

#define PWM_FREQ 4000
#define PWM_RES  8
#define CENTER_X 80     // camara 160 de ancho -> centro 80
#define DEADBAND 3

// ===== VELOCIDAD  (todo en PWM real 0..255. EMPIEZA LENTO y sube de a 5) =====
const int MIN_MOVE = 35;   // minimo para que la rueda arranque. Si zumba y no gira: SUBE. Si va rapido: BAJA.
const int CRUISE   = 45;   // <<< velocidad de avance (super lento). Esta es tu perilla principal.
const int SEARCH   = 40;   // velocidad de giro al buscar.
const int TURN_CAP = 50;   // cuanto puede girar al corregir (no mas que esto).

// ===== AFINADO DEL SEGUIMIENTO ==============================================
const float KP_TURN     = 0.35f; // correccion de rumbo. Si gira al lado CONTRARIO de la pelota, ponlo NEGATIVO.
const int   CENTERED_PX = 16;    // si |x-80| < esto, ya esta centrada -> va recto, no corrige (no tiembla)
const int   ALIGN_PX    = 50;    // mas descentrada que esto -> gira fuerte y avanza poco
const float FWD_FLOOR   = 0.45f; // avance minimo aunque este girando (0..1). Que nunca se quede pivoteando.

const int   BALL_MIN_AREA   = 40;   // ignora manchas naranjas chiquitas (ruido)
const unsigned long BALL_TIMEOUT_MS = 250;
const unsigned long SWEEP_AFTER_MS  = 1500; // tras esto sin hallarla, el LED indica barrido completo

// ===== NeoPixel =============================================================
Adafruit_NeoPixel pixel(1, LEDPIN, NEO_GRB + NEO_KHZ800);
void led(uint8_t r, uint8_t g, uint8_t b) { pixel.setPixelColor(0, pixel.Color(r, g, b)); pixel.show(); }

// ===== MOTORES (0=FL 1=FR 2=BL 3=BR) ========================================
const int wP[4] = { FL_P, FR_P, BL_P, BR_P };
const int wN[4] = { FL_N, FR_N, BL_N, BR_N };
int       inv[4] = { -1, -1, -1, -1 };          // todas invertidas (tu robot iba al reves)
const char* wName[4] = { "FL", "FR", "BL", "BR" };

// PWM directo: el valor es la velocidad real, con un piso para que arranque.
void wheel(int i, int v) {
  v = constrain(v * inv[i], -255, 255);
  int a = abs(v);
  if (a < DEADBAND) { ledcWrite(wP[i], 0); ledcWrite(wN[i], 0); return; }
  int pwm = constrain(a, MIN_MOVE, 255);
  if (v > 0) { ledcWrite(wP[i], pwm); ledcWrite(wN[i], 0); }
  else       { ledcWrite(wP[i], 0);   ledcWrite(wN[i], pwm); }
}
void stopAll() { for (int i = 0; i < 4; i++) wheel(i, 0); }

void drive(int fwd, int turn) {     // turn>0 curva a un lado, turn<0 al otro
  int left  = fwd + turn;           // FL, BL
  int right = fwd - turn;           // FR, BR
  wheel(0, left);  wheel(2, left);
  wheel(1, right); wheel(3, right);
}

// ===== UART camara: "found,x,area\n" ========================================
char buf[48]; uint8_t len = 0;
int  bFound = 0, bX = 0, bArea = 0;
unsigned long lastBall = 0;
float errSmooth = 0;       // filtro para que el centro no salte
int   lastTurnDir = +1;    // hacia donde estaba girando para seguir la pelota

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
    wheel(i, 120); delay(1500); wheel(i, 0); delay(800);
  }
  Serial.println("ADELANTE"); drive(120, 0); delay(2000); stopAll(); delay(1000);
  Serial.println("GIRO");     drive(0, 120); delay(2000); stopAll(); delay(1500);
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
  while (digitalRead(PUSH)) { led(60, 40, 0); delay(150); led(0, 0, 0); delay(150); }  // espera START
  led(0, 0, 0);
}

// ===== LOOP =================================================================
void loop() {
  if (TEST_MODE) { motorTest(); return; }

  readCam();
  bool ballValid = bFound && (millis() - lastBall < BALL_TIMEOUT_MS) && (bArea > BALL_MIN_AREA);

  if (ballValid) {
    // --- PERSEGUIR (misil) ---
    int err = bX - CENTER_X;                 // + = pelota a la derecha
    errSmooth += 0.4f * (err - errSmooth);   // suaviza el ruido
    int e = (int)errSmooth;

    int turn = 0;
    if (abs(e) >= CENTERED_PX) {             // si no esta centrada, corrige; si lo esta, va recto
      turn = constrain((int)(KP_TURN * e), -TURN_CAP, TURN_CAP);
      lastTurnDir = (turn >= 0) ? +1 : -1;
    }
    // avance: maximo centrada, pero NUNCA se apaga del todo (siempre empuja)
    float align = 1.0f - constrain(fabsf((float)e) / ALIGN_PX, 0.0f, 1.0f);
    int fwd = (int)(CRUISE * (FWD_FLOOR + (1.0f - FWD_FLOOR) * align));

    drive(fwd, turn);
    led(0, abs(e) < CENTERED_PX ? 150 : 60, 0);   // verde fuerte centrada, tenue corrigiendo
  } else {
    // --- BUSCAR: gira hacia el ultimo lado donde la vio, sin parar ---
    drive(0, SEARCH * lastTurnDir);
    if (millis() - lastBall < SWEEP_AFTER_MS) led(60, 25, 0);   // naranja: buscando cerca
    else                                      led(80, 0, 60);   // morado: barriendo todo
  }
}
