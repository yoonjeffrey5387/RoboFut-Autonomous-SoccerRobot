/*  ===========================================================================
 *  OmniFut - SEGUIR + BUSCAR  (ESP32-S3)
 *    Ve la pelota -> gira para encararla y avanza despacio (mas lento si esta descentrada).
 *    La pierde     -> la busca girando hacia el ultimo lado donde la vio;
 *                     si tarda, hace un barrido de 360 grados.
 *
 *  Arduino IDE -> Tools:
 *    Board "ESP32S3 Dev Module" | PSRAM "Disabled" | Flash "16MB (128Mb)"
 *    USB Mode "Hardware CDC and JTAG"
 *  =========================================================================== */

#include <Adafruit_NeoPixel.h>

// 0 = SEGUIR/BUSCAR   |   1 = CALIBRAR motores (ruedas al aire)
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
#define MIN_PWM  60     // duty minimo para que la rueda arranque (sube si alguna no gira)
#define DEADBAND 4
#define CENTER_X 80     // camara 160 de ancho -> centro 80

// ===== AJUSTES PRINCIPALES (lo que vas a tocar) =============================
const int   FWD_SPEED   = 70;    // <<< velocidad de avance MUY baja (antes 150). Sube de a 10.
const float KP_TURN     = 0.55f; // correccion de rumbo. Si gira al lado contrario, ponlo NEGATIVO.
const int   CENTERED_PX = 14;    // |x-80| menor que esto = "centrada" -> avanza de frente
const int   ALIGN_PX    = 45;    // si esta mas descentrada que esto, casi no avanza (solo gira)

const int   TURN_MAX    = 90;    // tope de cuanto puede girar mientras persigue (0..255)
const unsigned long BALL_TIMEOUT_MS = 250;

// ===== BUSQUEDA =============================================================
const int           SEARCH_SPIN   = 20;    // velocidad de giro al buscar (baja = no se pasa)
const unsigned long SWEEP_AFTER_MS = 1500; // si busca mas de esto sin hallarla, barre 360
int                 lastDir = +1;          // ultimo lado donde se vio (+1 der, -1 izq)

// ===== NeoPixel =============================================================
Adafruit_NeoPixel pixel(1, LEDPIN, NEO_GRB + NEO_KHZ800);
void led(uint8_t r, uint8_t g, uint8_t b) { pixel.setPixelColor(0, pixel.Color(r, g, b)); pixel.show(); }

// ===== MOTORES (0=FL 1=FR 2=BL 3=BR) ========================================
const int wP[4] = { FL_P, FR_P, BL_P, BR_P };
const int wN[4] = { FL_N, FR_N, BL_N, BR_N };
int       inv[4] = { -1, -1, -1, -1 };          // todas invertidas (tu robot iba al reves)
const char* wName[4] = { "FL", "FR", "BL", "BR" };

void wheel(int i, int v) {
  v = constrain(v * inv[i], -255, 255);
  if (abs(v) < DEADBAND) { ledcWrite(wP[i], 0); ledcWrite(wN[i], 0); return; }
  int pwm = map(abs(v), DEADBAND, 255, MIN_PWM, 255);
  if (v > 0) { ledcWrite(wP[i], pwm); ledcWrite(wN[i], 0); }
  else       { ledcWrite(wP[i], 0);   ledcWrite(wN[i], pwm); }
}
void stopAll() { for (int i = 0; i < 4; i++) wheel(i, 0); }

// avance recto + giro (turn>0 gira hacia un lado, turn<0 al otro)
void drive(int fwd, int turn) {
  int left  = fwd + turn;
  int right = fwd - turn;
  wheel(0, left);  wheel(2, left);     // FL, BL
  wheel(1, right); wheel(3, right);    // FR, BR
}

// ===== UART camara: "found,x,area\n" ========================================
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

  while (digitalRead(PUSH)) { led(60, 40, 0); delay(150); led(0, 0, 0); delay(150); }  // espera START
  led(0, 0, 0);
}

// ===== LOOP =================================================================
void loop() {
  if (TEST_MODE) { motorTest(); return; }

  readCam();
  bool ballValid = bFound && (millis() - lastBall < BALL_TIMEOUT_MS);

  if (ballValid) {
    // --- PERSEGUIR ---
    int err = bX - CENTER_X;                 // + = pelota a la derecha
    lastDir = (err >= 0) ? +1 : -1;          // recuerda hacia donde se va

    int turn = (int)(KP_TURN * err);
    turn = constrain(turn, -TURN_MAX, TURN_MAX);

    // avance proporcional al centrado: descentrada -> casi 0; centrada -> FWD_SPEED
    float align = 1.0f - constrain(fabsf((float)err) / ALIGN_PX, 0.0f, 1.0f);
    int fwd = (int)(FWD_SPEED * align);

    drive(fwd, turn);

    if (abs(err) < CENTERED_PX) led(0, 150, 0);    // verde fuerte = centrada y avanzando
    else                        led(0, 60, 0);     // verde tenue = girando para encarar
  } else {
    // --- BUSCAR ---
    unsigned long lost = millis() - lastBall;
    if (lost < SWEEP_AFTER_MS) {
      drive(0, SEARCH_SPIN * lastDir);             // gira hacia donde la vio por ultima vez
      led(60, 25, 0);                              // naranja = buscando cerca
    } else {
      drive(0, SEARCH_SPIN * lastDir);             // sigue girando = barrido 360 lento
      led(80, 0, 60);                              // morado = barriendo todo el entorno
    }
  }
}
