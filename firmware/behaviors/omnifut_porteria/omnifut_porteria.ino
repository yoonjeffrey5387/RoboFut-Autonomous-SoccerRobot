#include <Adafruit_NeoPixel.h>

#define ATTACK_BLUE 1          // gol ENEMIGO = azul
#define DEBUG 0

// ===== AJUSTE CRUCIAL =====
#define TARGET_X 78            // ¡CORREGIDO! El centro de 160 píxeles es ~80. Ajusta entre 75 y 85 si el diseño varía.
#define PUSH_Y 45              // pelota mas abajo de esto = cerca -> empujar al gol
#define TURN_TRIM -8           // ¡CORREGIDO! Empecemos casi en 0. Si se va a la derecha pon -4, si se va a la izquierda pon +4.

// ===== PINES =====
#define PUSH_BTN 45
#define LEDPIN 3
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
#define PWM_RES 8
#define DEADBAND 3

// ===== VELOCIDADES =====
const int MIN_MOVE=45, CRUISE=58, GOLPE=92, SEARCH_SPIN=42;
const unsigned long SEARCH_ON_MS=120, SEARCH_OFF_MS=300;

// ===== CONTROL (PURA VISION) =====
const float KP_TURN=0.32f, KD_TURN=0.70f;
const int   TURN_CAP=38, BALL_DB=10;
const float KP_GOAL=0.30f;
const int   GOAL_DB=9, CARRY_TURN=32;
const int   BALL_MIN_AREA=20;
const unsigned long BALL_TIMEOUT_MS=250, GRACE_MS=450;

Adafruit_NeoPixel pixel(1, LEDPIN, NEO_GRB + NEO_KHZ800);
void led(uint8_t r,uint8_t g,uint8_t b){ pixel.setPixelColor(0,pixel.Color(r,g,b)); pixel.show(); }

const int wP[4]={FL_P,FR_P,BL_P,BR_P}, wN[4]={FL_N,FR_N,BL_N,BR_N};
int inv[4]={-1,-1,-1,-1};
void wheel(int i,int v){
  v=constrain(v*inv[i],-255,255); int a=abs(v);
  if(a<DEADBAND){ ledcWrite(wP[i],0); ledcWrite(wN[i],0); return; }
  int pwm=constrain(a,MIN_MOVE,255);
  if(v>0){ ledcWrite(wP[i],pwm); ledcWrite(wN[i],0); } else { ledcWrite(wP[i],0); ledcWrite(wN[i],pwm); }
}
void stopAll(){ for(int i=0;i<4;i++) wheel(i,0); }
void drive(int fwd,int turn){ int l=fwd+turn,r=fwd-turn,m=max(abs(l),abs(r)); if(m>255){ l=l*255/m; r=r*255/m; } wheel(0,l); wheel(2,l); wheel(1,r); wheel(3,r); }

// ===== UART: bf,bx,by,ba,blf,blx,ylf,ylx =====
char buf[64]; uint8_t blen=0;
int bFound=0,bX=0,bY=0,bArea=0,blueF=0,blueX=0,yellowF=0,yellowX=0;
unsigned long lastBall=0; bool newCam=false;
void readCam(){
  while(Serial1.available()){
    char c=Serial1.read();
    if(c=='\n'){ buf[blen]='\0'; int a1,a2,a3,a4,a5,a6,a7,a8;
      if(sscanf(buf,"%d,%d,%d,%d,%d,%d,%d,%d",&a1,&a2,&a3,&a4,&a5,&a6,&a7,&a8)==8){ bFound=a1;bX=a2;bY=a3;bArea=a4;blueF=a5;blueX=a6;yellowF=a7;yellowX=a8; if(bFound) lastBall=millis(); newCam=true; }
      blen=0;
    } else if(blen<sizeof(buf)-1) buf[blen++]=c; else blen=0;
  }
}

void setup(){
  Serial.begin(115200); Serial1.setRxBufferSize(512); Serial1.begin(115200,SERIAL_8N1,CAM_RX,CAM_TX);
  pinMode(PUSH_BTN,INPUT_PULLUP);
  for(int i=0;i<4;i++){ ledcAttach(wP[i],PWM_FREQ,PWM_RES); ledcAttach(wN[i],PWM_FREQ,PWM_RES); }
  stopAll(); pixel.begin();
  while(digitalRead(PUSH_BTN)){ led(60,40,0); delay(150); led(0,0,0); delay(150); }
  for(int k=0;k<3;k++){ led(0,150,0); delay(120); led(0,0,0); delay(120); }
}

float errPrev=0,dErrF=0;
int lastTurnDir=+1,lastGoalDir=+1,lastBallX=TARGET_X;
unsigned long lastBallSeen=0,searchStart=0; bool searchSpinning=true;

void loop(){
  readCam();
  bool ballValid = bFound && (millis()-lastBall<BALL_TIMEOUT_MS) && (bArea>BALL_MIN_AREA);
  if(ballValid){ lastBallSeen=millis(); lastBallX=bX; }

  int enF=ATTACK_BLUE?blueF:yellowF, enX=ATTACK_BLUE?blueX:yellowX;
  int fwd=0, turn=0;

  if(ballValid){
    
    // ===== MODIFICADO: EVITAR AUTOGOL (PORTERIA AMARILLA EN VISION) =====
    // Si ve la portería propia, va adelante pero abriéndose en arco para rodear la pelota
    if(yellowF && bY < PUSH_Y){
      fwd = CRUISE;                                        // Va hacia adelante
      turn = (yellowX > TARGET_X) ? -TURN_CAP : TURN_CAP;   // Gira fuerte al lado opuesto para rodearla
      led(255, 0, 0);                                      // Led Rojo
    }
    
    else if(bY < PUSH_Y){
      // ===== IR DERECHO POR LA PELOTA =====
      int e=bX-TARGET_X;
      if(newCam){ float d=e-errPrev; errPrev=e; dErrF+=0.4f*(d-dErrF); }
      if(abs(e)<BALL_DB){ turn=TURN_TRIM; }                              
      else { turn=constrain((int)(KP_TURN*e + KD_TURN*dErrF),-TURN_CAP,TURN_CAP); lastTurnDir=(e>0)?+1:-1; }
      int ae=abs(e); fwd=CRUISE-(int)((CRUISE*0.5f)*(min(ae,70)/70.0f));
      led(0,90,0); // Led Verde Claro
    } 
    
    else {
      // ===== EMPUJAR AL GOL AZUL =====
      if(enF){
        int eg=enX-TARGET_X; lastGoalDir=(eg<=0)?-1:+1;
        turn=(abs(eg)<GOAL_DB)?TURN_TRIM:constrain((int)(KP_GOAL*eg),-CARRY_TURN,CARRY_TURN);
        fwd=GOLPE; led(0,200,0); // Led Verde Fuerte (Acelera directo al gol)                                            
      } 
      else {
        // ===== SOLUCIÓN AL TEMBLOR =====
        int e=bX-TARGET_X;
        turn = (CARRY_TURN / 2) * lastGoalDir + (int)(KP_TURN * e); 
        fwd = MIN_MOVE; // Avanza suave para no escupir la pelota de la pala
        led(0,0,140); // Led Azul            
      }
    }
  }
  else if(lastBallSeen && millis()-lastBallSeen<GRACE_MS){
    turn=((lastBallX>=TARGET_X)?+1:-1)*(int)(TURN_CAP*0.8f); fwd=CRUISE/2; led(0,90,40);   
  }
  else{
    // ===== MODIFICADO: ELIMINADA FUNCIÓN DE ATACAMIENTO DE PARED CON LED BLANCO =====
    // Búsqueda normal por pulsos (Gira/Para)
    if(millis()-searchStart>(searchSpinning?SEARCH_ON_MS:SEARCH_OFF_MS)){ searchSpinning=!searchSpinning; searchStart=millis(); }
    turn=searchSpinning?(SEARCH_SPIN*lastTurnDir):0; fwd=0; led(searchSpinning?60:15,25,0);  
  }

  drive(fwd,turn);

  if(DEBUG){ static unsigned long t=0; if(millis()-t>500){ t=millis();
    Serial.printf("bf=%d bx=%d by=%d ba=%d | enF=%d enX=%d\n",bFound,bX,bY,bArea,enF,enX); } }
  newCam=false;
}