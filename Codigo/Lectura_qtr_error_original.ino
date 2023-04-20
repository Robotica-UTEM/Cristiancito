#include <QTRSensors.h>
#define AIN1                2
#define PWMA                3
#define AIN2                4
#define PWMB                5
#define STANBY              6
#define BIN2                7
#define BIN1                8
#define S_A                 9
#define S_B                 10
#define BOTON               11
#define Led_Rojo            12
#define Tiempo_Led          100

#define Tiempo_aceleracion_base  10
#define Tiempo_aceleracion_max   3
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
float Kp = 0.18;
float Ki = 0.002;
float Kd = 4.5 ;
//float Kp = 0.48;
//float Ki = 0.0002;
//float Kd = 30;
int P;
int I;
int D;
int error6 = 0;
int error5 = 0;
int error4 = 0;
int error3 = 0;
int error2 = 0;
int error1 = 0;
int error = 0;
int lastError = 0;
long anteriortiempo;
boolean onoff = false;
boolean estado_led = false;

int maxspeed = 50;
const uint8_t minspeed = 0;
int basespeed = 30   ;

int reduccion_velocidad_base=0;
int reduccion_velocidad_max=30 ;

const uint8_t SAVE_MAX = maxspeed;
const uint8_t SAVE_BASE = basespeed;
unsigned long Retardo_aceleracion_base_anterior;
unsigned long Retardo_aceleracion_max_anterior;
unsigned long Bloqueo_inicio_anterior;
unsigned long Comprobacion_anterior = millis();

void setup() {
  pinMode (S_A, INPUT);
  pinMode (S_B, INPUT);
  pinMode (PWMA, OUTPUT);
  pinMode (PWMB, OUTPUT);
  pinMode (AIN1, OUTPUT);
  pinMode (AIN2, OUTPUT);
  pinMode (BIN1, OUTPUT);
  pinMode (BIN2, OUTPUT);
  pinMode (STANBY, OUTPUT);
  pinMode(Led_Rojo, OUTPUT);
  digitalWrite(STANBY, HIGH);
  digitalWrite(Led_Rojo, HIGH);
  Serial.begin(9600);
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]) {
    A7, A6, A5, A4, A3, A2, A1, A0
  }, SensorCount);
  //qtr.setEmitterPin(7);//LEDON PIN
  delay(500);
  for (int i = 0; i < 400; i++) // make the calibration take about 10 seconds
  {
    qtr.calibrate(); // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(Led_Rojo, LOW); // turn off Arduino's LED to indicate we are through with calibration
}
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  if (digitalRead(BOTON) == HIGH) {
    onoff = ! onoff;
    delay(1000);
    anteriortiempo = millis();
    Bloqueo_inicio_anterior = millis();
    estado_led = ! estado_led;
    digitalWrite(Led_Rojo, LOW);
    
  }
  if (onoff == true) {
    PID_control();
    //Sensores_lados();   //sensor pin 9 curvas
    //Sensores_ladosderecha();  //sensor pin 10 curvas
    Intermitencia_Led();
  }
  else {
    velocidad_motores(0, 0);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////
void velocidad_motores(int VmotorA, int VmotorB) {
  //Direccion motor A derecha
  digitalWrite (AIN1, HIGH);
  digitalWrite (AIN2, LOW);
  analogWrite (PWMA, VmotorA); //Velocidad motor A
  //Direccion motor B izquierda
  digitalWrite (BIN1, LOW);
  digitalWrite (BIN2, HIGH);
  analogWrite (PWMB, VmotorB); //Velocidad motor B
}

void PID_control() {
  unsigned int position = qtr.readLineWhite(sensorValues);
  error = 3500   - position;
  //----------------algoritmo PID--------------------------
  P = error;
  D = error - lastError;
  I = error1 + error2 + error3 + error4 + error5 + error6;

  error6 = error5;
  error5 = error4;
  error4 = error3;
  error3 = error2;
  error2 = error1;
  error1 = error;
  lastError = error;
  int motorspeed = P * Kp + I * Ki + D * Kd;
  int motorspeeda = basespeed + motorspeed;
  int motorspeedb = basespeed - motorspeed;

  if (motorspeeda > maxspeed) {
    motorspeeda = maxspeed;
  }
  if (motorspeedb > maxspeed) {
    motorspeedb = maxspeed;
  }
  if (motorspeeda < minspeed) {
    motorspeeda = minspeed;
  }
  if (motorspeedb < minspeed) {
    motorspeedb = minspeed;
  }
  velocidad_motores(motorspeeda, motorspeedb);
}

/*void Sensores_ladosderecha() {
  int B = digitalRead(S_A);
  int A = digitalRead(S_B);
  if (A == 0 && B == 0) {
    Comprobacion_anterior = millis();
    basespeed = SAVE_BASE;
  }
  if (A == 0 && B == 1) {
    unsigned long Comprobacion_actual = millis();
    if (Comprobacion_actual - Comprobacion_anterior > 500) {
      basespeed = 0;
      maxspeed = 50;
      velocidad_motores(0, 0);
      //delay(10);
      Retardo_aceleracion_base_anterior = millis();
      Retardo_aceleracion_max_anterior = millis();
    }
  }
  unsigned long Bloqueo_inicio_actual = millis();
  if (Bloqueo_inicio_actual-Bloqueo_inicio_anterior > 4000){
    if (B == 0 && A == 1) {
     while (true) {
       A = digitalRead(S_A);
       B = digitalRead(S_B);
       if (A == 0 && B == 0) {
         break;
       }
       unsigned long Comprobacion_actual = millis();
       if (Comprobacion_actual - Comprobacion_anterior < 500) {
         break;
       }
       if (basespeed > 0) {
         basespeed -= 1;
       }
       if (maxspeed > 0) {
          maxspeed -= 1;
       }
       PID_control();
       if (digitalRead(BOTON) == HIGH) {
          break;
        }
      }
    }
  }
  if (basespeed < SAVE_BASE) {
    unsigned long Retardo_aceleracion_base_actual = millis();
    if (Retardo_aceleracion_base_actual - Retardo_aceleracion_base_anterior > Tiempo_aceleracion_base) {
      Retardo_aceleracion_base_anterior = Retardo_aceleracion_base_actual;
      basespeed += 1;
    }
  }
  if (maxspeed < SAVE_MAX) {
    unsigned long Retardo_aceleracion_max_actual = millis();
    if (Retardo_aceleracion_max_actual - Retardo_aceleracion_max_anterior > Tiempo_aceleracion_max) {
      Retardo_aceleracion_max_anterior = Retardo_aceleracion_max_actual;
      maxspeed += 1;
    }
  }
}*/

void Sensores_lados() {
  int A = digitalRead(S_A);
  int B = digitalRead(S_B);
  if (A == 0 && B == 0) {
    Comprobacion_anterior = millis();
    basespeed = SAVE_BASE;
  }
  if (A == 0 && B == 1) {
    unsigned long Comprobacion_actual = millis();
    if (Comprobacion_actual - Comprobacion_anterior > 500) {
      basespeed = reduccion_velocidad_base;
      maxspeed = reduccion_velocidad_max;
      velocidad_motores(0, 0);
      //delay(200);
      Retardo_aceleracion_base_anterior = millis();
      Retardo_aceleracion_max_anterior = millis();
    }
  }
  unsigned long Bloqueo_inicio_actual = millis();
  if (Bloqueo_inicio_actual-Bloqueo_inicio_anterior > 5000){
    if (B == 0 && A == 1) {
     while (true) {
       A = digitalRead(S_A);
       B = digitalRead(S_B);
       if (A == 0 && B == 0) {
         break;
       }
       unsigned long Comprobacion_actual = millis();
       if (Comprobacion_actual - Comprobacion_anterior < 1000) {
         break;
       }
       if (basespeed > 0) {
         basespeed -= 1;
       }
       A = digitalRead(S_A);
       B = digitalRead(S_B);
       if (A == 0 && B == 0) {
         break;
       }
       if (maxspeed > 0) {
          maxspeed -= 1;
       }
       A = digitalRead(S_A);
       B = digitalRead(S_B);
       if (A == 0 && B == 0) {
         break;
       }
       PID_control();
       if (digitalRead(BOTON) == HIGH) {
          break;
        }
       A = digitalRead(S_A);
       B = digitalRead(S_B);
       if (A == 0 && B == 0) {
         break;
       }
      }
    }
  }
  if (basespeed < SAVE_BASE) {
    unsigned long Retardo_aceleracion_base_actual = millis();
    if (Retardo_aceleracion_base_actual - Retardo_aceleracion_base_anterior > Tiempo_aceleracion_base) {
      Retardo_aceleracion_base_anterior = Retardo_aceleracion_base_actual;
      basespeed += 1;
    }
  }
  if (maxspeed < SAVE_MAX) {
    unsigned long Retardo_aceleracion_max_actual = millis();
    if (Retardo_aceleracion_max_actual - Retardo_aceleracion_max_anterior > Tiempo_aceleracion_max) {
      Retardo_aceleracion_max_anterior = Retardo_aceleracion_max_actual;
      maxspeed += 1;
    }
  }
}


void Intermitencia_Led() {
  unsigned long actualtiempo = millis();
  if (actualtiempo - anteriortiempo > Tiempo_Led) {
    anteriortiempo = actualtiempo;
    if (estado_led == true) {
      digitalWrite(Led_Rojo, HIGH);
      estado_led = ! estado_led;
    }
    else {
      digitalWrite(Led_Rojo, LOW);
      estado_led = ! estado_led;
    }
  }
}
