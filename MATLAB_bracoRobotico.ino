/*
* Braço Robotico controlado pelo joytick/ MATLAB
* Mariana Pedroso Naves
*/

// Bibliotecas
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Criar objeto para o driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ********** Definições dos parametros do servo **********
// Posição servo na placa
#define SERVO_GARRA 12
#define SERVO_BASE 13
#define SERVO_LATERAL1 14
#define SERVO_LATERAL2 15

// Posicao inicial
#define INICIAL_GARRA 0
#define INICIAL_BASE 0
#define INICIAL_LATERAL1 90
#define INICIAL_LATERAL2 90

// Angulo minimo
#define MIN_GARRA 10
#define MIN_BASE 0
#define MIN_LATERAL1 30
#define MIN_LATERAL2 30

// Angulo maximo
#define MAX_GARRA 95
#define MAX_BASE 170
#define MAX_LATERAL1 150
#define MAX_LATERAL2 137

//Leitura analogica
#define ANALOGICO_GARRA_MIN 30
#define ANALOGICO_GARRA_MAX 1023

#define ANALOGICO_BASE_MIN 125
#define ANALOGICO_BASE_MAX 913

#define ANALOGICO_LATERAL1_MAX 900
#define ANALOGICO_LATERAL1_MIN 64

#define ANALOGICO_LATERAL2_MAX 900
#define ANALOGICO_LATERAL2_MIN 64

// Amostra para o vetor
#define AMOSTRAS 10

// Constantes
int ang_garra = INICIAL_GARRA;
int ang_base = INICIAL_BASE;
int ang_lateral1 = INICIAL_LATERAL1;
int ang_lateral2 = INICIAL_LATERAL2;

// Variaveis para armazenar leitura filtrada
float leitura_filtrada_garra = 0;
float leitura_filtrada_base = 0;
float leitura_filtrada_lateral1 = 0;
float leitura_filtrada_lateral2 = 0;

// Variáveis para o incremento
int increm_garra = 0;
int increm_base = 0;
int increm_lateral1 = 0;
int increm_lateral2 = 0;

// Saltos do servo
const int passo[12] = { 20, 15, 10, 5, 1, 0, -1, -5, -10, -15, -20, 0 };

// Conversão do angulo analogico para pwm
int angleToPulse(int angle) {
  int pulseMin = 150;
  int pulseMax = 575;
  return map(angle, 0, 180, pulseMin, pulseMax);
}

// ****************  Comunicação serial - MATLAB ****************
String serial2motor(String nome_mtr, String str_serial) {
  int indice = 0;
  String mtrxx;
  indice = str_serial.indexOf(nome_mtr);
  if (indice == -1) {
    mtrxx = "9999";
  } else {
    mtrxx.concat(str_serial[indice + 6]);
    mtrxx.concat(str_serial[indice + 7]);
    mtrxx.concat(str_serial[indice + 8]);
    mtrxx.concat(str_serial[indice + 9]);
  }
  return mtrxx;
}

void setup() {
  pwm.begin();         // Inicializando a placa
  pwm.setPWMFreq(60);  // Frequência recomendada para servos

  // Inicializando as leituras dos pinos analogicos
  leitura_filtrada_garra = analogRead(A0);
  leitura_filtrada_base = analogRead(A1);
  leitura_filtrada_lateral1 = analogRead(A2);
  leitura_filtrada_lateral2 = analogRead(A3);

  pwm.setPWM(SERVO_GARRA, 0, angleToPulse(ang_garra));
  pwm.setPWM(SERVO_BASE, 0, angleToPulse(ang_base));
  pwm.setPWM(SERVO_LATERAL1, 0, angleToPulse(ang_lateral1));
  pwm.setPWM(SERVO_LATERAL2, 0, angleToPulse(ang_lateral2));

  Serial.begin(9600);
}


void loop() {
  String comandoMotor;
  String mtr;

  // Media movel - remover as trepidações
  for (int i = 0; i < AMOSTRAS; i++) {
    increm_garra += analogRead(A0);  // Pino analógico da garra
    increm_base += analogRead(A1);   // Pino analógico da base
    increm_lateral1 += analogRead(A2);
    increm_lateral2 += analogRead(A3);
    delay(5);
  }

  // Tirar a média
  increm_garra = increm_garra / AMOSTRAS;
  increm_base = increm_base / AMOSTRAS;
  increm_lateral1 = increm_lateral1 / AMOSTRAS;
  increm_lateral2 = increm_lateral2 / AMOSTRAS;

  // Filtro para o servo retornar ao angulo minimo
  leitura_filtrada_garra = (0.6 * leitura_filtrada_garra) + (0.4 * increm_garra);
  increm_garra = constrain(map(increm_garra, ANALOGICO_GARRA_MIN, ANALOGICO_GARRA_MAX, 0, 11), 0, 11);

  leitura_filtrada_base = (0.6 * leitura_filtrada_base) + (0.4 * increm_base);
  increm_base = constrain(map(leitura_filtrada_base, ANALOGICO_BASE_MIN, ANALOGICO_BASE_MAX, 0, 11), 0, 11);

  leitura_filtrada_lateral1 = (0.6 * leitura_filtrada_lateral1) + (0.4 * increm_lateral1);
  increm_lateral1 = constrain(map(leitura_filtrada_lateral1, ANALOGICO_LATERAL1_MIN, ANALOGICO_LATERAL1_MAX, 0, 11), 0, 11);

  leitura_filtrada_lateral2 = (0.6 * leitura_filtrada_lateral2) + (0.4 * increm_lateral2);
  increm_lateral2 = constrain(map(leitura_filtrada_lateral2, ANALOGICO_LATERAL2_MIN, ANALOGICO_LATERAL2_MAX, 0, 11), 0, 11);

  // Incremento mediante ao passo
  ang_garra = ang_garra + passo[increm_garra];
  ang_base = ang_base + passo[increm_base];
  ang_lateral1 = ang_lateral1 + passo[increm_lateral1];
  ang_lateral2 = ang_lateral2 + passo[increm_lateral2];

  // Limita o angulo do servo
  ang_garra = constrain(ang_garra, MIN_GARRA, MAX_GARRA);
  ang_base = constrain(ang_base, MIN_BASE, MAX_BASE);
  ang_lateral1 = constrain(ang_lateral1, MIN_LATERAL1, MAX_LATERAL1);
  ang_lateral2 = constrain(ang_lateral2, MIN_LATERAL2, MAX_LATERAL2);

  delay(10);
  // Debug
  //Serial.print("Angulo lateral2: ");
  //Serial.println(ang_lateral2);

  if (Serial.available() > 0) {
    delay(5);
    comandoMotor = Serial.readString();

    mtr = serial2motor("mtr00 ", comandoMotor);
    Serial.print("Base: ");
    Serial.println(mtr);
    pwm.setPWM(SERVO_BASE, 0, angleToPulse(mtr.toFloat() / 10));

    mtr = serial2motor("mtr01 ", comandoMotor);
    Serial.print("Garra: ");
    Serial.println(mtr);
    pwm.setPWM(SERVO_GARRA, 0, angleToPulse(mtr.toFloat() / 10));

    mtr = serial2motor("mtr02 ", comandoMotor);
    Serial.print("Lateral1: ");
    Serial.println(mtr);
    pwm.setPWM(SERVO_LATERAL1, 0, angleToPulse(mtr.toFloat() / 10));

    mtr = serial2motor("mtr03 ", comandoMotor);
    Serial.print("Lateral2: ");
    Serial.println(mtr);
    pwm.setPWM(SERVO_LATERAL2, 0, angleToPulse(mtr.toFloat() / 10));
  }
}
