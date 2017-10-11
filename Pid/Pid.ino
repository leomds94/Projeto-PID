#include <string.h>
#include <LiquidCrystal.h>

int NbTopsFan;
int Calc;
int coolerState = 7;
int coolerPwm = 9;
int tempsensor = A0;

int value = 0;
unsigned long pulseDuration = 0;
double frequency = 0;
int rpmValue = 0;

int kp  = 1.5;
float ki = 0.2;
int kd = 0;
int setPoint = 25;
int erro;
int i = 0;
int last = 0;
float temp = 0;
char buf[20];

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

// Pegando tempo do pulso da porta de controle do cooler
void getRpm () {
  pulseDuration = pulseIn(coolerState, LOW);
  frequency = 1000000 / pulseDuration;
  rpmValue = frequency / 2 * 60;
 // rpmValue = temp*30;
}

void setup() {
  Serial.begin(9600);

  pinMode(coolerState, INPUT);
  digitalWrite(coolerState, HIGH);
  pinMode(coolerPwm, INPUT);
  lcd.begin(16, 2);
  getRpm();
  last = setPoint;
}

void loop () {
  // Transformar temperatura analógica em celsius
  temp = (float(analogRead(tempsensor)) * 5 / (1023)) / 0.01;

  // Mostrar temperatura no display LCD
  lcd.setCursor(0, 0);
  char str_temp[10];
  dtostrf(temp, 4, 2, str_temp);
  sprintf(buf, "Temp: %s", str_temp);
  Serial.println(buf);
  lcd.print(buf);

  // Mostrar RPM no LCD
  lcd.setCursor(0, 1);
  sprintf(buf, "%d rpm", rpmValue);
  Serial.println(buf);
  lcd.print(buf);


  // Pegando erro da temperatura
  int atual = temp;
  int erro = abs(last - atual);
  last = atual;

  // Regra de três entre temperatura e RPM estimado
  value = map(temp, 18, 50, 0, 255);

  // Realizando o PID
  int  p = erro * kp;
  i += erro * ki;
  int d = (atual - last) * kd;

  int  pid = value - (p + i + d);

  //Realizando pid no RPM
  rpmValue = temp*(pid/100);


  // Mostrando no serial port os dados PID e Erro
  sprintf(buf, "Erro: %d", erro);
  Serial.println(buf);
  sprintf(buf, "P: %d", p);
  Serial.println(buf);
  char str_i[10];
  dtostrf(i, 4, 2, str_i);
  sprintf(buf, "I: %s", str_i);
  Serial.println(buf);
  sprintf(buf, "D: %d", d);
  Serial.println(buf);
  sprintf(buf, "PID: %d", pid);
  Serial.println(buf);

  delay (1000);

  // Mostrando pwm que regula a velocidade do cooler
  sprintf(buf, "PWM: %d", value);
  rpmValue = value*8;
  Serial.println(buf);

  analogWrite(coolerPwm, value);
}
