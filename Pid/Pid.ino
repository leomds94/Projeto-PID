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

unsigned long lastTime;
double Input, Output, Setpoint, errSum, lastInput;
double kp, ki, kd;
int SampleTime = 1000;

double ITerm, outMax, outMin;
bool inAuto = false;

#define MANUAL 0
#define AUTOMATIC 1

int i = 0;
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

void Compute(){
 if(!inAuto) return;
 unsigned long now = millis();
 int timeChange = (now - lastTime);
 if(timeChange>=SampleTime){
  double error = Setpoint - Input;
  double outTranslate = map(temp, 18, 50, 0, 255);
  ITerm += (ki * error);
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
  double dInput = (Input - lastInput);
  Output = kp * error + ITerm - kd * dInput;
  if (Output > outMax) Output = outMax;
  else if (Output < outMin) Output = outMin;
  lastInput = Input;
  lastTime = now;
  sprintf(buf, "Erro: %d", error);
  Serial.println(buf);
 }
}

void SetOutputLimits(double Min, double Max)
{
 if(Min > Max) return;
 outMin = Min;
 outMax = Max;

 if(Output > outMax) Output = outMax;
 else if(Output < outMin) Output = outMin;
 if(ITerm> outMax) ITerm= outMax;
 else if(ITerm< outMin) ITerm= outMin;
}

// Modificar kp, ki e kd manualmente
void SetTunings(double Kp, double Ki, double Kd){
 double SampleTimeInSec = ((double)SampleTime)/1000;
 kp = Kp;
 ki = Ki * SampleTimeInSec;
 kd = Kd / SampleTimeInSec;

}

// Tempo de leitura de amostra
void SetSampleTime(int NewSampleTime) {
  if (NewSampleTime > 0) {
    double ratio = (double)NewSampleTime / (double)SampleTime;
    ki *= ratio;
    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

// Ajustar manual e automático
void SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto && !inAuto)
    {  /*we just went from manual to auto*/
        Initialize();
    }
    inAuto = newAuto;
}
 
void Initialize()
{
   lastInput = Input;
   ITerm = Output;
   if(ITerm> outMax) ITerm= outMax;
   else if(ITerm< outMin) ITerm= outMin;
}

void setup() {
  Serial.begin(9600);

  pinMode(coolerState, INPUT);
  digitalWrite(coolerState, HIGH);
  pinMode(coolerPwm, INPUT);
  lcd.begin(16, 2);
  getRpm();
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

  // Regra de três entre temperatura e RPM estimado
  Input = map(temp, 18, 50, 0, 255);

  // Realizando o PID
  Compute();

  //Saída no RPM com PID ajustado
  rpmValue = temp * (Output / 100);


  // Mostrando no serial port os dados PID e Erro
  sprintf(buf, "P: %d", kp);
  Serial.println(buf);
  char str_i[10];
  sprintf(buf, "I: %s", str_i);
  Serial.println(buf);
  sprintf(buf, "D: %d", kd);
  Serial.println(buf);
  sprintf(buf, "PID: %d", Output);
  Serial.println(buf);

  delay (1000);

  // Mostrando pwm que regula a velocidade do cooler
  sprintf(buf, "PWM: %d", value);
  rpmValue = value * 8;
  Serial.println(buf);

  analogWrite(coolerPwm, value);
}
