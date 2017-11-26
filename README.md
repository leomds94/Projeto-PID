![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/Esquematico.png)

# Controlado a temperatura com sensor LM35, cooler de pc com 4 fios e um sistema de controle PID.

## **1.	INTRODUÇÃO:**
<p>
	Ola pessoal, neste artigo iremos mostrar como projetar um sistema de regulação de temperatura, através de um sensor de temperatura LM35, um cooler de quatro fios e um arduino com um controle PID. Esse projeto se propõe a estabilizar a temperatura em um valor pré definido. Se a temperatura variar, ele tentará se estabilizar através de um controle. Este projeto irá lhe ensinar a trabalhar com um cooler 4 fios, um sensor LM35 e projetar um controle proporcional simples. 
</p>
Pra fazer esse controle, utilizaremos:

-	Um arduino, como controlador
-	Uma protoboard 
-	um sensor LM35, como sensor 
-	um cooler de 4 fios, como atuador 
-	um resistor de 10k ohms e um de 1k ohms 
-	fios jumper 

## **3.	 IMPLEMENTAÇÃO:**

Como o nosso controlador é composto por várias etapas, iremos explicar cada etapa separadamente, para depois juntamos tudo. Primeiro iremos explicar como pegamos a temperatura, em seguida como controlamos a velocidade do motor, depois medimos sua velocidade, e por fim, aplicamos o controle PID.

### **1ª etapa: obtendo a temperatura.**

![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/Esquematico2.png)
 <p>
	Essa etapa consiste em pegar o valor analogica do sensor e converter para graus celsius. Para isso, você deverá achar uma função de transferência que faça essa conversão.Para fazer a medição nós usamos três valores de referência  temperatura de congelamento da água (0°) temperatura de evaporação da água (100°) e a temperatura ambiente (que no momento era 31° mas pode variar). Com esse dados obtivemos a função de transferência, através de um processo matemático chamado regressão.  
</p>
<p>	
	A Função de transferência obtida: 0,7363 x - 13,7691 
</p>	

```ino
int sensor_pin = A0;

void setup(){
    pinMode(sensor_pin, INPUT);
}

void loop(){
  int leitura = analogRead(sensor_pin); 
  float temperatura = 0,7363 x - 13,7691;
    Serial.print(‘V :  ’);
  Serial.println(leitura);
  Serial.print(‘C° : ‘);
  Serial.println(temperatura);
   
  delay(1000);
}
```

### **2ª etapa:  Controlamos a velocidade do motor**

![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/esquematico3.png)

Essa etapa é bastante simples. O próprio cooler tem um circuito interno que permite você regular a temperatura. Para isso basta você conectar o fio azul na saída de uma porta com pwm. Assim, dependendo do valor que você colocar no pwm ele irá rodar mais rápido ou mais devagar.
Para ajudar a ver os efeitos, adicionamos um código que nos permite digitar o valor que queremos para o pwm através do terminal. 

# 

```ino
int sensor_pin = A0;
int pwm = 11;

void setup(){
    pinMode(sensor_pin, INPUT);
    Serial.begin(9600);
    pinMode(pwm, OUTPUT);
}

void loop(){
  int leitura = analogRead(sensor_pin);
  float temperatura = 0,7363 x - 13,7691;
    Serial.print(‘V :  ’);
  Serial.println(leitura);
  Serial.print(‘C° : ‘);
  Serial.println(temperatura);

  if (Serial.available() > 0){
    String recebido = leStringSerial();
    valor = recebido.toFloat();
    Serial.println(valor);
  }
  analogWrite(pwm,valor);
   
  delay(1000);
}
```

### **3ª etapa: Medindo o número de rotações por minuto (RPM) do cooler.**

Essa etapa é um pouco chatinha, mas, mais pelo código, porque a estrutura física é muito simples. Basta seguir o esquema abaixo. 

O sensor que nos permite medir o rpm do cooler funciona da seguinte forma: toda vez que uma das hélices passa pelo sensor magnético dentro do cooler ele manda um sinal para a porta digital. Simples não ! O complicado está em medir quantos sinais foram mandado para porta em um determinado espaço de tempo para definir o valor do rpm. 

```ino
int NbTopsFan; 
int Calc;
int hallsensor = 2; 

int sensor_pin = A0;
int pwm = 11;

void rpm (){ 
NbTopsFan++; 
}

void setup(){
    pinMode(sensor_pin, INPUT);
    Serial.begin(9600);
    pinMode(pwm, OUTPUT);
    attachInterrupt(0, rpm, RISING); 
}

void loop(){
NbTopsFan = 0;

  int leitura = analogRead(sensor_pin);
  float temperatura = 0,7363 x - 13,7691;
    Serial.print(‘V :  ’);
  Serial.println(leitura);
  Serial.print(‘C° : ‘);
  Serial.println(temperatura);

  if (Serial.available() > 0){
    String recebido = leStringSerial();
    valor = recebido.toFloat();
    Serial.println(valor);
  }
  analogWrite(pwm,valor);
   
  Calc = ((NbTopsFan * 60)/2);
  Serial.print (Calc, DEC);
  Serial.print (" rpm\r\n"); 

  delay(1000);
}
```

### **4ª etapa: Controle PID**

Agora vamos montar o controle PID. Esse controle irá permitir regular a temperatura. Mas para facilitar vamos primeiro testar o controle P depois o I depois o D e por fim o controle PID. O que é bastante simples, basta ir acrescentando valores às variáveis kp, ki, kd.

```ino
int NbTopsFan; 
int Calc;
int hallsensor = 2; 

int sensor_pin = A0;
int pwm = 11;

int kp  = 4.2;
float ki = 0.12;
int kd = 11.3;
int setPoint = 25;
int erro; 
int i = 0;
int atual = 0;

void rpm (){ 
NbTopsFan++; 
}

void setup(){
    pinMode(sensor_pin, INPUT);
    Serial.begin(9600);
    pinMode(pwm, OUTPUT);
    attachInterrupt(0, rpm, RISING); 
}

void loop(){
NbTopsFan = 0;

  int leitura = analogRead(sensor_pin);
  float temperatura = 0,7363*leitura  - 13,7691;
    Serial.print(‘V :  ’);
  Serial.println(leitura);
  Serial.print(‘C° : ‘);
  Serial.println(temperatura);

  int atual = temperatura;
  int last = atual;
  int erro = setPoint - atual;
  
  int p = erro * kp;
  i += erro * ki;
  int d = (atual - last) * kd;

  int pid = 100 - (p + i + d);
  if(pid >= 255)
    pid = 255;
  else if(pid <= 0)
    pid = 0;

/*
  if (Serial.available() > 0){
    String recebido = leStringSerial();
    valor = recebido.toFloat();
    Serial.println(valor);
  }
*/

  analogWrite(pwm,pid);
   
  Calc = ((NbTopsFan * 60)/2);
  Serial.print (Calc, DEC);
  Serial.print (" rpm\r\n"); 

  delay(1000);
}
```


## **4. MELHORIAS

### **4.1 SAMPLE TIME

Tanto o cálculo integral quanto o derivativo são diretamente influenciado pelo tempo. Logo intervalos de tempos irregulares pode gerar resultados irregulares. Assim, se o PID for chamado em intervalos constantes de tempos, os resultados serão bem mais consistente. 

Após a implementação de uma amostra de tempo, pode-se perceber uma diferença muito significativa, principalmente no derivativo. Antes o derivativo parecia descontrolado. Agora ele varia com mais estabilidade.

```ino
...
int SampleTime = 1000; //1 sec
void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      double error = Setpoint - Input;
      errSum += error;
      double dErr = (error - lastErr);
      
      ...
      ...
   }
}
```

```ino
void SetTunings(double Kp, double Ki, double Kd)
{
  double SampleTimeInSec = ((double)SampleTime)/1000;
   kp = Kp;
   ki = Ki * SampleTimeInSec;
   kd = Kd / SampleTimeInSec; 
}
``` 
 
```ino 
void SetSampleTime(int NewSampleTime)
{
   if (NewSampleTime > 0)
   {
      double ratio  = (double)NewSampleTime / (double)SampleTime;
      ki *= ratio;
      kd /= ratio;
      SampleTime = (unsigned long)NewSampleTime;
   }
}
```
### **4.2 DERIVATIVE KICK

Quando há uma mudança no setpoint uma alteração brusca no erro acontece. Isso ocorre porque a derivada dessa mudança gera um número muito grande.  Felizmente a solução é bastante simples. 

Mas no nosso caso essa alteração não fez muita diferença, pois o efeito do derivativo não surte muito efeito, pois não a mudança brusca de temperatura. 

```ino

double errSum, lastInput;
double kp, ki, kd;
int SampleTime = 1000; //1 sec
void Compute()
{
   unsigned long now = millis();
   int timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      ...
      ...
      double dInput = (Input - lastInput);
 
      /*Compute PID Output*/
      Output = kp * error + ki * errSum - kd * dInput;
 
      /*Remember some variables for next time*/
      lastInput = Input;
      ...
      ...
   }
}
```

### **4.3 ON-THE-FLY TUNING CHANGES

A melhoria 'On-The-Fly Tunning Changes' permite ao usuário modificar os valores de Kp, Ki e Kd em tempo de execução.

Isto evita que seja necessário parar o sistema para modificar os valores das constantes assim podendo gerar um superaquecimento sem uma ação imediata do controlador PID para aumentar o RPM.

A implementação desta funcionalidade é simples como mostrada a seguir:

...
void SetTunings(double Kp, double Ki, double Kd){
 double SampleTimeInSec = ((double)SampleTime)/1000;
 kp = Kp;
 ki = Ki * SampleTimeInSec;
 kd = Kd / SampleTimeInSec;
}
...

### **4.4 RESET WINDUP MITIGATION

A melhoria de Reset Windup cria limites, de máximo e mínimo, para o cálculo do ki e para a saída.

A correção da soma do erro ki (errSum) é substituída para ITerm += (ki * error); e esse ITerm tem que se enquadrar a uma faixa entre o outMin e o outMax.
O cálculo da saída é bem parecido ao ITerm porque também tem que se enquadrar a uma faixa entre o outMin e o outMax.

Os valores de outMin e outMax são calculados na função a seguir:

```
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
```

De acordo com os testes feitos, os valores atribuídos são:

	outMin = 0;
	outMax = 255;
	
### **4.5 ON/OFF (AUTO/MANUAL)

### **4.6 INITIALIZATION

A melhoria de Initialization é utilizada quando o controlador inicializa sem carga ou retorna ao seu estado de automático. Nas duas ocasiões, o controlador pode imprimir um valor de saída muito indesejado, como algo bem distante do SetPoint. 

A programação executa a função “initialize()” quando o controlador inicializa o modo “automático” e parte com os valores como se ele estivesse sempre nesse modo.

```
bool inAuto = false; 
#define MANUAL 0
#define AUTOMATIC 1

...

void SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if(newAuto && !inAuto)
	{  /*we just went from manual to auto*/
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
```

### **4.7 CONTROLLER DIRECTION


## **5.    CONCLUSÃO**
 
### **5.1.        APLICAÇÕES UTILIZANDO ESTE PROJETO**
 
Esse projeto pode ser utilizado para o controle de temperatura em qualquer ambiente, desde uma pequena caixa (quadro elétrico, aquário,...) como também uma subestação, utilizando um ventilador maior ou um exaustor para diminuir a temperatura do local.
Alguns quadros elétricos possuem equipamentos que liberam muito calor, podendo ocasionar algum incêndio. Com isso, o nosso produto pode ser instalado nesses tipos de quadro para manter sempre uma temperatura agradável. A figura abaixo mostra um quadro com um ventilador instalado para essa finalidade.

![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/Painel.jpg)
 
Outra aplicação com esse projeto seria a instalação em mini-racks, com isso, a temperatura interna seria controlada, diminuindo o custo com energia.
 
![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/rack.jpg)

Também é uma solução para quem possui aquários e deseja controlar a temperatura da água. Existem alguns tipos de peixes que não toleram variações grandes e por isso devem ser criados com a máxima estabilidade. 

 
### **5.2.        MELHORIAS**
 
Esse produto pode ser melhorado adicionando alguns equipamentos e aumentando as funcionalidades e finalidades.
 
Exemplo 01: Variação de temperatura.
Caso o usuário queira variar a temperatura desejada, pode-se colocar um display com botões. Com isso, esse produto poderia ser utilizado tanto em locais mais frios como mais quentes, já que há uma facilidade maior de alterar a temperatura desejada. Na figura abaixo, é mostrado um modelo já comercializado de como isso é possível.

![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/Variador.jpg)  
 
Exemplo 02: Redução do controlador.
Uma melhoria significativa do produto seria na troca do controlador. As funcionalidades e o programa embarcado são muito simples para esse projeto, com isso, um microcontrolador básico com menos memória, quantidade de pinos digitais e analógicos reduziria o custo. Além disso, o tamanho físico de uma placa eletrônica seria bem mais compacta, já que seria confeccionada com menos quantidade de trilhas e componentes, e integrando um microcontrolador PIC ou um ARM no lugar de um Arduino completo.
 
### **5.3.        CUSTOS DO PRODUTO**
 
A seguir, é mostrado o preço dos materiais mais significativos do projeto. Esses valores foram baseados nos sites www.mercadolivre.com.br e www.autocore.com.br.
 
![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/preços.jpg)  
 
 
### **5.4.        VALOR DO PRODUTO**
 
Baseado no custo do item anterior, e colocando uma margem de lucro de 50%, o valor para ser comercializado seria de R$ 110,00.
As aplicações já listadas do nosso projeto (controle de temperatura em quadros elétricos, mini-racks e aquários) já existem em alguns produtos no mercado. Para se ter uma comparação, o preço de um AquaCool Master Fan  que serve para ser instalado em um aquário, custa R$ 224,00. Um kit básico para rack com 2 ventiladores custa R$ 134,00 com o frete.
 
![Alt Text](https://github.com/leomds94/Projeto-PID/raw/master/imagens/aquaCool.jpg)  

## **5.5.        REFERÊNCIAS DE MATERIAIS**

Arduino Uno Rev3
https://www.autocorerobotica.com.br/arduino-uno 

Sensor de Temperatura
https://www.autocorerobotica.com.br/sensor-de-temperatura-lm35 

Cooler
https://produto.mercadolivre.com.br/MLB-721118480-cooler-intel-_JM?source=gps 

Fonte 12Vcc
https://produto.mercadolivre.com.br/MLB-863436791-fonte-de-alimentaco-12v-25a-bivolt-110220v-_JM 

AquaCool Master Fan
https://produto.mercadolivre.com.br/MLB-834555092-ventilador-para-aquario-c-controle-de-temperatura-aquacool-_JM

Kit de ventilação para rack
https://produto.mercadolivre.com.br/MLB-706060609-kit-de-ventilaco-para-rack-2-ventiladores-padro-univ-19-_JM
