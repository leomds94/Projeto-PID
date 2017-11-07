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
	Essa etapa consiste em pegar o valor analogica do sensor e converter para graus celsius. Para isso, você deverá achar uma função de transferência que faça essa conversão.
</p><p>
	Para fazer a medição nós usamos três valores de referência  temperatura de congelamento da água (0°) temperatura de evaporação da água (100°) e a temperatura ambiente (que no momento era 31° mas pode variar). 
</p><p>
	Com esse dados obtivemos a função de transferência, através de um processo matemático chamado regressão.  
</p><p>	
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

## **4.    CONCLUSÃO**
 
### **4.1.        APLICAÇÕES UTILIZANDO ESTE PROJETO**
 
Esse projeto pode ser utilizado para o controle de temperatura em qualquer ambiente, desde uma pequena caixa (quadro elétrico, aquário,...) como também uma subestação, utilizando um ventilador maior ou um exaustor para diminuir a temperatura do local.
Alguns quadros elétricos possuem equipamentos que liberam muito calor, podendo ocasionar algum incêndio. Com isso, o nosso produto pode ser instalado nesses tipos de quadro para manter sempre uma temperatura agradável. A figura abaixo mostra um quadro com um ventilador instalado para essa finalidade.

  
 
Outra aplicação com esse projeto seria a instalação em mini-racks, com isso, a temperatura interna seria controlada, diminuindo o custo com energia.
 
 
Também é uma solução para quem possui aquários e deseja controlar a temperatura da água. Existem alguns tipos de peixes que não toleram variações grandes e por isso devem ser criados com a máxima estabilidade. 

 
### **4.2.        MELHORIAS**
 
Esse produto pode ser melhorado adicionando alguns equipamentos e aumentando as funcionalidades e finalidades.
 
Exemplo 01: Variação de temperatura.
Caso o usuário queira variar a temperatura desejada, pode-se colocar um display com botões. Com isso, esse produto poderia ser utilizado tanto em locais mais frios como mais quentes, já que há uma facilidade maior de alterar a temperatura desejada. Na figura abaixo, é mostrado um modelo já comercializado de como isso é possível.

  
 
Exemplo 02: Redução do controlador.
Uma melhoria significativa do produto seria na troca do controlador. As funcionalidades e o programa embarcado são muito simples para esse projeto, com isso, um microcontrolador básico com menos memória, quantidade de pinos digitais e analógicos reduziria o custo. Além disso, o tamanho físico de uma placa eletrônica seria bem mais compacta, já que seria confeccionada com menos quantidade de trilhas e componentes, e integrando um microcontrolador PIC ou um ARM no lugar de um Arduino completo.
 
### **4.3.        CUSTOS DO PRODUTO**
 
A seguir, é mostrado o preço dos materiais mais significativos do projeto. Esses valores foram baseados nos sites www.mercadolivre.com.br e www.autocore.com.br.
 
MATERIAL	REFÊRENCIA	FABRICANTE	PREÇO R$
Arduino	Uno Rev3	Arduino	26,80
Sensor de Temperatura	CILM35DZ	National Instruments	7,50
Cooler	E97375-001	intel	29,99
Fonte 12Vcc	-	-	10,00
TOTAL	 	 	74,29
 
 
### **4.4.        VALOR DO PRODUTO**
 
Baseado no custo do item anterior, e colocando uma margem de lucro de 50%, o valor para ser comercializado seria de R$ 110,00.
As aplicações já listadas do nosso projeto (controle de temperatura em quadros elétricos, mini-racks e aquários) já existem em alguns produtos no mercado. Para se ter uma comparação, o preço de um AquaCool Master Fan  que serve para ser instalado em um aquário, custa R$ 224,00. Um kit básico para rack com 2 ventiladores custa R$ 134,00 com o frete.
 

## **4.5.        REFERÊNCIAS DE MATERIAIS**

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
