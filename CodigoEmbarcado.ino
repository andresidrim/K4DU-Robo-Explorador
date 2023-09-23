// TODO
// - Ajustar o codigo para usar o multiplexador
// - Implementar a logica do robô no loop
// - Organizar e limpar o codigo

#include <NewPing.h> // Bilioteca para o sensor ultrasonico
#include <AccelStepper.h> // Biblioteca usada para o  motor de passo
#include <HX711.h> // Biblioteca usada pela célula de carga
#include <LiquidCrystal_I2C.h>


#pragma region Variaveis

// Definição das variaveis //

#pragma region Variaveis do motor

// Variaveis do motor //
// Motor esquerdo //
#define LEFT_MOTOR_STEP 6 // A frequencia de pulsos enviados para o pino STEP controla a velocidade no motor
#define LEFT_MOTOR_DIR 7 // Controla a direção do motor (HIGH para horário | LOW para anti-horário)
#define LEFT_MOTOR_ENABLE 5 // LOW = Gira o motor | HIGH = Trava o motor

// Motor direito //
#define RIGHT_MOTOR_STEP 9
#define RIGHT_MOTOR_DIR 10
#define RIGHT_MOTOR_ENABLE 8

// Definição dos objetos 'left_motor' e 'right_motor' //
AccelStepper left_motor(1, LEFT_MOTOR_STEP, LEFT_MOTOR_DIR);
AccelStepper right_motor(1, RIGHT_MOTOR_STEP, RIGHT_MOTOR_DIR);

#define VELOCITY 200 // Velocidade de rotação dos motores = 200 passos por segundo
#define ACCELERATION 200 // Velocidade da aceleração dos motores = 200 passos por segundo²

#pragma endregion

#pragma region Variaveis Timer

// Variaveis do timer //
unsigned long initialTime;
unsigned int waitTime = 15000; // Tempo desejado em milissegundos (5 segundos)

#pragma endregion

#pragma region Varaveis do sensor ultrassonico

// Variaveis do Sensor Ultrasonico (HC-SR04) //
#define ECHO 11 // Pino de recebimento das ondas do sensor
#define TRIGGER 12 // Pino de envio das ondas do sensor
#define MAX_DISTANCE 200 // Distancia máxima que o sensor vai ler em cm

NewPing usensor(TRIGGER, ECHO, MAX_DISTANCE); // Definição do objeto 'usensor'

#pragma endregion

#pragma region Variaveis do sensor infravermelho

#define IR_LEFT 13
#define IR_MIDDLE 14
#define IR_RIGHT 15

#pragma endregion

#pragma region Variaveis da celula de carga

// Variaveis da celula de carga //

#define DT A1
#define SCK A0

/* Sensor usa portas analógicas A1 e A0 */

// Valor calculado para obter os valores corretos
#define SET_SCALE_VALUE 72627

// Criação do objeto 'scale' //
HX711 scale;

#pragma endregion

#pragma region Variaveis dos botões

#define NEXT_BUTTON 2
#define CONFIRM_BUTTON 3

#pragma endregion

#pragma region Variaveis do LCD

#define SDA A4
#define SCL A5
#define ADDRESS 0x27

#define COL 16
#define ROW 2

LiquidCrystal_I2C LCD(ADDRESS, COL, ROW);

#pragma endregion

#pragma region Variaveis do LED

#define LED_LEFT 16
#define LED_MIDDLE 17
#define LED_RIGHT 18

bool ledLeftState = HIGH;
bool ledMiddleState = LOW;
bool ledRightState = HIGH;

#pragma endregion

#pragma region Variaveis do Buzzer

#define BUZZER 5

#pragma endregion

#pragma region Varaveis para checagem da rotina

#define STATION_AMOUNT 3

bool hasConfirmedWeight = false;
bool hasConfirmedPath = false;

#pragma endregion

#pragma endregion



#pragma region Funções

// FUNÇÕES //

#pragma region Funções do timer

// testar
bool Timer(unsigned long inital_time, unsigned int wait_for) {
  inital_time = millis();

  return millis() < initialTime + wait_for;
}

#pragma endregion

#pragma region Funções do motor

// MOTOR //

#pragma region Função para definição dos pinos do motor

void SetMotorSettings(uint8_t motorVel, uint8_t motorAccel) {
  // Definição do pino ENABLE como saida
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  // Configurações do motor esquerdo
  left_motor.setMaxSpeed(motorVel); // Define a velocidade máxima do motor
  left_motor.setAcceleration(motorAccel); // Define a aceleração do motor

  // Configurações do motor direito
  right_motor.setMaxSpeed(motorVel);
  right_motor.setAcceleration(motorAccel);

  // Os motores devem levar em torno de 1 segundo para atingir a velocidade máxima //
}

#pragma endregion

/* ########################################################################################
OBS: A mudança de direção dos motores será feita diretamente na velocidade dos motores
Se a velocidade ter sinal positivo (VELOCITY > 0) o motor irá girar no sentido horário
Se a velocidade ter sinal negativo (VELOCITY < 0) o motor irá girar no sentido anti-horário
######################################################################################## */

// Move o robo para frente
void MoveForward() {
  // Permite girar os motores
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(VELOCITY); // Gira no sentido horário
  right_motor.setSpeed(-VELOCITY); // Gira no sentido anti-horário

  // Gira os motores
  left_motor.runSpeed(); 
  right_motor.runSpeed();
}

// Move o robô para trás //
void MoveBackwards() {
  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(-VELOCITY); // Gira no sentido anti-horário
  right_motor.setSpeed(VELOCITY); // Gira no sentido horário

  // Gira os motores
  left_motor.runSpeed();
  right_motor.runSpeed();
}

// Faz uma curva para a esquerda
void TurnLeft() {
  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(VELOCITY); // Gira no sentido horário
  right_motor.setSpeed(VELOCITY); // Gira no sentido horário

  // Gira os motores
  left_motor.runSpeed();
  right_motor.runSpeed();
}

// Faz uma curva para a direita
void TurnRight() {
  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(-VELOCITY); // Gira no sentido anti-horário
  right_motor.setSpeed(-VELOCITY); // Gira no sentido anti-horário

  // Gira os motores
  left_motor.runSpeed();
  right_motor.runSpeed();
}

// Para o robô //
void Stop() {
  // Trava os motores
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);

  // Para os motores
  left_motor.stop();
  right_motor.stop();
}

#pragma endregion

#pragma region Funções do sensor ultrassonico

// SENSOR ULTRASONICO //

// Retorna a distancia lida do sensor ultrassonico
uint8_t GetUltrasonicDistance() {
  uint8_t distance = usensor.ping_cm(); // Salva a distancia (em cm) em uma variavel
  Serial.println(distance);
  
  return distance; // Retorna a distancia
}

#pragma endregion

#pragma region Funções da celula de carga

void SetScaleSettings() {
  scale.begin(DT, SCK);

  scale.set_scale(SET_SCALE_VALUE);
  scale.tare(); // Tira uma tara dos valores que recebe, baseando-se nos valores de set_scale
}

#pragma endregion

#pragma region Funções do LCD

void SetLCDSettings() {
  LCD.init();
  LCD.setBacklight(HIGH);
}

#pragma endregion

#pragma region Funções do sensor infravermelho

void SetIRSettings() {
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_MIDDLE, INPUT);
  pinMode(IR_RIGHT, INPUT);
}

// if digitalRead() -> preto
// if !digitalRead() -> branco
bool DetectLeftTurn() {
  if (!digitalRead(IR_LEFT) && !digitalRead(IR_MIDDLE) && digitalRead(IR_RIGHT)) { return true; }
  else { return false; }
}

bool DetectRightTurn() {
  if (digitalRead(IR_LEFT) && !digitalRead(IR_MIDDLE) && !digitalRead(IR_RIGHT)) { return true; }
  else { return false; }
}

bool AdjustLeft() {
  if (digitalRead(IR_LEFT) && digitalRead(IR_MIDDLE) && !digitalRead(IR_RIGHT)) { return true; }
  else { return false; }
}

bool AdjustRight() {
  if (!digitalRead(IR_RIGHT) && digitalRead(IR_MIDDLE) && digitalRead(IR_RIGHT)) { return true; }
  else { return false; }
}

#pragma endregion

#pragma region Funções do LED

void SetLEDSettings() {
  pinMode(LED_LEFT, OUTPUT);
  pinMode(LED_MIDDLE, OUTPUT);
  pinMode(LED_RIGHT, OUTPUT);

  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, LOW);
  digitalWrite(LED_RIGHT, LOW);
}

void LEDSetupPhase() {
  digitalWrite(LED_LEFT, ledLeftState);
  digitalWrite(LED_MIDDLE, ledMiddleState);
  digitalWrite(LED_RIGHT, ledRightState);

  ledLeftState = !ledLeftState;
  ledMiddleState = !ledMiddleState;
  ledRightState = !ledRightState;

  delay(500);
}

void LEDSecurityPhase() {
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_MIDDLE, HIGH);
  digitalWrite(LED_RIGHT, HIGH);

  delay(250);

  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, LOW);
  digitalWrite(LED_RIGHT, LOW);

  delay(250);
}

#pragma endregion

#pragma region Funções do Buzzer

// void SetBuzzerSettings() {
//   pinMode(BUZZER, OUTPUT);
// }

void SetBuzzerTone(uint16_t frequency, uint16_t length) {
  /*
  Frequência das notas:
  Dó - 262 Hz
  Ré - 294 Hz
  Mi - 330 Hz
  Fá - 349 Hz
  Sol - 392 Hz
  Lá - 440 Hz
  Si - 494 Hz
  #Dó - 528 Hz
  */

  unsigned long startTime = millis();
  unsigned long halfPeriod = 1000000L / frequency / 2;
  pinMode(BUZZER, OUTPUT);

  while(millis() - startTime < length) {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(halfPeriod);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(halfPeriod);
  }
  pinMode(BUZZER, INPUT);
}

#pragma endregion

#pragma region Funções da rotina

void LoadRobot() {
  LEDSetupPhase();

  waitTime = 500; // 500ms
  float weight = scale.get_units(10);

  bool confirmWeight = false;
  bool resetWeight = false;

  // LCD.setCursor(0, 0);
  LCD.print("Peso: ");
  LCD.print(weight);
  LCD.clear(); // testar

  if (digitalRead(CONFIRM_BUTTON)) {
    initialTime = millis();
    LCD.clear();

    while (Timer(initialTime, waitTime)) { LEDSetupPhase(); }

    do {
      LEDSetupPhase();
      LCD.print("Voce deseja confirmar?");

      confirmWeight = digitalRead(CONFIRM_BUTTON);
      resetWeight = digitalRead(NEXT_BUTTON);
    } while (!resetWeight && !confirmWeight);

    if (resetWeight) { hasConfirmedWeight = false; }
    else if (confirmWeight) { hasConfirmedWeight = true; }
  }
}

void SelectPath() {
  LEDSetupPhase();

  waitTime = 500; // 500ms
  uint8_t pathChoices[3] = {1, 2, 3};
  uint8_t pathOrder[3];
  uint8_t timesPressed = 0;

  bool resetSelection = false;
  bool confirmSelection = false;

  for (int i = 0; i < STATION_AMOUNT; i++) {
    while (!digitalRead(CONFIRM_BUTTON)) {
      LEDSetupPhase();
      LCD.print("Estação ");
      LCD.print(pathChoices[timesPressed]);

      if (digitalRead(NEXT_BUTTON)) {
        LEDSetupPhase();
        LCD.clear();
        timesPressed++;
        if (timesPressed == 3) { timesPressed = 0; }
      }
    }
    pathOrder[i] = pathChoices[timesPressed];
  }

  initialTime = millis();

  LCD.clear();
  while (Timer(initialTime, waitTime)) { LEDSetupPhase(); }

  do {
    LEDSetupPhase();
    LCD.print("Voce deseja confirmar?");

    confirmSelection = digitalRead(CONFIRM_BUTTON);
    resetSelection = digitalRead(NEXT_BUTTON);
  } while (!resetSelection && !confirmSelection);

  if (resetSelection) { hasConfirmedPath = false; }
  else if (confirmSelection) { hasConfirmedPath = true; }
}


#pragma endregion

#pragma region Funções de segurança

bool HasDetectedObstacle() {
  return GetUltrasonicDistance() <= 30;
}

#pragma endregion

void setup() {
  Serial.begin(9600);

  // SetIRSettings();

  // SetScaleSettings(); // Define o valor da tara

  // SetLCDSettings();

  // SetMotorSettings(VELOCITY, ACCELERATION); // Define os valores de velocidade e aceleração do motor
}

void loop() {
  /*Rotina basica do robô*/

  // Objetos devem ser colocados no robô
  // Peso deve ser confirmado -> valor do peso é salvo na memória
  // Ordem das estações são escolhidas -> ordem é salva na memória
  // Verifica e salva as direções necessárias para realizar o percurso
  // Robô vai até a primeira estação e espera o peso ser removido
  // Remoção confirmada
  // Robô segue para a proxima estação (passos anteriores são repetidos)
  // Quando o robô passar por todas as estações e não estiver carregando nada, ele volta para o inicio

  // if (!hasConfirmedWeight) { LoadRobot(); } // Peso é colocado no robo
  // else if (!hasConfirmedPath) { SelectPath(); } // Caminho é escolhido

  // SetBuzzerTone(262, 500); Dó
  // SetBuzzerTone(294, 500); Ré
  // SetBuzzerTone(330, 500); Mi
  // SetBuzzerTone(349, 500); Fá
  // SetBuzzerTone(392, 500); Sol
  // SetBuzzerTone(440, 500); Lá
  // SetBuzzerTone(494, 500); Si
  // SetBuzzerTone(528, 500); #Dó

  if (HasDetectedObstacle()) {
    SetBuzzerTone(400, 500);
    // SetBuzzerTone(294, 500);
    // SetBuzzerTone(330, 500);
    // SetBuzzerTone(349, 500);
    // SetBuzzerTone(392, 500);
    // SetBuzzerTone(440, 500);
    // SetBuzzerTone(494, 500);
    SetBuzzerTone(100, 500);
  }

  delay(10); // Pequeno delay para evitar processamento excessivo
}
































/*initialTime = millis(); // Salva o tempo atual em milissegundos
    
    // Roda o trecho de código enquanto o tempo atual for menor que o tempo inicial + o tempo do delay
    while(millis() < initialTime + delayTime) {
      // Se a distancia for menor que 25
      if(GetUltrasonicDistance() < 25) {
        Stop(); // Para o motor
        return; // Retorna nulo (e volta para o inicio do void loop)
      }
      else {
        MoveForward();
        Serial.print("Move Forward | Distance = ");
        Serial.println(GetUltrasonicDistance());
      }
    }

    initialTime = millis();
    while(millis() < initialTime + delayTime) {
      if(GetUltrasonicDistance() < 25) {
        Stop();
        return;
      }
      else {
        MoveBackwards();
        Serial.print("Move Backwards | Distance = ");
        Serial.println(GetUltrasonicDistance());
      }
    }

    initialTime = millis();
    while(millis() < initialTime + delayTime) {
      if(GetUltrasonicDistance() < 25) {
        Stop();
        return;
      }
      else {
        TurnLeft();
        Serial.print("Turn Left | Distance = ");
        Serial.println(GetUltrasonicDistance());
      }
    }

    initialTime = millis();
    while(millis() < initialTime + delayTime) {
      if(GetUltrasonicDistance() < 25) {
        Stop();
        return;
      }
      else {
        TurnRight();
        Serial.print("Turn Right | Distance = ");
        Serial.println(GetUltrasonicDistance());
      }
    }

    initialTime = millis();
    while(millis() < initialTime + delayTime) {
      Stop();
      if(GetUltrasonicDistance() < 25) { return; }
      else {
        Serial.print("Stop | Distance = ");
        Serial.println(GetUltrasonicDistance());
      }
    }*/