// TODO
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
#define LEFT_MOTOR_ENABLE 9 // LOW = Gira o motor | HIGH = Trava o motor
#define LEFT_MOTOR_STEP 10 // A frequencia de pulsos enviados para o pino STEP controla a velocidade no motor
#define LEFT_MOTOR_DIR 11 // Controla a direção do motor (HIGH para horário | LOW para anti-horário)

// Motor direito //
#define RIGHT_MOTOR_ENABLE 50
#define RIGHT_MOTOR_STEP 48
#define RIGHT_MOTOR_DIR 49

// Definição dos objetos 'left_motor' e 'right_motor' //
AccelStepper left_motor(1, LEFT_MOTOR_STEP, LEFT_MOTOR_DIR);
AccelStepper right_motor(1, RIGHT_MOTOR_STEP, RIGHT_MOTOR_DIR);

#define VELOCITY 250 // Velocidade de rotação dos motores = 200 passos por segundo
#define ACCELERATION 250 // Velocidade da aceleração dos motores = 200 passos por segundo²

#pragma endregion

#pragma region Variaveis Timer

// Variaveis do timer //
unsigned long initialTime;
unsigned int waitTime = 15000; // Tempo desejado em milissegundos (5 segundos)

#pragma endregion

#pragma region Varaveis do sensor ultrassonico

// Variaveis do Sensor Ultrasonico (HC-SR04) //
#define ECHO 52 // Pino de recepeção de dados
#define TRIGGER 53 // Pino de envio das ondas do sensor
#define MAX_DISTANCE 200 // Distancia máxima que o sensor vai ler em cm

NewPing usensor(TRIGGER, ECHO, MAX_DISTANCE); // Definição do objeto 'usensor'

#pragma endregion

#pragma region Variaveis do sensor infravermelho

#define IR_FAR_LEFT 47
#define IR_LEFT 46
#define IR_RIGHT 45
#define IR_FAR_RIGHT 44

#pragma endregion

#pragma region Variaveis da celula de carga

// Variaveis da celula de carga //

#define DT A15
#define SCK A14

/* Sensor usa portas analógicas A1 e A0 */

// Valor calculado para obter os valores corretos
#define SET_SCALE_VALUE 72627

// Criação do objeto 'scale' //
HX711 scale;

#pragma endregion

#pragma region Variaveis dos botões

#define NEXT_BUTTON 45
#define BACK_BUTTON 47
#define CONFIRM_BUTTON 43

#pragma endregion

#pragma region Variaveis do LCD

#define SDA A0
#define SCL A1
#define ADDRESS 0x27 // TODO

#define COL 16
#define ROW 2

LiquidCrystal_I2C LCD(ADDRESS, COL, ROW);

#pragma endregion

#pragma region Variaveis do LED

#define LED_LEFT 30
#define LED_MIDDLE 29
#define LED_RIGHT 28

bool ledLeftState = HIGH;
bool ledMiddleState = LOW;
bool ledRightState = HIGH;

#pragma endregion

#pragma region Variaveis do Buzzer

#define BUZZER 7

#pragma endregion

#pragma region Varaveis para checagem da rotina

#define STATION_AMOUNT 3

#define STATION1 1
#define STATION2 2
#define STATION3 3

bool hasConfirmedWeight = false;
bool hasConfirmedPath = false;
bool hasComputedPaths = false;
bool hasCleared = false;

#pragma endregion

#pragma region Caminhos

uint8_t pathOrder[3];

bool hasBeenToStation1 = false;
bool hasBeenToStation2 = false;
bool hasBeenToStation3 = false;

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
void MotorMoveForward() {
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, HIGH);
  digitalWrite(LED_RIGHT, LOW);

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
void MotorMoveBackwards() {
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, HIGH);
  digitalWrite(LED_RIGHT, LOW);

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
void MotorTurnLeft() {
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_MIDDLE, LOW);
  digitalWrite(LED_RIGHT, LOW);

  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.stop(); // Gira no sentido horário
  right_motor.setSpeed(VELOCITY); // Gira no sentido horário

  // Gira os motores
  // left_motor.runSpeed();
  right_motor.runSpeed();
}

void MotorAdjustLeft() {
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, HIGH);
  digitalWrite(LED_LEFT, LOW);
  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(-VELOCITY / 2); // Gira no sentido horário
  right_motor.setSpeed(VELOCITY); // Gira no sentido horário

  // Gira os motores
  left_motor.runSpeed();
  right_motor.runSpeed();
}

// Faz uma curva para a direita
void MotorTurnRight() {
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_MIDDLE, LOW);
  digitalWrite(LED_LEFT, LOW);

  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(-VELOCITY); // Gira no sentido anti-horário
  right_motor.stop(); // Gira no sentido anti-horário

  // Gira os motores
  left_motor.runSpeed();
  // right_motor.runSpeed();
}

void MotorAdjustRight() {
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, HIGH);
  digitalWrite(LED_LEFT, LOW);

  // Permite girar o motor
  digitalWrite(LEFT_MOTOR_ENABLE, LOW);
  digitalWrite(RIGHT_MOTOR_ENABLE, LOW);

  // Define a velocidade de rotação dos motores
  left_motor.setSpeed(-VELOCITY); // Gira no sentido anti-horário
  right_motor.setSpeed(VELOCITY / 2); // Gira no sentido anti-horário

  // Gira os motores
  left_motor.runSpeed();
  right_motor.runSpeed();
}

// Para o robô //
void MotorStop() {
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, LOW);
  digitalWrite(LED_LEFT, LOW);
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
  
  return distance; // Retorna a distancia
}

#pragma endregion

#pragma region Funções da celula de carga

void SetScaleSettings() {
  scale.begin(DT, SCK);

  scale.set_scale(SET_SCALE_VALUE);
  scale.tare(); // Tira uma tara dos valores que recebe, baseando-se nos valores de set_scale
}

bool HasRemovedWeight (float currentWeight) {
  return abs(scale.get_units(10)) < currentWeight;
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
  pinMode(IR_FAR_LEFT, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FAR_RIGHT, INPUT);
}

// bool DetectLeftTurn() {
//   return (!digitalRead(IR_FAR_LEFT) && !digitalRead(IR_MIDDLE) && digitalRead(IR_RIGHT));
// }

// bool DetectRightTurn() {
//   return (digitalRead(IR_LEFT) && !digitalRead(IR_MIDDLE) && !digitalRead(IR_RIGHT));
// }

// if digitalRead() -> preto
// if !digitalRead() -> branco


bool DetectCrossroad() {
  return (!digitalRead(IR_FAR_LEFT) && !digitalRead(IR_LEFT) && !digitalRead(IR_RIGHT) && !digitalRead(IR_FAR_RIGHT));
}

bool AdjustLeft() {
  return (digitalRead(IR_FAR_LEFT) && !digitalRead(IR_LEFT) && digitalRead(IR_RIGHT) && digitalRead(IR_FAR_RIGHT));
}

bool AdjustRight() {
  return (digitalRead(IR_FAR_LEFT) && digitalRead(IR_LEFT) && !digitalRead(IR_RIGHT) && digitalRead(IR_FAR_RIGHT));
}

bool DetectStraight() {
  return (!digitalRead(IR_FAR_LEFT) && !digitalRead(IR_LEFT) && !digitalRead(IR_RIGHT) && !digitalRead(IR_FAR_RIGHT));
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

void LEDSecurityPhaseHIGH() {
  digitalWrite(LED_LEFT, HIGH);
  digitalWrite(LED_MIDDLE, HIGH);
  digitalWrite(LED_RIGHT, HIGH);
}

void LEDSecurityPhaseLOW() {
  digitalWrite(LED_LEFT, LOW);
  digitalWrite(LED_MIDDLE, LOW);
  digitalWrite(LED_RIGHT, LOW);
}

#pragma endregion

#pragma region Funções do Botão

void SetButtonSettings() {
  pinMode(BACK_BUTTON, INPUT);
  pinMode(CONFIRM_BUTTON, INPUT);
  pinMode(NEXT_BUTTON, INPUT);
}

#pragma endregion

#pragma region Funções do Buzzer

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

bool HasLoadedRobot() {
  if (!hasCleared) {
    LCD.clear();
    LCD.setCursor(0, 0);
    LCD.print("Confirmacao");
    LCD.setCursor(0, 1);
    LCD.print("do peso");
    delay(2500);
    LCD.clear();
    hasCleared = true;
  }

  while (!digitalRead(BACK_BUTTON) && !digitalRead(CONFIRM_BUTTON)) {
    LCD.setCursor(0, 0);
    LCD.print("Peso (kg): ");
    LCD.print(abs(scale.get_units(10)));
  }

  if (digitalRead(BACK_BUTTON)) {
    LCD.clear();
    return false;
  }

  else if (digitalRead(CONFIRM_BUTTON)) {
    LCD.clear();
    delay(250);
    
    while (!digitalRead(CONFIRM_BUTTON)) {
      LCD.setCursor(0, 0);
      LCD.print("Voce deseja");
      LCD.setCursor(0, 1);
      LCD.print("confirmar?");

      if (digitalRead(BACK_BUTTON)) {
        LCD.clear();
        return false;
      }
      else if (digitalRead(CONFIRM_BUTTON)) {
        LCD.clear();
        LCD.setCursor(0, 0);
        LCD.print("Peso");
        LCD.setCursor(0, 1);
        LCD.print("Confirmado");
        delay(2500);
        LCD.clear();
        return true;
      }
    }
  }
}

bool HasSelectedPath() {
  uint8_t pathChoices[3] = {1, 2, 3};
  uint8_t timesPressed = 0;
  int8_t indexSelected[2] = {-1, -1};

  bool hasConfirmed = false;

  LCD.clear();
  LCD.setCursor(0, 0);
  LCD.print("Selecao");
  LCD.setCursor(0, 1);
  LCD.print("das estacoes");
  delay(2500);
  LCD.clear();

  while (int i = 0 < STATION_AMOUNT) {
    LCD.clear();
    while (!digitalRead(NEXT_BUTTON) || !digitalRead(BACK_BUTTON)) {
      LCD.setCursor(0, 0);
      LCD.print("Estacao ");
      LCD.print(pathChoices[i]);

      if (digitalRead(NEXT_BUTTON)) {
        if (i == 2) { i = 0; }
        else { i++; }
        for (int j = 0; j <= timesPressed; j++) {
          if (indexSelected[j] == i) {
            i++;
          }
        }
        if (i > 2) { i = 0; }
        delay(250);
      }

      else if (digitalRead(BACK_BUTTON)) {
        if (i == 0) { i = 2; }
        else { i--; }
        for (int j = 0; j <= timesPressed; j++) {
          if (indexSelected[j] == i) {
            i--;
          }
        }
        if (i < 0) { i = 2; }
        delay(250);
      }

      else if (digitalRead(CONFIRM_BUTTON)) {
        LCD.clear();
        delay(250);

        while (!digitalRead(CONFIRM_BUTTON)) {
          LCD.setCursor(0, 0);
          LCD.print("Voce deseja");
          LCD.setCursor(0, 1);
          LCD.print("confirmar?");
          hasConfirmed = true;

          if (digitalRead(BACK_BUTTON)) {
            LCD.clear();
            hasConfirmed = false;
            break;
          }
        }
        LCD.clear();
        delay(250);

        if (hasConfirmed) {
          LCD.setCursor(0, 0);
          LCD.print("Estacao");
          LCD.setCursor(0, 1);
          LCD.print("Confirmada");
          delay(2500);

          LCD.clear();

          pathOrder[timesPressed] = pathChoices[i];
          indexSelected[timesPressed] = i;
          if (timesPressed == 2) {
            LCD.clear();
            LCD.setCursor(0, 0);
            LCD.print("Ordem Estacoes:");
            LCD.setCursor(0, 1);
            for (int j = 0; j < STATION_AMOUNT; j++) {
              LCD.print(pathOrder[j]);
              if (j < 2) { LCD.print(" -> "); }
            }
            delay(2500);
            LCD.clear();
            return true;
          }

          timesPressed++;

          i++;
          for (int j = 0; j < 2; j++) {
          if (indexSelected[j] == i) {
            i++;
          }
        }
          if (i > 2) { i = 0; }
        }
      }
    }
  }
}

bool HasComputedPaths() {
  unsigned long startTimer;
  bool isInStation = false;
  float currentWeight;

  switch (pathOrder[0]) {
    case STATION1:
        if (pathOrder[1] == STATION2) {
          while (AdjustLeft()) { MotorAdjustLeft(); }
          while (AdjustRight()) { MotorAdjustRight(); }
          MotorMoveForward();

          if (DetectCrossroad()) {
            startTimer = millis();
            while (Timer(startTimer, 750)) {
              MotorMoveForward();
              if (DetectCrossroad()) { isInStation = true; }
            }

            if (isInStation) {
              MotorStop();
              delay(250);

              startTimer = millis();
              while (Timer(startTimer, 2000)) { // Robo gira até virar 180°
                MotorTurnRight();
              }

              startTimer = millis();
              while (Timer(startTimer, 500)) { // Robo da ré para se ajustar na plataforma
                MotorMoveBackwards();
              }

              MotorStop();
              LCD.clear();
              currentWeight = abs(scale.get_units(10));
              while (!HasRemovedWeight(currentWeight)) {
                LCD.setCursor(0, 0);
                LCD.print("Por favor");
                LCD.setCursor(0, 1);
                LCD.print("Remover Carga");
              }

              if (!hasBeenToStation1) { hasBeenToStation1 = true; }
              else if (!hasBeenToStation2) { hasBeenToStation2 = true; }
              else { hasBeenToStation3 = true; }
            }

            else if (!hasBeenToStation1) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }

            else if (!hasBeenToStation2) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
            else {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
          }

          if (hasBeenToStation1 && hasBeenToStation2 && hasBeenToStation3) { break; }
        }
        else if (pathOrder[1] == STATION3) {
          while (AdjustLeft()) { MotorAdjustLeft(); }
          while (AdjustRight()) { MotorAdjustRight(); }
          MotorMoveForward();

          if (DetectCrossroad()) {
            startTimer = millis();
            while (Timer(startTimer, 750)) {
              MotorMoveForward();
              if (DetectCrossroad()) { isInStation = true; }
            }

            if (isInStation) {
              MotorStop();
              delay(250);

              startTimer = millis();
              while (Timer(startTimer, 2000)) { // Robo gira até virar 180°
                MotorTurnRight();
              }

              startTimer = millis();
              while (Timer(startTimer, 500)) { // Robo da ré para se ajustar na plataforma
                MotorMoveBackwards();
              }

              MotorStop();
              LCD.clear();
              currentWeight = abs(scale.get_units(10));
              while (!HasRemovedWeight(currentWeight)) {
                LCD.setCursor(0, 0);
                LCD.print("Por favor");
                LCD.setCursor(0, 1);
                LCD.print("Remover Carga");
              }

              if (!hasBeenToStation1) { hasBeenToStation1 = true; }
              else if (!hasBeenToStation2) { hasBeenToStation2 = true; }
              else { hasBeenToStation3 = true; }
            }

            else if (!hasBeenToStation1) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }

            else if (!hasBeenToStation2) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
            else {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
          }

          if (hasBeenToStation1 && hasBeenToStation2 && hasBeenToStation3) { break; }
        }
    case STATION2:
        if (pathOrder[1] == STATION1) {
          // estacao 2, estacao 1, estacao 3
          if (hasBeenToStation1 && hasBeenToStation2 && hasBeenToStation3) { break; }
        }
        else if (pathOrder[1] == STATION3) {
          // estacao 2, estacao 3, estacao 1
          if (hasBeenToStation1 && hasBeenToStation2 && hasBeenToStation3) { break; }
        }
    case STATION3:
        if (pathOrder[1] == STATION1) {
          while (AdjustLeft()) { MotorAdjustLeft(); }
          while (AdjustRight()) { MotorAdjustRight(); }
          MotorMoveForward();

          if (DetectCrossroad()) {
            startTimer = millis();
            while (Timer(startTimer, 750)) {
              MotorMoveForward();
              if (DetectCrossroad()) { isInStation = true; }
            }

            if (isInStation) {
              MotorStop();
              delay(250);

              startTimer = millis();
              while (Timer(startTimer, 2000)) { // Robo gira até virar 180°
                MotorTurnRight();
              }

              startTimer = millis();
              while (Timer(startTimer, 500)) { // Robo da ré para se ajustar na plataforma
                MotorMoveBackwards();
              }

              MotorStop();
              LCD.clear();
              currentWeight = abs(scale.get_units(10));
              while (!HasRemovedWeight(currentWeight)) {
                LCD.setCursor(0, 0);
                LCD.print("Por favor");
                LCD.setCursor(0, 1);
                LCD.print("Remover Carga");
              }

              if (!hasBeenToStation1) { hasBeenToStation1 = true; }
              else if (!hasBeenToStation2) { hasBeenToStation2 = true; }
              else { hasBeenToStation3 = true; }
            }

            else if (!hasBeenToStation1) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }

            else if (!hasBeenToStation2) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
            else {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
          }

          if (hasBeenToStation1 && hasBeenToStation2 && hasBeenToStation3) { break; }
        }
        else if (pathOrder[1] == STATION2) {
          while (AdjustLeft()) { MotorAdjustLeft(); }
          while (AdjustRight()) { MotorAdjustRight(); }
          MotorMoveForward();

          if (DetectCrossroad()) {
            startTimer = millis();
            while (Timer(startTimer, 750)) {
              MotorMoveForward();
              if (DetectCrossroad()) { isInStation = true; }
            }

            if (isInStation) {
              MotorStop();
              delay(250);

              startTimer = millis();
              while (Timer(startTimer, 2000)) { // Robo gira até virar 180°
                MotorTurnRight();
              }

              startTimer = millis();
              while (Timer(startTimer, 500)) { // Robo da ré para se ajustar na plataforma
                MotorMoveBackwards();
              }

              MotorStop();
              LCD.clear();
              currentWeight = abs(scale.get_units(10));
              while (!HasRemovedWeight(currentWeight)) {
                LCD.setCursor(0, 0);
                LCD.print("Por favor");
                LCD.setCursor(0, 1);
                LCD.print("Remover Carga");
              }

              if (!hasBeenToStation1) { hasBeenToStation1 = true; }
              else if (!hasBeenToStation2) { hasBeenToStation2 = true; }
              else { hasBeenToStation3 = true; }
            }

            else if (!hasBeenToStation1) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }

            else if (!hasBeenToStation2) {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
            else {
              MotorStop();
              delay(250);
              
              while (Timer(startTimer, 1000)) { // Robo gira até virar 90°
                MotorTurnLeft();
              }
            }
          }

          if (hasBeenToStation1 && hasBeenToStation2 && hasBeenToStation3) { break; }
        }
    default:
        return false;
    }
    return true;
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

  SetMotorSettings(VELOCITY, ACCELERATION); // Define os valores de velocidade e aceleração do motor
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

  // if (!HasComputedPaths()) {
  //   LCD.clear();
  //   LCD.setCursor(0, 0);
  //   LCD.print("Failed Path");
  //   LCD.setCursor(0, 1);
  //   LCD.print("Calculation");
  //   hasConfirmedWeight = false;
  //   hasConfirmedPath = false;
  //   delay(1000);
  // }

  // while (HasDetectedObstacle()) {
  //   MotorStop();
  //   LEDSecurityPhaseHIGH();
  //   SetBuzzerTone(400, 500);
  //   LEDSecurityPhaseLOW();
  //   SetBuzzerTone(100, 500);
  // }

  // while (!hasConfirmedWeight) { hasConfirmedWeight = HasLoadedRobot(); }
  // while (!hasConfirmedPath) { hasConfirmedPath = HasSelectedPath(); }

  while(true) { MotorMoveForward();} 

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