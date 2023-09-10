#include <AccelStepper.h> // Biblioteca usada para o  motor de passo

// Definição dos pinos //

// Variaveis do motor //
// Motor esquerdo //
#define STEP_MOTOR_ESQUERDO 6 // A frequencia de pulsos enviados para o pino STEP controla a velocidade no motor
#define DIR_MOTOR_ESQUERDO 7 // Controla a direção do motor (HIGH para horário | LOW para anti-horário)
#define ENABLE_MOTOR_ESQUERDO 2 // LOW = Gira o motor | HIGH = Trava o motor

// Motor direito //
#define STEP_MOTOR_DIREITO 8
#define DIR_MOTOR_DIREITO 9
#define ENABLE_MOTOR_DIREITO 3

// Definição dos objetos 'motor_esquerdo' e 'motor_direito' //
AccelStepper motor_esquerdo(1, STEP_MOTOR_ESQUERDO, DIR_MOTOR_ESQUERDO);
AccelStepper motor_direito(1, STEP_MOTOR_DIREITO, DIR_MOTOR_DIREITO);

#define velocity 100 // Velocidade de rotação dos motores

// MOTOR //

/* ########################################################################################
OBS: A mudança de direção dos motores será feita diretamente na velocidade dos motores
Se a velocidade ter sinal positivo (velocity > 0) o motor irá girar no sentido horário
Se a velocidade ter sinal negativo (velocity < 0) o motor irá girar no sentido anti-horário
######################################################################################## */

// Move o robo para frente
void MoveForward() {
  // Permite girar os motores
  digitalWrite(ENABLE_MOTOR_ESQUERDO, LOW);
  digitalWrite(ENABLE_MOTOR_DIREITO, LOW);

  // Define a velocidade de rotação dos motores
  motor_esquerdo.setSpeed(velocity); // Gira no sentido horário
  motor_direito.setSpeed(-velocity); // Gira no sentido anti-horário

  // Gira os motores
  motor_esquerdo.runSpeed(); 
  motor_direito.runSpeed();
}

// Move o robô para trás //
void MoveBackwards() {
  // Permite girar o motor
  digitalWrite(ENABLE_MOTOR_ESQUERDO, LOW);
  digitalWrite(ENABLE_MOTOR_DIREITO, LOW);

  // Define a velocidade de rotação dos motores
  motor_esquerdo.setSpeed(-velocity); // Gira no sentido anti-horário
  motor_direito.setSpeed(velocity); // Gira no sentido horário

  // Gira os motores
  motor_esquerdo.runSpeed();
  motor_direito.runSpeed();
}

// Faz uma curva para a esquerda
void TurnLeft() {
  // Permite girar o motor
  digitalWrite(ENABLE_MOTOR_ESQUERDO, LOW);
  digitalWrite(ENABLE_MOTOR_DIREITO, LOW);

  // Define a velocidade de rotação dos motores
  motor_esquerdo.setSpeed(velocity); // Gira no sentido horário
  motor_direito.setSpeed(velocity); // Gira no sentido horário

  // Gira os motores
  motor_esquerdo.runSpeed();
  motor_direito.runSpeed();
}

// Faz uma curva para a direita
void TurnRight() {
  // Permite girar o motor
  digitalWrite(ENABLE_MOTOR_ESQUERDO, LOW);
  digitalWrite(ENABLE_MOTOR_DIREITO, LOW);

  // Define a velocidade de rotação dos motores
  motor_esquerdo.setSpeed(-velocity); // Gira no sentido anti-horário
  motor_direito.setSpeed(-velocity); // Gira no sentido anti-horário

  // Gira os motores
  motor_esquerdo.runSpeed();
  motor_direito.runSpeed();
}

// Para o robô //
void Stop() {
  // Trava os motores
  digitalWrite(ENABLE_MOTOR_ESQUERDO, HIGH);
  digitalWrite(ENABLE_MOTOR_DIREITO, HIGH);

  // Para os motores
  motor_esquerdo.stop();
  motor_direito.stop();
}

// Variaveis do timer
unsigned long initialTime; // Tempo inicial do timer em milissegundos
unsigned long delayTime = 5000; // Tempo desejado em milissegundos (5 segundos)

void setup() {
  Serial.begin(9600);

  // Definição do pino ENABLE como saida
  pinMode(ENABLE_MOTOR_ESQUERDO, OUTPUT);
  pinMode(ENABLE_MOTOR_DIREITO, OUTPUT);

  // Configurações do motor esquerdo
  motor_esquerdo.setMaxSpeed(100); // Velocidade máxima do motor = 100 passos por segundo
  motor_esquerdo.setAcceleration(100); // Aceleração do motor = 100 passos por segundo²

  // Configurações do motor direito
  motor_direito.setMaxSpeed(100);
  motor_direito.setAcceleration(100);

  // Os motores devem levar em torno de 1 segundo para atingir a velocidade máxima //

  // Motores ficam inativos por um segundo
  digitalWrite(ENABLE_MOTOR_ESQUERDO, HIGH);
  digitalWrite(ENABLE_MOTOR_DIREITO, HIGH);
  delay(1000);
}

void loop() {
  initialTime = millis(); // Salva o tempo atual em milissegundos
  
  // Roda o trecho de código enquanto o tempo atual for menor que o tempo inicial + o tempo do delay
  while(millis() < initialTime + delayTime) {
    MoveForward();
    Serial.println("Move Forward");
  }

  initialTime = millis();
  while(millis() < initialTime + delayTime) {
    MoveBackwards();
    Serial.println("Move Backwards");
  }

  initialTime = millis();
  while(millis() < initialTime + delayTime) {
    TurnLeft();
    Serial.println("Turn Left");
  }

  initialTime = millis();
  while(millis() < initialTime + delayTime) {
    TurnRight();
    Serial.println("Turn Right");
  }

  initialTime = millis();
  while(millis() < initialTime + delayTime) {
    Stop();
    Serial.println("Stop");
  }

  delay(10); // Pequeno delay para evitar processamento excessivo
}