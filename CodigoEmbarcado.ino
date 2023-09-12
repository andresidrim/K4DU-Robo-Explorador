#include <NewPing.h> // Bilioteca para o sensor ultrasonico
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

#define velocity 200 // Velocidade de rotação dos motores

// Variaveis do timer //
unsigned long initialTime; // Tempo inicial do timer em milissegundos
unsigned long delayTime = 5000; // Tempo desejado em milissegundos (5 segundos)

// Variaveis do Sensor Ultrasonico (HC-SR04) //
#define ECHO 11 // Pino de recebimento das ondas do sensor
#define TRIGGER 10 // Pino de envio das ondas do sensor
#define MAX_DISTANCE 200 // Distancia máxima que o sensor vai ler em cm

NewPing usensor(TRIGGER, ECHO, MAX_DISTANCE); // Definição do objeto 'usensor'

// FUNÇÕES //

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

// SENSOR ULTRASONICO //

// Retorna a distancia lida do sensor ultrasonico
uint8_t GetUltrasonicDistance() {
  uint8_t distance = usensor.ping_cm(); // Salva a distancia (em cm) em uma variavel
  return distance; // Retorna a distancia
}

void setup() {
  Serial.begin(9600);

  // Definição do pino ENABLE como saida
  pinMode(ENABLE_MOTOR_ESQUERDO, OUTPUT);
  pinMode(ENABLE_MOTOR_DIREITO, OUTPUT);

  // Configurações do motor esquerdo
  motor_esquerdo.setMaxSpeed(200); // Velocidade máxima do motor = 100 passos por segundo
  motor_esquerdo.setAcceleration(200); // Aceleração do motor = 100 passos por segundo²

  // Configurações do motor direito
  motor_direito.setMaxSpeed(200);
  motor_direito.setAcceleration(200);

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
    }

  delay(10); // Pequeno delay para evitar processamento excessivo
}