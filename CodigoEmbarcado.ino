#include <NewPing.h> // Bilioteca para o sensor ultrasonico
#include <AccelStepper.h> // Biblioteca usada para o  motor de passo

// Definição dos pinos //

// Variaveis do motor //
// Motor esquerdo //
#define LEFT_MOTOR_STEP 6 // A frequencia de pulsos enviados para o pino STEP controla a velocidade no motor
#define LEFT_MOTOR_DIR 7 // Controla a direção do motor (HIGH para horário | LOW para anti-horário)
#define LEFT_MOTOR_ENABLE 2 // LOW = Gira o motor | HIGH = Trava o motor

// Motor direito //
#define RIGHT_MOTOR_STEP 8
#define RIGHT_MOTOR_DIR 9
#define RIGHT_MOTOR_ENABLE 3

// Definição dos objetos 'left_motor' e 'right_motor' //
AccelStepper left_motor(1, LEFT_MOTOR_STEP, LEFT_MOTOR_DIR);
AccelStepper right_motor(1, RIGHT_MOTOR_STEP, RIGHT_MOTOR_DIR);

#define VELOCITY 200 // Velocidade de rotação dos motores = 200 passos por segundo
#define ACCELERATION 200 // Velocidade da aceleração dos motores = 200 passos por segundo²

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

// SENSOR ULTRASONICO //

// Retorna a distancia lida do sensor ultrasonico
uint8_t GetUltrasonicDistance() {
  uint8_t distance = usensor.ping_cm(); // Salva a distancia (em cm) em uma variavel
  return distance; // Retorna a distancia
}

void setup() {
  Serial.begin(9600);

  // Definição do pino ENABLE como saida
  pinMode(LEFT_MOTOR_ENABLE, OUTPUT);
  pinMode(RIGHT_MOTOR_ENABLE, OUTPUT);

  // Configurações do motor esquerdo
  left_motor.setMaxSpeed(VELOCITY); // Define a velocidade máxima do motor
  left_motor.setAcceleration(ACCELERATION); // Define a aceleração do motor

  // Configurações do motor direito
  right_motor.setMaxSpeed(VELOCITY);
  right_motor.setAcceleration(ACCELERATION);

  // Os motores devem levar em torno de 1 segundo para atingir a velocidade máxima //

  // Motores ficam inativos por um segundo
  digitalWrite(LEFT_MOTOR_ENABLE, HIGH);
  digitalWrite(RIGHT_MOTOR_ENABLE, HIGH);
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
