#include <NewPing.h> // Bilioteca para o sensor ultrasonico

// Definição dos pinos //

// Variaveis do Sensor Ultrasonico (HC-SR04) //
#define ECHO 11 // Pino de recebimento das ondas do sensor
#define TRIGGER 10 // Pino de envio das ondas do sensor
#define MAX_DISTANCE 200 // Distancia máxima que o sensor vai ler em cm

NewPing usensor(TRIGGER, ECHO, MAX_DISTANCE); // Definição do objeto 'usensor'

// SENSOR ULTRASONICO //

// Retorna a distancia lida do sensor ultrasonico
uint8_t GetUltrasonicDistance() {
  uint8_t distance = usensor.ping_cm(); // Salva a distancia (em cm) em uma variavel
  return distance; // Retorna a distancia
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Distancia: ");
  Serial.print(GetUltrasonicDistance()); // Printa a distancia do sensor
  Serial.println("cm");
  delay(500);
}
