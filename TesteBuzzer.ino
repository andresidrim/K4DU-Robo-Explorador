#define BUZZER 5 // Porta digital ligada ao buzzer

// Possibilita a escolha das frequencias tocadas pelo buzzer e por quanto tempo será tocada
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

void setup() {}

void loop() {
  SetBuzzerTone(262, 500); // Dó
  SetBuzzerTone(294, 500); // Ré
  SetBuzzerTone(330, 500); // Mi
  SetBuzzerTone(349, 500); // Fá
  SetBuzzerTone(392, 500); // Sol
  SetBuzzerTone(440, 500); // Lá
  SetBuzzerTone(494, 500); // Si
  SetBuzzerTone(528, 500); // #Dó
}
