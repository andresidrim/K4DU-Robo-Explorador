#include "HX711.h" // Biblioteca usada pela célula de carga

// Definição dos pinos //

// Célula de carga //
#define DT A1
#define SCK A0

/* Sensor usa portas analógicas A1 e A0 */

// Criação do objeto 'scale' //
HX711 scale;
 
void setup() {
  Serial.begin(9600); // Inicia o monitor serial

  scale.begin(DT, SCK); // Usa a função 'begin' e recebe os pinos DT e SCK como parametros
  scale.set_scale(72627); // Valor definido para melhor precisão nas medições

  /* ##############################################################################
  O valor lido pelo sensor (sem nenhum parametro dentro de set_scale())
  foi dividido pelo peso de um objeto de referencia, em kilogramas,
  para obter os resultados nessa mesma unidade.
  
  O resultado desta conta sofreu alguns pequenos ajustes para obter melhor precisão
  ############################################################################## */

  scale.tare(); // Tira uma tara dos valores que recebe, baseando-se nos valores de set_scale
}
 
void loop() {
  Serial.print("Valor da Leitura:  ");
  Serial.println(scale.get_units(10), 3);  // Printa a massa do objeto, em kilogramas (valor irá aparecer com 3 casas decimais)
  delay(100);
}