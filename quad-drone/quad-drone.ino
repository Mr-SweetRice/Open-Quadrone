#include <OpenQuadrone.h>

#define SDA         0
#define SCL         1
#define CustomIMU   false

// Instancia o framework OpenQuadrone
OpenQuadrone quad(CustomIMU, SDA, SCL);

float X, Y, Z;
void setup() {
  // Inicializa IMU e Serial
  quad.begin();
  Serial.begin(115200);
}

void loop() {
  quad.getAngles(X, Y, Z);
  
  Serial.print("XDS:= "); Serial.print(X);
  Serial.print(" YDS:= "); Serial.print(Y);
  Serial.print(" ZDS:= "); Serial.println(Z);
} 
