#include <OpenQuadrone.h>

#define SDA         0
#define SCL         1
#define CustomIMU   false

// Instancia o framework OpenQuadrone
OpenQuadrone quad(CustomIMU, SDA, SCL);
float alt;
float X, Y, Z;
void setup() {
  // Inicializa IMU e Serial
  quad.begin();
  Serial.begin(115200);
}

void loop() {
  //quad.kalmanFilter();
  //quad.readRaw();
  getAngles(roll,pitch,yaw)
  quad.getAltitude(alt);
  Serial.println(alt);
}
