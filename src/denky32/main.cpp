#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define TACH_PIN 4  // Conectar al tach del ventilador

volatile int pulseCount = 0;
unsigned long lastTime = 0;
float rpm = 0;

VL53L0X sensor;


void IRAM_ATTR tachInterrupt();
void setup()
{
  pinMode(TACH_PIN, INPUT);  
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachInterrupt, FALLING);
  Serial.begin(115200);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop()
{
  Serial.print(sensor.readRangeContinuousMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }

  Serial.print(" RPM: ");
  Serial.print(rpm);
  Serial.println();
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= 100) {  // Cada 0.1s calcula RPM
    detachInterrupt(TACH_PIN);  
    float frequency = pulseCount / 2.0;  
    rpm = frequency * 600.0;  
    pulseCount = 0;
    lastTime = currentTime;
    attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachInterrupt, FALLING);
}
delay(10);
}

// Interrupción para contar pulsos
void IRAM_ATTR tachInterrupt() {
  pulseCount++;
}
