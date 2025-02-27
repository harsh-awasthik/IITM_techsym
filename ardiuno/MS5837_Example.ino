
#include <Wire.h>
#include <MS5837.h> // Bar30 sensor library

MS5837 sensor;

void setup() {
  Serial.begin(9600); // For communication with Xavier
  Wire.begin();
  
  if (!sensor.init()) {
    Serial.println("Sensor initialization failed!");
    while (1);
  }

  sensor.setFluidDensity(997); // Set to 1029 for saltwater
}

void loop() {
  sensor.read();
  float pressure = sensor.pressure(); // Pressure in mbar
  float temperature = sensor.temperature(); // Temperature in Celsius

  // Convert pressure to Pascals
  float pressurePa = pressure * 100.0;
  
  // Calculate depth (assuming surface pressure is 101325 Pa)
  float depth = ((pressurePa - 101325) / (997 * 9.81))+0.15; // Adjust density for saltwater if needed

  // Send depth to Xavier
  Serial.println(depth*100);

  delay(500); // Send data every 500ms
}
