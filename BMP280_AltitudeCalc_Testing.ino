#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp;
float rawPressure = 0;
float filteredPressure = 0;
float alpha = 0.2;  // Smoothing factor for the low-pass filter

unsigned long lastTime = 0;
unsigned long currentTime = 0;
const unsigned long interval = 100;

const int calibrationSamples = 50;
float groundPressure = 0;  // Calibrated ground pressure in Pascals

void setup() {
  Serial.begin(9600);

  if (!bmp.begin()) {
    Serial.println("Check bmp wiring");
    while (1);
  }

  // Configure BMP280 with high oversampling and filtering
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X1,   // Temperature oversampling
                  Adafruit_BMP280::SAMPLING_X8,   // Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,    // Internal filtering
                  Adafruit_BMP280::STANDBY_MS_1); // Standby time

  // Initialize the pressure readings
  rawPressure = bmp.readPressure();
  filteredPressure = rawPressure;

  // Calibrate the ground pressure at the start
  calibrateBMP();

  Serial.print("Calibrated Ground Pressure (Pa): ");
  Serial.println(groundPressure);
}

void loop() {
  currentTime = millis();

  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    // Read raw pressure from BMP280
    rawPressure = bmp.readPressure();

    // Apply the low-pass filter to smooth the pressure
    filteredPressure = alpha * rawPressure + (1 - alpha) * filteredPressure;

    // Use the filtered pressure to calculate altitude relative to ground pressure
    float alt = calculateAltitude(filteredPressure, groundPressure);

    // Output the filtered altitude
    Serial.print("Altitude (m): ");
    Serial.println(alt);
  }
}

// Function to calculate altitude using the barometric formula
float calculateAltitude(float currentPressure, float baselinePressure) {
  // Convert pressure from Pascals to hPa for altitude calculation
  float pressure_hPa = currentPressure / 100.0;
  float baseline_hPa = baselinePressure / 100.0;

  // Use the barometric formula to calculate altitude relative to the baseline
  float altitude = 44330.0 * (1.0 - pow(pressure_hPa / baseline_hPa, 0.1903));

  return altitude;
}

// Function to calibrate the ground pressure at the start (average over several readings)
void calibrateBMP() {
  float sum = 0;
  for (int i = 0; i < calibrationSamples; i++) {
    sum += bmp.readPressure();
    delay(100);
  }
  // Set the ground pressure as the average of the readings
  groundPressure = sum / calibrationSamples;
}
