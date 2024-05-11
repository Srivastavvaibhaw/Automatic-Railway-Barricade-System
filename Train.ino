//Necessary libraries
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <Math.h>

#define NUMBER "+919294981607"
#define distanceThreshold 500 // Replace with your desired distance

// Define software serial pins for GPS and GSM modules
SoftwareSerial gpsSerial(2, 3); // RX, TX pins for GPS
SoftwareSerial gsmSerial(7, 8); // RX, TX pins for GSM

// Create TinyGPS++ object
TinyGPSPlus gps;

// Define the target latitude and longitude
const double targetLat = 28.4671372; // Replace with your desired location
const double targetLon = 77.4632543;

// Define the distance threshold (in meters)


// Earth's radius in meters
const double earthRadius = 6371000;

// Function to calculate distance between two GPS coordinates using the Haversine formula
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = (lat2 - lat1) * M_PI / 180.0;
  double dLon = (lon2 - lon1) * M_PI / 180.0;
  double a = sin(dLat / 2) * sin(dLat / 2) +
             cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
             sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return earthRadius * c;
}

void setup() {
  Serial.begin(9600); // Initialize serial communication
  gpsSerial.begin(9600); // Initialize software serial for GPS
  gsmSerial.begin(9600); // Initialize software serial for GSM

  // Configure GSM module
  gsmSerial.println("AT+CMGF=1"); // Set SMS mode to text
  delay(1000);
  Serial.println("Initializing GPS...");
}

void loop() {
  // Read data from GPS module
  while (gpsSerial.available()) {
    if (gps.encode(gpsSerial.read())) {
      double lat = gps.location.lat(); // Get current latitude
      double lon = gps.location.lng(); // Get current longitude

      // Calculate distance from target location using the Haversine formula
      double distance = calculateDistance(lat, lon, targetLat, targetLon);

      // Check if distance is within the threshold
      if (distance <= distanceThreshold) {
        // Call the control room using GSM module
        gsmSerial.println("ATD+919294981607;"); // Replace with control room phone number
        delay(5000); // Wait for call to connect
        gsmSerial.println("ATH"); // Hang up the call
        delay(1000);
      }
    }
    else // If no valid GPS data is received
    {
        Serial.println("Waiting for GPS data...");
        delay(1000); // Wait for 1 second before trying again
    }
  }
}
