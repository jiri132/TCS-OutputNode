#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <FastLED.h>

#define DEBUG 1


#define NUM_LEDS 1               // Number of LEDs per traffic light (project: 3)
#define NUM_TRAFFIC_LIGHTS 18     // Number of traffic lights (project: 18)

#define DATA_PIN1 4 // Testing DATA_PIN more are in official project

#define COLOR_OFF     CRGB::Black
#define COLOR_GREEN   CRGB::Green
#define COLOR_ORANGE  CRGB::Orange
#define COLOR_RED     CRGB::Red


typedef struct TrafficLight {
  uint8_t status;
  uint8_t timeInactive;
};


typedef struct TrafficControl {
  uint64_t bitField; // 36 bits (2 bits / traffic light) (28 unused bits) (00 = Off, 01 = Green 10 = Orange, 11 = Red)
  uint16_t cycle;
  TrafficLight trafficLights[NUM_TRAFFIC_LIGHTS];
};

TrafficControl controller;


CRGB leds[NUM_TRAFFIC_LIGHTS][NUM_LEDS];

// Wi-Fi credentials
const char* ssid = "TCS-net";           // Replace with your WiFi SSID
const char* password = "tcs-pass24";   // Replace with your WiFi password

// API endpoint
const char* serverUrl = "http://192.168.4.1:3000/int"; // Replace with your API URL

// Macro Definitions
#define SETUP_TRAFFIC_LIGHT(INDEX, PIN) FastLED.addLeds<NEOPIXEL, PIN>(leds[INDEX], NUM_LEDS);

// Function Definitions
void connectWifi(const char* ssid,const char* password);
void initTrafficController(TrafficControl* control);
void decodeTrafficLights(TrafficControl *control);
void setTrafficLight(TrafficLight light, uint8_t status);
void displayTrafficLight(TrafficLight light, CRGB* led);



#pragma region Setup & Loop 
void setup() {
  Serial.begin(115200);                // Start serial communication

  connectWifi(ssid,password);
  initTrafficController(&controller);

  pinMode(DATA_PIN1, OUTPUT); // Debug  

  for (int i = 0; i < NUM_TRAFFIC_LIGHTS; i++) {
    SETUP_TRAFFIC_LIGHT(i,DATA_PIN1);    
  }
}

void loop() {
  if (WiFi.status() == WL_CONNECTED) {     // Check Wi-Fi connection
        HTTPClient http;                     // Create an HTTPClient object
        http.begin(serverUrl);               // Specify the API endpoint
        int httpResponseCode = http.GET();   // Send the request

        if (httpResponseCode > 0) {          // Check for a successful response
            String response = http.getString(); // Get the response payload
            Serial.println("Response Code: " + String(httpResponseCode));
            Serial.println("Response: " + response);
            uint64_t data = strtoull(response.c_str(), NULL, 10); // StringToULonLong (uint64_t)
            controller.bitField = data;

            decodeTrafficLights(&controller);
            FastLED.show();  // Refresh the LEDs with new colors

        } else {
            Serial.println("Error on HTTP request");
        }

        http.end();                          // Close connection
    } else {
        Serial.println("WiFi Disconnected");
    }
    delay(10000); // Wait for 10 seconds before the next request
}
#pragma endregion

#pragma region Wifi Functions
void connectWifi(const char* ssid,const char* password) {
  WiFi.begin(ssid, password);          // Connect to Wi-Fi network

  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {  // Wait for connection
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
}
#pragma endregion

#pragma region Traffic Functions
void initTrafficController(TrafficControl* control) {
    control->cycle = 0;
    control->bitField = 0;

    // Initialize each traffic light and populate the map
    for (int i = 0; i < NUM_TRAFFIC_LIGHTS; i++) {
        control->trafficLights[i].status = 0;
        control->trafficLights[i].timeInactive = 0;
    }
}


void decodeTrafficLights(TrafficControl *control) {
  control->cycle++;
  for (int i = 0; i < NUM_TRAFFIC_LIGHTS; i++) {
  // Extract 2 bits for the current traffic light's status
    uint8_t status = (control->bitField >> (i * 2)) & 0b11;

    setTrafficLight(control->trafficLights[i], status);
    displayTrafficLight(control->trafficLights[i],leds[i]);
  }
}

void setTrafficLight(TrafficLight light, uint8_t status) {
  if (light.status != status) {
    light.status = status;
    light.timeInactive = 0;
    #if DEBUG == 1
    Serial.println("Status: " + String(light.status));
    #endif
  } else {
    light.timeInactive++;
    #if DEBUG == 1
    Serial.println("Inactive");
    #endif
  }
}

void displayTrafficLight(TrafficLight light, CRGB* led) {
  CRGB color;

  // Determine the color based on the traffic light's status
  switch (light.status) {
    case 0b00: color = COLOR_OFF; break;     // Off
    case 0b01: color = COLOR_GREEN; break;   // Green
    case 0b10: color = COLOR_ORANGE; break;  // Orange
    case 0b11: color = COLOR_RED; break;     // Red
    default: color = COLOR_OFF; break;       // Default to Off for undefined states
  }

  // Return the function since it is off
  if (color == COLOR_OFF) {return;}

  // L[s-1] = correct led index (11 = 2, 10 = 1, 01 = 0, 00 = Non existent)
  led[light.status-1] = color;
}
#pragma endregion