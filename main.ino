// ========================================
// Libraries
// ========================================
#include "DHT.h"
#include "Arduino_LED_Matrix.h"
#include <LiquidCrystal.h>
#include <string>
#include <Wire.h>

#include <Adafruit_GFX.h>     // Core graphics library
#include <Adafruit_ST7735.h>  // Hardware-specific library
#include <SPI.h>

#include <ArduinoJson.h>

#include "WiFiS3.h"
#include "HttpClient.h"
#include "arduino_secrets.h"

// ========================================
// Definitions
// ========================================

// TFT Pins
#define TFT_CS 10
#define TFT_RST 9  // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC 8
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// TFT Pane colors
uint16_t bigPaneColor = ST77XX_WHITE;
uint16_t smallLeftPaneColor = ST77XX_WHITE;
uint16_t smallRightPaneColor = ST77XX_WHITE;

// DHT Pins
#define DHTPIN 2
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);  // dht sensor initialization

// Color definitions
#define GREEN 0x3440
#define YELLOW 0x0477
#define RED 0x0019

uint16_t color = ST77XX_WHITE;  // standard color - value used to check for changes

// button Pin
#define BUTTON_PIN 5
bool buttonState = 0;  // button state
bool mode = 0;         // current mode -> sensor mode or cloud mode
#define MODE_SENSOR 0
#define MODE_CLOUD 1

// Arduino LED Matrix initialization and frame definitions
byte frame_on[8][12] = {
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 },
};

byte frame_off[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
};
ArduinoLEDMatrix matrix;

bool blink = false;  // matrix blinker
bool isOn = false;   // matrix state

// ========================================
// Variables for time messuring
// ========================================
#define DELAY_SENSOR_MESSUREMENT 1000;
unsigned long start_delayMessurement;

#define DELAY_LED_MATRIX 333
unsigned long start_delayLedMatrix = 0;

#define DELAY_SERVER_READING 10000
unsigned long start_delayServerReading = 0;

#define DELAY_TFT_PRINT 1000
unsigned long start_delayTftPrint = 0;

unsigned long currentMillis = 0;

// ========================================
// Variables for WiFi
// ========================================
char ssid[] = SECRET_SSID;                                // SSID for network - using arduino_secrets.h
char pass[] = SECRET_PASS;                                // Password for network - using arduino_secrets.h
int status = WL_IDLE_STATUS;                              // wifi status
char server[] = "www.api.smarthome.temperature.henk.pm";  // backend api server
WiFiClient client;                                        // using wifi client - no ssl client
HttpClient http = HttpClient(client, server, 80);

String server_city = "";
float server_temp = 0.0f;
int server_hum = 0;
String server_weather = "";

// ========================================
// TFT Functions
// ========================================

/*
 * Print the temperature, humidity and air quality to the TFT display
 * @param temperature - the temperature to print
 * @param humidity - the humidity to print
 * @param quality - the air quality to print
*/
void printToTft(float temperature, float humidity, int quality) {
  tftDrawBigPane(temperature, changeColor(static_cast<int>(temperature), 22, 32));
  tftDrawSmallLeftPane(humidity, changeColor(static_cast<int>(humidity), 40, 60));
  tftDrawSmallRightPane(quality, changeColor(quality, 30, 50));
}

void printToTft(float temperature, float humidity, String weather) {
  tftDrawBigPane(temperature, changeColor(static_cast<int>(temperature), 22, 32));
  tftDrawSmallLeftPane(humidity, changeColor(static_cast<int>(humidity), 40, 60));
  tftDrawSmallRightPane(weather, ST7735_BLACK);
}

/*
 * Setup the TFT display with according lines
 */
void tftSetup() {
  tft.drawFastHLine(0, 79, 128, ST77XX_WHITE);
  tft.drawFastHLine(0, 80, 128, ST77XX_WHITE);
  tft.drawFastVLine(63, 80, 80, ST77XX_WHITE);
  tft.drawFastVLine(64, 80, 80, ST77XX_WHITE);
}

/*
 * Draw the big pane on the TFT display
 * @param value - the value to print
 * @param color - the color of the pane
 */
void tftDrawBigPane(float value, uint16_t color) {
  if (bigPaneColor != color) {
    bigPaneColor = color;
    tft.fillRect(0, 0, 128, 79, bigPaneColor);
  }

  tft.setCursor(0, 0);
  tft.setTextColor(ST77XX_BLACK, ST77XX_WHITE);
  tft.print("Mode: ");
  tft.print(getMode(mode));

  tft.setTextColor(ST77XX_WHITE, bigPaneColor);
  tft.setCursor(30, 10);
  tft.setTextSize(1);
  tft.println("Temperature:");
  tft.setTextSize(4);
  tft.setCursor(20, 30);
  tft.print(static_cast<int>(value));
  tft.write(0xF7);
  tft.println("C");
}

/*
 * Draw the small left pane on the TFT display
 * @param value - the value to print
 * @param color - the color of the pane
 */
void tftDrawSmallLeftPane(float value, uint16_t color) {
  if (smallLeftPaneColor != color) {
    smallLeftPaneColor = color;
    tft.fillRect(0, 81, 63, 78, smallLeftPaneColor);
  }

  tft.setTextColor(ST77XX_WHITE, smallLeftPaneColor);
  tft.setCursor(5, 90);
  tft.setTextSize(1);
  tft.println("Humidity:");
  tft.setCursor(5, 115);
  tft.setTextSize(3);
  tft.print(static_cast<int>(value));
  tft.println("%");
}

/*
 * Draw the small right pane on the TFT display
 * @param value - the value to print
 * @param color - the color of the pane
 */
void tftDrawSmallRightPane(int value, uint16_t color) {
  if (smallRightPaneColor != color) {  // check if the color has changed
    smallRightPaneColor = color;
    tft.fillRect(65, 81, 63, 78, color);
  }

  tft.setTextColor(ST77XX_WHITE, color);
  tft.setCursor(73, 90);
  tft.setTextSize(1);
  tft.println("Quality:");
  tft.setCursor(70, 115);
  tft.setTextSize(3);
  tft.print(value);
  tft.setTextSize(1);
  tft.println("ppm");
}

/*
 * Draw the small right pane on the TFT display
 * @param value - the value to print
 * @param color - the color of the pane
 */
void tftDrawSmallRightPane(String value, uint16_t color) {
  if (smallRightPaneColor != color) {  // check if the color has changed
    smallRightPaneColor = color;
    tft.fillRect(65, 81, 63, 78, smallRightPaneColor);
  }

  tft.setTextColor(ST77XX_WHITE, smallRightPaneColor);
  tft.setCursor(73, 90);
  tft.setTextSize(1);
  tft.println("Weather");
  tft.setCursor(75, 115);
  tft.setTextSize(1);
  tft.print(value);
}

/*
 * Draw the status screen on the TFT display
 */
void tftStatusScreen() {
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.println("Startup");
  tft.println();
  tft.setTextSize(1);
  tft.println("connection status:");
  tft.setTextSize(2);
  switch(status){
    case WL_IDLE_STATUS:
      tft.setTextColor(ST77XX_WHITE);
      tft.println("IDLE");
      break;
    case WL_CONNECTED:
      tft.setTextColor(ST77XX_GREEN);
      tft.println("CONNECTED");
      break;
    case WL_CONNECT_FAILED:
      tft.setTextColor(ST77XX_RED);
      tft.println("FAILED");
      break;
    case WL_CONNECTION_LOST:
      tft.setTextColor(ST77XX_RED);
      tft.println("LOST");
      break;
  }

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.println();
  tft.println("SSID: ");
  tft.setTextSize(2);
  tft.print(ssid);

  tft.setTextSize(1);
}

/*
 * Change the color of the pane depending on the value
 * @param value - the value to check
 * @param middleBarrier - the middle barrier for the color change
 * @param highBarrier - the high barrier for the color change
 */
uint16_t changeColor(int value, int middleBarrier, int highBarrier) {
  if (middleBarrier <= value && highBarrier >= value) {
    return YELLOW;
  } else if (highBarrier <= value) {
    return RED;
  }

  return GREEN;
}

// ========================================
// Matrix Functions
// ========================================

/* 
 * Toggle the matrix on and off
 */
void toggleMatrix() {
  if (!isOn) {
    matrix.renderBitmap(frame_on, 8, 12);
    isOn = true;
  } else {
    matrix.renderBitmap(frame_off, 8, 12);
    isOn = false;
  }
}

/*
 * Blink the matrix
 */
void matrixBlinker() {
  if (blink) {
    if (isDelayOver(start_delayLedMatrix, 333)) {
      toggleMatrix();
    }
  }
  if (!blink && isOn) {
    toggleMatrix();
  }
}

// ========================================
// Messurement Functions
// ========================================
void massureAndPrint() {
  if (!isDelayOver(start_delayMessurement, 1000)) {  // check if the delay is over to messure
    return;
  }

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int quality = analogRead(A0);
  tftSetup();
  printToTft(temperature, humidity, quality);
  if (quality >= 50) {
    blink = true;
  } else {
    blink = false;
  }
}

// ========================================
// WiFi Functions
// ========================================

/*
 * Read the response from the server and print to serial
 */
void readAndPrintFromServer() {
  if (!isDelayOver(start_delayTftPrint, 1000)) {  // check if the delay is over to messure
    return;
  }

  if (isDelayOver(start_delayServerReading, 120000)) {  // check if the delay is over to messure
    setServerReading();
    Serial.println("reading");
  }

  printToTft(server_temp, server_hum, server_weather);
}

/*
 * Print the wifi status to serial
 */
void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");
}

// ========================================
// Util functions
// ========================================

String getMode(bool mode) {
  if (mode == MODE_SENSOR) {
    return "inside";
  }

  return "outside";
}

JsonDocument deserializeJsonResponse(String response) {
  JsonDocument doc;
  deserializeJson(doc, response);
  return doc;
}

void setServerReading() {
  http.get("/get-weather?lat=52.20&lon=8.04");
  JsonDocument doc = deserializeJsonResponse(http.responseBody());
  server_city = static_cast<String>(doc["city"]);
  server_temp = doc["temperature"];
  server_hum = doc["humidity"];
  server_weather = static_cast<String>(doc["weather"]);
}

boolean isDelayOver (unsigned long &startOfPeriod, unsigned long TimePeriod) {
  unsigned long currentMillis  = millis();
  if ( currentMillis - startOfPeriod >= TimePeriod ) {
    // more time than TimePeriod has elapsed since last time if-condition was true
    startOfPeriod = currentMillis; // a new period starts right here so set new starttime
    return true;
  }
  else return false;            // actual TimePeriod is NOT yet over
}

void resetColors() {
  // reset colors
  color = ST77XX_BLUE;
  bigPaneColor = ST77XX_WHITE;
  smallLeftPaneColor = ST77XX_WHITE;
  smallRightPaneColor = ST77XX_WHITE;
}

// ========================================
// Setup and Loop
// ========================================
void setup() {
  Serial.begin(9600);  // Serielle Verbindung starten
  matrix.begin();

  tft.initR(INITR_GREENTAB);
  tft.fillScreen(ST77XX_BLACK);

  tftSetup();
  dht.begin();  // DHT11 Sensor starten

  pinMode(A0, INPUT);  // analog Air Quality Sensor

  tftStatusScreen();

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);
    // wait 10 seconds for connection:
    delay(10000);
  }

  printWifiStatus();

  tftStatusScreen();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  setServerReading();
}

void loop() {
  // read button input -> Switch mode when pressed
  if (!digitalRead(BUTTON_PIN)) {
    if (!buttonState) {
      buttonState = true;
      mode = !mode;

      resetColors();
    }
  } else {
    buttonState = false;
  }

  if (!mode) {
    massureAndPrint();
  } else {
    readAndPrintFromServer();
  }
}