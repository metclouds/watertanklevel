#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266WebServer.h>
#include <ElegantOTA.h>

String VERSION = "2024081901";

ESP8266WebServer server(80);
unsigned long ota_progress_millis = 0;

//Variables for LED notifications
const int led = D0; // ESP8266 Pin to which onboard LED is connected
unsigned long previousLedMillis = 0;  // will store last time LED was updated
const long interval = 5000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED
//End

//Sensor Pins and buzzer pins
#define TRIG_PIN D5 // The ESP8266 pin connected to Ultrasonic Sensor's TRIG pin
#define ECHO_PIN D6 // The ESP8266 pin connected to Ultrasonic Sensor's ECHO pin
#define MOTORCONTROL_PIN D2
#define BUZZER_PIN D1
//End

//Variables for Buzzer notifications
unsigned long previousBuzzerMillis = 0;
const long buzzerOnInterval = 100;
const long buzzerOffInterval = 5000;
int buzzerState = LOW;
//End

//Wifi Login
const char* ssid1 = "";
const char* password1 = "";

const char* ssid2 = "";
const char* password2 = "";
//End

//APIURL
String APIURL   = "http://yourdomain.com/watertank/api.php?level="; 
String APIEventURL = "http://yourdomain.com/watertank/api.php?event=";
String APICommandURL = "http://yourdomain.com/watertank/api.php?command=";
//End

//Variables for ultrasonic sensor readings
float filterArray[30]; // array to store data samples from sensor
float distance = 0; // store the distance from sensor
float prevDistance = 0;
unsigned long previousLevelReadingMillis = 0;
unsigned long normalDelay = 58000;
float changeThresholdInLevelDuringNormalDelay = 5;
bool higherPrecisionMode = false;
unsigned long higherPrecisionDelay = 8000;
const int totalHigherPrecisionCount = 60;  //time of higherprecision = higherPrecisionDelay (in ms) x totalHigherPrecisionCount. Default 10 mins.
int higherPrecisionCount = 60;
float distanceLowRange = 5;
float distanceHighRange = 150;
bool incorrectSensorReadingDetected = false;
//End

//Variabled for motor control
int tankMaxLevel = 127;
int tankThresholdLowLevel = 65;
int tankThresholdHighLevel = 97;
bool waterLevelLow = false;
bool motorOnStatus = false;
unsigned long motorOnTime = 0;
unsigned long dryrunDetectionInterval = 60000;
float dryrunThresholdLevel = 2.5;
float previousDryRunDistance = 0;
unsigned long previousDryRunMillies = 0;
bool dryRunDetected = false;
//End

void onOTAStart()
{
  Serial.println("OTA update started!");
}

void onOTAProgress(size_t current, size_t final) 
{
  if (millis() - ota_progress_millis > 1000)
  {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success)
{
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
}

bool tryConnect(int maxAttempts) 
{
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < maxAttempts)
  {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void setup() 
{
  // Configure the trigger and echo pins to output mode
  pinMode(led, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(MOTORCONTROL_PIN, HIGH);
  pinMode(MOTORCONTROL_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  Serial.begin(9600);
  //booting
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid1, password1);
  String connectedSSID;
  if (tryConnect(10))
  {
    Serial.println("Connected to primary Wi-Fi " + String(ssid1));
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    connectedSSID = String(ssid1);
  }
  else
  {
    Serial.println("Failed to connect to primary Wi-Fi " + String(ssid1));

    // Try connecting to the secondary Wi-Fi
    Serial.println("Connecting to secondary Wi-Fi " + String(ssid2));
    WiFi.begin(ssid2, password2);
    
    // Wait for connection
    if (tryConnect(10))
    {
      Serial.println("Connected to secondary Wi-Fi " + String(ssid2));
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      connectedSSID = String(ssid2);
    }
    else
    {
      Serial.println("Failed to connect to secondary Wi-Fi " + String(ssid2));
      delay(10000);
      ESP.restart();
    }
  }

  postEventToServer("Device+On");
  postEventToServer("Version:" + VERSION);
  postEventToServer("SSID:" + connectedSSID);
  postEventToServer("IP:" + WiFi.localIP().toString());
  postEventToServer("Wifi+Strength:+" + String(WiFi.RSSI()) + "+dBm");

  server.on("/", []() {
    server.send(200, "text/plain", "Controller");
  });

  ElegantOTA.begin(&server);
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);
  server.begin();
}

void loop() 
{
  if(!motorOnStatus)
  {
    server.handleClient();
    ElegantOTA.loop();
  }
  
  //On or Off Led notification
  unsigned long currentLedMillis = millis();

  if(currentLedMillis - previousLedMillis >= interval) 
  {
    previousLedMillis = currentLedMillis;
    ledState = not(ledState);
    digitalWrite(led,  ledState);
  }

  //Get ultrasonic sensor reading
  if(isItTimeToTakeNextReading())
  {
    // 1. TAKING MULTIPLE MEASUREMENTS AND STORE IN AN ARRAY
    for(int sample = 0; sample < 30; sample++) 
    {
      filterArray[sample] = ultrasonicMeasure();
      delay(60); // to avoid untrasonic interfering
    }

    // 2. SORTING THE ARRAY IN ASCENDING ORDER
    for (int i = 0; i < 29; i++) 
    {
      for (int j = i + 1; j < 30; j++) 
      {
        if (filterArray[i] > filterArray[j]) 
        {
          float swap = filterArray[i];
          filterArray[i] = filterArray[j];
          filterArray[j] = swap;
        }
      }
    }

    //start of noise filtering
    double sum = 0;
    for (int sample = 10; sample < 20; sample++) 
    {
      sum += filterArray[sample];
    }

    distance = sum / 10;

    //Update value to server only if the distance is within the suitable range
    if(distance > distanceLowRange && distance < distanceHighRange)
    {
      incorrectSensorReadingDetected = false;

      if (WiFi.status() == WL_CONNECTED) 
      {
        WiFiClient client;
        HTTPClient http;
        
        String url = APIURL;
        url += distance;
        
        http.begin(client, url);
        http.GET();
        http.end();
      } 
      else
      {
        Serial.println("WiFi Disconnected");
      }
    }
    else
    {
      //sensor reading incorrect
      incorrectSensorReadingDetected = true;
      postEventToServer("Incorrect+sensor+reading");
      if(motorOnStatus)
      {
        motorOnStatus = false;
        motorOnTime = 0;
        digitalWrite(MOTORCONTROL_PIN, HIGH);
        postEventToServer("Motor+Off");
      }
    }

    previousLevelReadingMillis = millis();
  
    if(higherPrecisionMode == true)
    {
      //need to check if higher precision mode is still needed
      higherPrecisionCount--;
      if(higherPrecisionCount <= 0)
      {
        higherPrecisionMode = false;
        higherPrecisionCount = totalHigherPrecisionCount;
      }
    }
    else
    {
      //normal mode
      if(abs(distance-prevDistance) > changeThresholdInLevelDuringNormalDelay && prevDistance != 0)
      {
        higherPrecisionMode = true;
      }
    }

    if(!incorrectSensorReadingDetected)
      prevDistance = distance;
  }

  unsigned long currentBuzzerMillis = millis();
  if(isWaterLevelLow(distance))
  {
    if(buzzerState == LOW)
    {
      if(currentBuzzerMillis - previousBuzzerMillis >= buzzerOffInterval) 
      {
        previousBuzzerMillis = currentBuzzerMillis;
        buzzerState = HIGH;
        digitalWrite(BUZZER_PIN, buzzerState);
      }
    }
    else
    {
      //here buzzerState is HIGH
      if(currentBuzzerMillis - previousBuzzerMillis >= buzzerOnInterval)
      {
        previousBuzzerMillis = currentBuzzerMillis;
        buzzerState = LOW;
        digitalWrite(BUZZER_PIN, buzzerState);
      }
    }

    //Turn on motor if dry run is not previously detected
    if(!motorOnStatus && !dryRunDetected)
    {
      motorOnStatus = true;
      motorOnTime = millis();
      digitalWrite(MOTORCONTROL_PIN, LOW);
      previousDryRunMillies = millis();
      previousDryRunDistance = distance;
      postEventToServer("Motor+On");
      higherPrecisionMode = true;
    }
  }
  else
  {
    if(buzzerState == HIGH)
    {
      previousBuzzerMillis = currentBuzzerMillis = millis();
      buzzerState = LOW;
      digitalWrite(BUZZER_PIN, buzzerState);
    }
  }

  //turn motor off when water level is full
  if(isWaterLevelFull(distance) && motorOnStatus)
  {
    motorOnStatus = false;
    motorOnTime = 0;
    digitalWrite(MOTORCONTROL_PIN, HIGH);
    postEventToServer("Motor+Off");
    higherPrecisionMode = false;
    higherPrecisionCount = totalHigherPrecisionCount;
  }

  //turn motor off if dry run is detected
  if(motorOnStatus)
  {
    if(millis() - previousDryRunMillies > dryrunDetectionInterval)
    {
      if(abs(distance - previousDryRunDistance) < dryrunThresholdLevel)
      {
        //Dry run is happening
        dryRunDetected = true;
        motorOnStatus = false;
        motorOnTime = 0;
        digitalWrite(MOTORCONTROL_PIN, HIGH);
        postEventToServer("Dry+run+detected");
        postEventToServer("Motor+Off");
        higherPrecisionMode = false;
        higherPrecisionCount = totalHigherPrecisionCount;
      }

      previousDryRunMillies = millis();
      previousDryRunDistance = distance;
    }
  }

  //If motor is off and water is not full in tank, then check force_motoron status
  //force_motoron is 1, turn on motor
  if(!motorOnStatus && !isWaterLevelFull(distance))
  {
    WiFiClient client;
    HTTPClient http;
        
    String url = APICommandURL;
    url += "force_motoron";
        
    http.begin(client, url);
    int httpCode = http.GET();

    if(httpCode > 0) 
    {
      String payload = http.getString();
      int result = payload.toInt();
      if(result == 1)
      {
        motorOnStatus = true;
        motorOnTime = millis();
        digitalWrite(MOTORCONTROL_PIN, LOW);
        previousDryRunMillies = millis();
        previousDryRunDistance = distance;
        postEventToServer("Motor+On");
        higherPrecisionMode = true;

        //set force_motoron to 0
        String url = APICommandURL;
        url += "set&force_motoron=0";

        http.begin(client, url);
        http.GET();
      }
    }
    else 
    {
      Serial.println("Error on HTTP request");
    }
        
    http.end();
  }

  //If motor is on, then check force_motoroff status
  //force_motoroff is 1, turn off motor
  if(motorOnStatus)
  {
    WiFiClient client;
    HTTPClient http;
        
    String url = APICommandURL;
    url += "force_motoroff";
        
    http.begin(client, url);
    int httpCode = http.GET();

    if(httpCode > 0) 
    {
      String payload = http.getString();
      int result = payload.toInt();
      if(result == 1)
      {
        motorOnStatus = false;
        motorOnTime = 0;
        digitalWrite(MOTORCONTROL_PIN, HIGH);
        postEventToServer("Motor+Off");
        higherPrecisionMode = false;
        higherPrecisionCount = totalHigherPrecisionCount;

        //set force_motoron to 0
        String url = APICommandURL;
        url += "set&force_motoroff=0";

        http.begin(client, url);
        http.GET();
      }
    }
    else 
    {
      Serial.println("Error on HTTP request");
    }
        
    http.end();
  }
}

//Other Functions

float ultrasonicMeasure() {
  // Produce a 10-microsecond pulse to the TRIG pin.
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the pulse duration from the ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  float distance_cm = 0.017 * duration_us;

  return distance_cm;
}

bool isWaterLevelLow(float d)
{
  //in case of incorrect sensor reading, assume water level is high
  if(d < distanceLowRange || d > distanceHighRange)
    return false;

  if ( tankMaxLevel - d < tankThresholdLowLevel )
    return true;    
  else
    return false;
}

bool isWaterLevelFull(float d)
{
  //in case of incorrect sensor reading, assume water level is high
  if(d < distanceLowRange || d > distanceHighRange)
    return true;

  if(tankMaxLevel - d >= tankThresholdHighLevel)
    return true;
  else
    return false;
}

bool isItTimeToTakeNextReading()
{
  if(previousLevelReadingMillis == 0)
    return true;
  
  unsigned long currentLevelReadingMillis = millis();

  if(higherPrecisionMode == true)
  {
    if(currentLevelReadingMillis  - previousLevelReadingMillis > higherPrecisionDelay)
    {
      return true;
    }
  }
  else
  {
    if(currentLevelReadingMillis - previousLevelReadingMillis > normalDelay)
    {
      return true;
    }
  }

  return false;
}

void postEventToServer(String event)
{
  if (WiFi.status() == WL_CONNECTED) 
  {
    WiFiClient client;
    HTTPClient http;
    
    String url = APIEventURL  + event;

    http.begin(client, url);
    http.GET();
    http.end();
  } 
  else
  {
    Serial.println("WiFi Disconnected");
  }
}
