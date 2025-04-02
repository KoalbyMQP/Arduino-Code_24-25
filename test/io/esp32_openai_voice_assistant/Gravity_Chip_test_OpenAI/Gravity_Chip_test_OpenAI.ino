#include <WiFi.h>
#include <HTTPClient.h>
#include <DFRobot_DF2301Q.h>
#include <Wire.h>
#include <ArduinoJson.h>

const char* ssid = "";
const char* password = "";
const char* openai_api_key = "";


const char* openai_endpoint = "https://api.openai.com/v1/chat/completions";


#define HELLO_ROBOT_ID 2  
DFRobot_DF2301Q_I2C asr;

enum State {
  IDLE,
  ACTIVE
};

State currentState = IDLE;

void setup() {
  Serial.begin(115200);
  delay(1000);

  // WiFi connection
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");

  while (!asr.begin()) {
    Serial.println("DF2301Q not detected. Check connections.");
    delay(3000);
  }

  asr.setVolume(4);        
  asr.setMuteMode(0);      
  asr.setWakeTime(20);     
  Serial.println("Voice module initialized!");
}

void loop() {
  uint8_t commandID = asr.getCMDID();

  if (commandID == 0) return;

  // Wake Word Detected
  if (currentState == IDLE && commandID == HELLO_ROBOT_ID) {
    Serial.println("Wake word detected: Hello robot");
    currentState = ACTIVE;

    String prompt = "Tell me a fun fact!";
    String reply = askOpenAI(prompt);
    Serial.println("OpenAI says:\n" + reply);

    currentState = IDLE;
  }

  delay(100);
}

String askOpenAI(String prompt) {
  if (WiFi.status() != WL_CONNECTED) {
    return "WiFi not connected.";
  }

  HTTPClient http;
  http.begin(openai_endpoint);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + String(openai_api_key));

  // Prepare JSON request
  StaticJsonDocument<1024> jsonRequest;
  jsonRequest["model"] = "gpt-3.5-turbo";

  JsonArray messages = jsonRequest.createNestedArray("messages");
  JsonObject userMessage = messages.createNestedObject();
  userMessage["role"] = "user";
  userMessage["content"] = prompt;

  String requestBody;
  serializeJson(jsonRequest, requestBody);

  int httpCode = http.POST(requestBody);

  if (httpCode > 0) {
    String response = http.getString();
    StaticJsonDocument<2048> jsonResponse;
    DeserializationError error = deserializeJson(jsonResponse, response);

    if (!error) {
      String reply = jsonResponse["choices"][0]["message"]["content"];
      http.end();
      return reply;
    } else {
      http.end();
      return "Failed to parse OpenAI response.";
    }
  } else {
    http.end();
    return "HTTP Error: " + String(httpCode);
  }
}
