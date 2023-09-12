#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
//Provide the token generation process info.
#include "addons/TokenHelper.h"
//Provide the RTDB payload printing info and other helper functions.
#include "addons/RTDBHelper.h"

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

#define REPORTING_PERIOD_MS 1000

#define API_KEY "AIzaSyBeTRTDlY0yfRG9dRA93hx6xZQU9SXnbMk"
#define DATABASE_URL "https://therasi-1cc8d-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define ssid "LAB FISIKA"
#define password "fisika7YK"
#define GPS_RX 18
#define GPS_TX 19
#define GPS_BAUD 9600

// GPS 
SoftwareSerial gps_ss(GPS_RX, GPS_TX);
TinyGPSPlus gps;
Adafruit_MPU6050 mpu;


//Define Firebase Data object
FirebaseData fbdo;

FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

PulseOximeter pox;

uint32_t tsLastReport = 0;

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
    Serial.println("Beat!");
}

void setup() {
  Serial.begin(115200);
  gps_ss.begin(GPS_BAUD);
  // wifi stuff
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println("\nConnecting");

  while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(100);
  }

  Serial.println("\nConnected to the WiFi network");
  Serial.print("Local ESP32 IP: ");
  Serial.println(WiFi.localIP());

  // MPU

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
   
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  

  Serial.print("Initializing pulse oximeter..");

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip
    if (!pox.begin()) {
        Serial.println(F("FAILED"));
        for(;;);
    } else {
        Serial.println(F("SUCCESS"));
    }

    // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback for the beat detection
    pox.setOnBeatDetectedCallback(onBeatDetected);

  // Firebase
  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("Firebase ok");
    signupOK = true;
  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);

}



void loop(){
  double lat, lng;
  char gpstime[30];
  if (gps_ss.available() > 0){
      if (gps.encode(gps_ss.read())) {
        if (gps.location.isValid()){
          lat = gps.location.lat();
          lng = gps.location.lng();
          if (gps.time.isUpdated() && gps.time.isValid())
          {

// Format the datetime string using sprintf
          sprintf(gpstime, "%02d:%02d:%02d %04d-%02d-%02d",
                  gps.time.hour(), gps.time.minute(), gps.time.second(),
                  gps.date.year(), gps.date.month(), gps.date.day());
                    }
        }
      }
  }

  // MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  
  uint8_t x = (a.acceleration.x);
  uint8_t y = (a.acceleration.y);
  uint8_t z = (a.acceleration.z);

  uint8_t temp_res = (temp.temperature);

  pox.update();

    // Asynchronously dump heart rate and oxidation levels to the serial
    // For both, a value of 0 means "invalid"
  float heart_rate;
  int sp02;

  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      
      heart_rate =  (pox.getHeartRate());
      sp02 = (pox.getSpO2());

      tsLastReport = millis();
  }
  
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();

    if (lat && Firebase.RTDB.setFloat(&fbdo, "lokasi/lat", double(lat)))
      {
        Serial.println("LOKASI TERUPLOAd, lat");
      }else {
      Serial.println("FAILED, GPS lat");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (lng && Firebase.RTDB.setFloat(&fbdo, "lokasi/long", double(lng)))
      {
        Serial.println("LOKASI TERUPLOAd, long");
      }
    else {
      Serial.println("FAILED, GPS long");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (gpstime && Firebase.RTDB.setString(&fbdo, "lokasi/timestamp", gpstime))
      {
        Serial.println("LOKASI TERUPLOAd, time");
        Serial.println(lng);
      }else {
      Serial.println("FAILED, GPS time");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (x && Firebase.RTDB.setInt(&fbdo, "acl/x", x))
      {
        Serial.println("MPU BERHASIL, x");
      }else {
      Serial.println("FAILED, MPU X");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (y && Firebase.RTDB.setInt(&fbdo, "acl/y", y))
      {
        Serial.println("MPU BERHASIL, y");
      }else {
      Serial.println("FAILED. MPU Y");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (z && Firebase.RTDB.setInt(&fbdo, "acl/z", z))
      {
        Serial.println("MPU BERHASIL,z ");
      }else {
      Serial.println("FAILED, MPU Z");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (temp_res && Firebase.RTDB.setInt(&fbdo, "acl/temp", temp_res))
      {
        Serial.println("MPU BERHASIL, temp");
        Serial.println(x);
      }else {
      Serial.println("FAILED, MPU Temp");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (heart_rate && Firebase.RTDB.setFloat(&fbdo, "pulse_oxi/heart-rate", heart_rate))
      {
        Serial.println("MAX AMAN! herat rate");
      }else {
      Serial.println("FAILED, MAX Heart");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    if (sp02 && Firebase.RTDB.setInt(&fbdo, "pulse_oxi/sp02", sp02))
      {
        Serial.println("MAX SpO2");
        Serial.println(x);
      }else {
      Serial.println("FAILED, MAX Sp02");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
  }
}


