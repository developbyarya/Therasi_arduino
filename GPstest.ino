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
#define ssid "dinus"
#define password "110507be"
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

bool signupOK = false;

PulseOximeter pox;

uint32_t tsLastReport = 0, sendDataPrevMillis = 0, gpsDataPrevMillis = 0 ;

void onBeatDetected()
{
  // Serial.println("Beat!");
}

void setup() {
  Serial.begin(115200);
  digitalWrite(2, HIGH);
  gps_ss.begin(GPS_BAUD);
  // led mode
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(2, OUTPUT);


  // pox.begin();

  // wifi stuff
  WiFi.mode(WIFI_STA); //Optional
  WiFi.begin(ssid, password);
  Serial.println(F("\nConnecting"));

  while(WiFi.status() != WL_CONNECTED){
      Serial.print(F("."));
      delay(100);
  }

  Serial.println(F("\nConnected to the WiFi network"));
  digitalWrite(2, LOW);
  digitalWrite(4, HIGH);
  // MPU

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
   
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  

  // Serial.print(F("Initializing pulse oximeter.."));

    // Initialize the PulseOximeter instance
    // Failures are generally due to an improper I2C wiring, missing power supply
    // or wrong target chip

    // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    // pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);

    // Register a callback for the beat detection
    // pox.setOnBeatDetectedCallback(onBeatDetected);
  // Firebase
  /* Assign the api key (required) */
  config.api_key = API_KEY;

  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  /* Sign up */
  if (Firebase.signUp(&config, &auth, "", "")){
    Serial.println("Firebase ok");
    signupOK = true;
    digitalWrite(4, LOW);
    digitalWrite(5, HIGH);

  }
  else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }

  /* Assign the callback function for the long running token generation task */
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  
  if (!pox.begin()) 
    digitalWrite(2, HIGH);
  else  Serial.println("PulseOximeter SUCCESS!");

  // pox.setOnBeatDetectedCallback(onBeatDetected);

  digitalWrite(32, HIGH);


}

float bpm_total =0;
int bpm_recorded=0, oxigen_total=0, oxigen_recorded=0;


void loop(){
  

  // MPU
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  uint8_t x = (a.acceleration.x);
  uint8_t y = (a.acceleration.y);
  uint8_t z = (a.acceleration.z);

  uint8_t temp_res = (temp.temperature);

  pox.update();


  if (millis() - tsLastReport > REPORTING_PERIOD_MS) {
      // Serial.println(pox.getHeartRate());
      // Serial.println(pox.getSpO2());
      bpm_total +=  (pox.getHeartRate());
      oxigen_total += (pox.getSpO2());
      bpm_recorded++;
      oxigen_recorded++;
      tsLastReport = millis();
  }

  while (gps_ss.available() > 0){
    if (Firebase.ready() && signupOK && gps.encode(gps_ss.read()) && (millis() - gpsDataPrevMillis > 5000 || gpsDataPrevMillis == 0) ){
      gpsDataPrevMillis = millis();
      
          if (gps.location.isValid()){
            double lat, lng;
            lat = gps.location.lat();
            lng = gps.location.lng();
            
            if (lat && Firebase.setFloat(fbdo, "lokasi/lat", double(lat))){
              Serial.println(lat);
            }else {
              Serial.println("FAILED, GPS lat");
              Serial.println("REASON: " + fbdo.errorReason());
            }
            if (lng && Firebase.setFloat(fbdo, "lokasi/long", double(lng)))
              {
                Serial.println("LOKASI TERUPLOAd, long");
              }
            else {
              Serial.println("FAILED, GPS long");
              Serial.println("REASON: " + fbdo.errorReason());
            }
            Firebase.setBool(fbdo, "lokasi/recorded", false);
           
            
          }
         
    }
  }
  
  if (Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)){
    digitalWrite(13, LOW);
    
    sendDataPrevMillis = millis();

    if (Firebase.setInt(fbdo, "acl/x", x))
      {
        Serial.println("MPU BERHASIL, x");
      }else {
      Serial.println("FAILED, MPU X");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if ( Firebase.setInt(fbdo, "acl/y", y))
      {
        Serial.println("MPU BERHASIL, y");
      }else {
      Serial.println("FAILED. MPU Y");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (Firebase.setInt(fbdo, "acl/z", z))
      {
        Serial.println("MPU BERHASIL,z ");
      }else {
      Serial.println("FAILED, MPU Z");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    if (temp.temperature && Firebase.setInt(fbdo, "acl/temp", temp.temperature))
      {
        Serial.println("MPU BERHASIL, temp: ");
        Serial.println(temp.temperature);
      }else {
      Serial.println("FAILED, MPU Temp");
      Serial.println("REASON: " + fbdo.errorReason());
    }

    if (bpm_total && Firebase.setFloat(fbdo, "pulse_oximeter/bpm", float(bpm_total/bpm_recorded)))
      {
        Serial.println("MAX AMAN! herat rate: ");
        Serial.println(bpm_total/bpm_recorded);
        bpm_total=0;
        bpm_recorded=0;
        Firebase.setBool(fbdo, "pulse_oximeter/recorded", false);
      }else {
      Serial.println("FAILED, MAX Heart");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    
    if (oxigen_total && Firebase.setInt(fbdo, "pulse_oximeter/oxigen", int(oxigen_total/oxigen_recorded)))
      {
        // Serial.println("MAX SpO2");
        oxigen_total=0;
        oxigen_recorded=0;
        Firebase.setBool(fbdo, "pulse_oximeter/recorded", false);
        // Serial.println(x);
      }else {
      Serial.println("FAILED, MAX Sp02");
      Serial.println("REASON: " + fbdo.errorReason());
    }
    // delay(200);
    digitalWrite(13, HIGH);
    
    int co = analogRead(14);
    if (Firebase.setInt(fbdo, "co/data", co)){
      Firebase.setBool(fbdo, "co/recorded", false);
    }
)

    
  }




}

