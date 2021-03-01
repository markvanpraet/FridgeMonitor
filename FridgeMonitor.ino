#include <EMailSender.h>              // alert to email/sms
#include <OneWire.h>                  // temperature sensor bus management
#include <DallasTemperature.h>        // temperature sensor management
#include "pitches.h"                  // list of misical pitches by note
#include "config.h"                   // Adafruit IO configuration for IoT
//
// Temperature sensor setup
//
#define ONE_WIRE_BUS 4              // Temp sensor data wire is plugged into board (ESP8266 board show D2 but actually maps to GPIO 4)
#define TEMPERATURE_PRECISION 10    // Temperature precision to 10 bits (1/4 degree C)
OneWire oneWire(ONE_WIRE_BUS);      // setup oneWire instance to communicate with Dallas temperature style sensor 
DallasTemperature sensors(&oneWire);  // pass oneWire reference to Dallas Temperature.
// Addresses of DS18B20s
uint8_t freezerSensor[8] = { 0x28, 0x6C, 0xFF, 0x07, 0xB6, 0x01, 0x3C, 0x5B };
uint8_t fridgeSensor[8] = { 0x28, 0x4F, 0x89, 0x07, 0xB6, 0x01, 0x3C, 0x45 };

//
// general use variables
//
int lightPin = A0;                  // analog interface 0 connect the photosensitive resistance
int tonePin = 14;                    // passive buzzer pin
long unsigned int lastTempCheck;    // milliseconds since last temp sensor check
long unsigned int lastFreezerCheck;    // milliseconds since last temp sensor check
long unsigned int lastLightCheck;   // milliseconds since last photosensor check
long unsigned int lightOnTime;      // milliseconds since light alert was triggered
boolean tempAlertState = false;     // set when the temp is currently exceeding the threshold
boolean freezerAlertState = false;     // set when the temp is currently exceeding the threshold
boolean lightAlertState = false;    // flag set when the photosensor is currently exceeding the threshold
int pollingDelay = 5000;            // general polling delay (may not be necessary with component poll frequencies already built in
int lastAlertTemp = 0;              // temperature as of the latest alert
int lastAlertFreezer = 0;              // temperature as of the latest alert
//
const int ALLCLEAR = 0;             // flag sent to Adafruit/IFTTT to clear alert state
const int TEMPALERT = 1;            // flag sent to Adafruit/IFTTT to indicate temperatue is too high
const int LIGHTALERT = 2;           // flag sent to Adafruit/IFTTT to indicate fridge light has been on more than allowed window
const int INITIALIZE = 3;           // flag sent to Adafruit/IFTTT to clear alert state
const int FREEZERALERT = 4;         // flag sent to Adafruit/IFTTT to indicate freezer temperatue is too high
//
// thresholds
//  
int connectTimeout = 30000;       // milliseconds to try to connect to wifi, after which will run in disconnected mode
int tempFreq = 30;                // temp sensor polling frequency (seconds)
int freezerFreq = 30;                // temp sensor polling frequency (seconds)
int lightFreq = 10;               // photosensor polling frequency (seconds)
float alertTempC = 7.0;           // alert will be triggered if temperature exceeds this value (Celcius) 
float alertFreezerC = -2.0;           // alert will be triggered if freezer temperature exceeds this value (Celcius) 
int alertLightR = 25;             // alert will be triggered if inverse % photo-resistance is above this value (also stored/retrieved on Adafruit feed)
int alertLightTime = 45;          // light on alert will be triggered if light is above threshold for this many seconds (for short door openings)
//
// Adafruit IO feeds
//
AdafruitIO_Feed *temperature = io.feed("Temperature");                            // temperature data sent to feed
AdafruitIO_Feed *freezer = io.feed("Freezer");                                    // freezer temperature data sent to feed
AdafruitIO_Feed *photoresistor = io.feed("Photoresistor");                        // photoresistor data sent to feed
AdafruitIO_Feed *freezerthreshold = io.feed("Freezer Threshold");                 // allows management of freezer temperature threshold
AdafruitIO_Feed *temperaturethreshold = io.feed("Temperature Threshold");         // allows management of temperature threshold
AdafruitIO_Feed *fridgealarm = io.feed("FridgeAlarm");                            // trigger watched by IFTTT to send notification
//
// email variables
//
EMailSender emailSend("**********@*********", "***********");
uint8_t connection_state = 0;
uint16_t reconnect_interval = 10000;
long unsigned int last_alert_email = 0;
int email_threshold = 10 * 60 * 1000; // 10 minutes = 10 minutes * 60 seconds * 1000 milliseconds
//
// melodies used for alerts
//
int wakeAlertLen = 4;
int wakeAlert[] = {NOTE_C5, NOTE_E5, NOTE_G5, NOTE_C6};     // C major scale
int wakeAlertBeat[] = {8, 8, 8, 2};

int wakeAlertNCLen = 4;
int wakeAlertNC[] = {NOTE_C5, NOTE_C4, NOTE_C5, NOTE_C4};   // C octaves
int wakeAlertNCBeat[] = {4, 4, 4, 2};

int tempAlertLen = 11;
int tempAlert[] = {NOTE_F4, NOTE_G4, 0, NOTE_C5, NOTE_C5, NOTE_B4, 0, NOTE_E4, NOTE_E4, NOTE_C4, NOTE_F4};   // "Hot Blooded"
int tempAlertBeat[] = {8, 4, 4, 4, 4, 4, 8, 4, 4, 4, 1};

int freezerAlertLen = 10;
int freezerAlert[] = {NOTE_FS5, NOTE_FS4, NOTE_FS5, NOTE_FS4, NOTE_FS5, NOTE_FS4, NOTE_FS5, NOTE_FS4, NOTE_FS5, NOTE_FS4};   // F# octaves
int freezerAlertBeat[] = {4, 4, 4, 4, 4, 4, 4, 4, 4, 4};

int lightAlertLen = 8;
int lightAlert[] = {NOTE_F5, NOTE_F5, NOTE_F5, NOTE_D5, NOTE_C5, NOTE_AS4, NOTE_C5, NOTE_A4};     // "Blinded by the Light"
int lightAlertBeat[] = {8, 8, 4, 4, 4, 4, 4, 4};
//
// initialize email wifi
//
uint8_t WiFiConnect(const char* nSSID = nullptr, const char* nPassword = nullptr)
{
    static uint16_t attempt = 0;
    Serial.print("Connecting to ");
    if(nSSID) {
        WiFi.begin(nSSID, nPassword);
        Serial.println(nSSID);
    }

    uint8_t i = 0;
    while(WiFi.status()!= WL_CONNECTED && i++ < 50)
    {
        delay(200);
        Serial.print(".");
    }
    ++attempt;
    Serial.println("");
    if(i == 51) {
        Serial.print("Connection: TIMEOUT on attempt: ");
        Serial.println(attempt);
        if(attempt % 2 == 0)
            Serial.println("Check if access point available or SSID and Password\r\n");
        return false;
    }
    Serial.println("Connection: ESTABLISHED");
    Serial.print("Got IP address: ");
    Serial.println(WiFi.localIP());
    return true;
}
void Awaits()
{
    uint32_t ts = millis();
    while(!connection_state)
    {
        delay(50);
        if(millis() > (ts + reconnect_interval) && !connection_state){
            connection_state = WiFiConnect();
            ts = millis();
        }
    }
}
//////////////////
// Board setup
//////////////////
void setup(){
  
  Serial.begin(115200);       // start serial port for debug
  while(! Serial);            // wait for serial monitor to open
//
// Adafruit IO handshake
//
  AIOInit();

  sensors.begin();                                         // start temperature sensor

  if(io.status() == AIO_CONNECTED){
    playAlert(INITIALIZE, wakeAlertLen, wakeAlert, wakeAlertBeat);          // audible "ready to go" tune (internet connected)
  } else {
    playAlert(INITIALIZE, wakeAlertNCLen, wakeAlertNC, wakeAlertNCBeat);    // audible "ready to go" tune (no internet connection)
  }
  emailAlert(INITIALIZE, 0, 0);
}
//////////////////////////////////
// Perpetual sensor monitor loop
//////////////////////////////////
void loop(){
  io.run();                   // always fired top of loop to keep connected and respond to Adafruit IO actions
  
  int latestLight = 0;        // latest retrieved photoresistor value
  float latestTemp = 0;       // latest retrieved temperature value
//
// fridge compartment temperature monitor (may refactor to have a common temp monitor function)
//
// check sensor and manage alerts once every n seconds
//
  if ((millis() - lastTempCheck) > (tempFreq * 1000))
  {
    lastTempCheck = millis();                   // reset polling counter
    sensors.requestTemperatures();              // send the command to get temperatures from the sensor bus
    latestTemp = sensors.getTempC(fridgeSensor);    // fridge is at index 1, freezer is at index 0
    temperature->save(latestTemp);              // send the temperature data to Adafruit IO
    Serial.print("temp:");
    Serial.print(latestTemp);
    Serial.print("  threshold:");
    Serial.println(alertTempC);

    if (latestTemp <= alertTempC)                             // temp is below alert threshold (OK)
    {
      if (tempAlertState)                                     // check if alert state was on (and now it's back to normal)
      {
        tempAlertState = false;
        allClear();                                           // back to normal - play all-clear  
      }
    }
    else                                                      // threshold exceeded
    {
      if (!tempAlertState)                                    // check if already in alert state
      {
        playAlert(TEMPALERT, tempAlertLen, tempAlert, tempAlertBeat);    // not already in alert state - play alert
        emailAlert(TEMPALERT, latestTemp, alertTempC);                           // email alert
        lastAlertTemp = latestTemp;                           // save the temp to compare later to see if temp is in decline
        tempAlertState = true;
      }
      else 
      {
        if (latestTemp >= lastAlertTemp)                       // play alert only if temp is going up or staying the same (someone may have closed the door)
        {
          Serial.print("last alert temp:");
          Serial.println(lastAlertTemp);
          playAlert(TEMPALERT, tempAlertLen, tempAlert, tempAlertBeat);   // play alert
          emailAlert(TEMPALERT, latestTemp, alertTempC);                           // email alert
          lastAlertTemp = latestTemp;                          // save the temp to compare later to see if temp is in decline          
        }
      }
    }
  }
//
// Freezer compartment temperature monitor (may refactor to have a common temp monitor function)
//
// check sensor and manage alerts once every n seconds
//
  if ((millis() - lastFreezerCheck) > (freezerFreq * 1000))
  {
    lastFreezerCheck = millis();                   // reset polling counter
    sensors.requestTemperatures();              // send the command to get temperatures from the sensor bus
    latestTemp = sensors.getTempC(freezerSensor);    // fridge is at index 1, freezer is at index 0
    freezer->save(latestTemp);              // send the temperature data to Adafruit IO
    Serial.print("freezer temp:");
    Serial.print(latestTemp);
    Serial.print("  threshold:");
    Serial.println(alertFreezerC);

    if (latestTemp <= alertFreezerC)                             // temp is below alert threshold (OK)
    {
      if (freezerAlertState)                                     // check if alert state was on (and now it's back to normal)
      {
        freezerAlertState = false;
        allClear();                                           // back to normal - play all-clear  
      }
    }
    else                                                      // threshold exceeded
    {
      if (!freezerAlertState)                                    // check if already in alert state
      {
        playAlert(FREEZERALERT, freezerAlertLen, freezerAlert, freezerAlertBeat);    // not already in alert state - play alert
        emailAlert(FREEZERALERT, latestTemp, alertFreezerC);                           // email alert
        lastAlertFreezer = latestTemp;                           // save the temp to compare later to see if temp is in decline
        freezerAlertState = true;
      }
      else 
      {
        if (latestTemp >= lastAlertFreezer)                       // play alert only if temp is going up or staying the same (someone may have closed the door)
        {
          Serial.print("last alert temp:");
          Serial.println(lastAlertFreezer);
          playAlert(FREEZERALERT, freezerAlertLen, freezerAlert, freezerAlertBeat);             // play alert  *** NEED NEW ALERT
          emailAlert(FREEZERALERT, latestTemp, alertFreezerC);                           // email alert
          lastAlertFreezer = latestTemp;                          // save the temp to compare later to see if temp is in decline          
        }
      }
    }
  }
////////////////////////////////
// photosensor monitor
////////////////////////////////
  if ((millis() - lastLightCheck) > (lightFreq * 1000))       // sensor check frequency
  {
    lastLightCheck = millis();      
    latestLight = (1024 - analogRead(lightPin)) / 10;         // read sensor's analog resistance values and invert to rough "percentage brightness"
    Serial.print("photo: ");
    Serial.print(latestLight);
    Serial.print("  threshold: ");
    Serial.println(alertLightR);
    photoresistor->save(latestLight);                         // send photoresitor data to Adafruit IO
     
    if (latestLight < alertLightR)                            // photosensor < alert = ok/light is off
    {
      if (lightAlertState)                                    // was in alert state but now is clear
      {
        lightAlertState = false;
        fridgealarm->save(0);                                 // clear Adafruit IO alarm state (door closed before alert sounded)        
      }
    }
    else                                                      // sensor exceeds threshold (alert status)
    {
      if (!lightAlertState)
      {
        lightAlertState = true;                               // initial change to alert stage - reset lag counter for 1st alert warning
        lightOnTime=millis();
      }
      else if ((millis() - lightOnTime) > (alertLightTime * 1000))      // check for lag
      {
        lightOnTime = millis();
        playAlert(LIGHTALERT, lightAlertLen, lightAlert, lightAlertBeat);           // play alert
        emailAlert(LIGHTALERT, latestLight, alertLightR);                           // email alert
      }
    }
  }
  delay(pollingDelay);                                         // small delay between polling tests
}

void playAlert(int alertIndicator, int alertLen, int alertNotes[], int alertBeat[]){
  Serial.print("Alert: ");
  Serial.println(alertLen);
  for (int thisNote = 0; thisNote < alertLen; thisNote++) {
    int noteDuration = 1000 / alertBeat[thisNote];
    tone(tonePin, alertNotes[thisNote], noteDuration); 
    int pauseBetweenNotes = noteDuration * 1.30;     // to distinguish the notes, set a minimum time between them.
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(tonePin);
  }
//
// send trigger data to Adafruit feed which is monitored by IFTTT which can send a notification to smart phones, etc.
//
  if (alertIndicator != INITIALIZE){
    fridgealarm->save(alertIndicator);
  }
}
//
// email/sms alert
//
void emailAlert(int alertType, float alertVal, float alertThreshold){
//*****
//***** REMOVE AFTER DEBUGGING
//*****
  Serial.print("Email type:");
  Serial.print(alertType);
  Serial.print("   val:");
  Serial.print(alertVal);  
  Serial.print("   threshold:");
  Serial.println(alertThreshold);  
//  return;
//*****
//***** REMOVE AFTER DEBUGGING
//*****

//
// for now, limit the frequency of emails/sms messages
//
  Serial.print("millis:");
  Serial.println(millis());
  Serial.print("last alert:");
  Serial.println(last_alert_email);
  Serial.print("threshold:");
  Serial.println(email_threshold);
//
// email/sms on first alert and no more than every 10 minutes thereafter
//
  if ((last_alert_email == 0) || (millis() > (last_alert_email + email_threshold)))
  {
  //
  // trigger email and email/sms alert (temp=1, light=2)
  //
    io.wifi_disconnect();   // temp disconnect from adafruit io - connection is not secure an email won't work
    delay(2000);            // allow disconnect to complete
        
    connection_state = WiFiConnect(WIFI_SSID, WIFI_PASS);       // re-connect for email
    if(!connection_state)  // if not connected to WIFI
        Awaits();          // constantly trying to connect
        
    EMailSender::EMailMessage message;                  // instantiate object
    String msg = "<br>Value recorded: ";
    msg.concat(alertVal);
    msg.concat("<br>Threshold: ");
    msg.concat(alertThreshold);
    switch(alertType){
      case 0:
        message.subject = "0-Fridge Alert is CLEARED";
        message.message = "Previous sensor alert(s) now clear.";
        break;
      case 1:
        message.subject = "1-Fridge TEMPERATURE Alert";
        message.message = "Fridge recorded a high temperature alert.";
        message.message.concat(msg);
        last_alert_email = millis();                // reset countdown to avoid repetitive emails
        break;
      case 2:
        message.subject = "2-Fridge LIGHT Alert";
        message.message = "Fridge recorded an interior light alert.";
        message.message.concat(msg);
        last_alert_email = millis();                // reset countdown to avoid repetitive emails
        break;
      case 3:
        message.subject = "3-Fridge Monitor Startup";
        message.message = "Fridge temperature and interior light monitor has begun.";
        break;
      case 4:
        message.subject = "4-FREEZER TEMPERATURE Alert";
        message.message = "Freezer compartment recorded a high temperature alert.";
        message.message.concat(msg);
        last_alert_email = millis();                // reset countdown to avoid repetitive emails
        break;
      default:
        String subj = "Fridge UNKNOWN Alert (";
        subj.concat(alertType);
        subj.concat(")");
        message.subject = subj;
        message.message = "Fridge recorded an unknown alert.";
        break;
    }
    const char* arrayOfEmail[] = {"5193650629@pcs.rogers.com", "markvanpraet@gmail.com"};  
 
    EMailSender::Response resp = emailSend.send(arrayOfEmail, 2, message);
  
    Serial.println("Sending status: ");
    Serial.println(resp.status);
    Serial.println(resp.code);
    Serial.println(resp.desc);
    
    AIOInit();            // reinitialize Adafruit IO feeds
  }
}
//
// Initialize the Adafruit IO feeds (thru MQTT)
//
void AIOInit(){
  Serial.print("Connecting to Adafruit IO");
  long unsigned int startConnect = millis();
  io.connect();                                     // connect to io.adafruit.com
  while((io.status() < AIO_CONNECTED) && (millis() < (startConnect + connectTimeout))){   // wait for a connection or until timeout
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.println(io.statusText());
  freezerthreshold->onMessage(handleFreezerThreshold);    // set up handle to catch any changes to the light threshold from Adafruit IO
  temperaturethreshold->onMessage(handleTempThreshold);    // set up handle to catch any changes to the temperature threshold from Adafruit IO    
  
  temperaturethreshold->get();                             // force retrieval of most recent threshold data
  freezerthreshold->get();                           // force retrieval of most recent threshold data
}

void allClear(){
  if(!tempAlertState && !lightAlertState)                 // both temperature and light alert states must be false to sound the all-clear
  {
    playAlert(ALLCLEAR, wakeAlertLen, wakeAlert, wakeAlertBeat);    // back to normal - play all-clear
    emailAlert(ALLCLEAR, 0, 0);
  }
}
//
// handle triggered by change to threshold value on Adafruit IO Feed
//
void handleFreezerThreshold(AdafruitIO_Data *data) {
  float frzThreshold = data->toFloat();                  // convert the data to integer
  alertFreezerC = frzThreshold;                        // set the new reference threshold
  Serial.print("<-freezer threshold: ");
  Serial.println(alertFreezerC);
}
//
void handleTempThreshold(AdafruitIO_Data *data) {
  float tempThreshold = data->toFloat();                  // convert the data to integer
  alertTempC = tempThreshold;                        // set the new reference threshold 
  Serial.print("<-temp threshold: ");
  Serial.println(alertTempC);
}
