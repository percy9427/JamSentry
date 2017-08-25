//    JamSentry is a program to detect Filament Jams and runout conditions
//    Copyright (C) 2017 - Stephen Hayes

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.

//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.

//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino

//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ESP8266mDNS.h>namecheap

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_HMC5883_U.h>

#define IFTTTHOST "maker.ifttt.com"  //Host address for sending IFTTT triggers
#define IFTTTPORT 80                 //Port for sending IFTTT triggers
//Various configuration parameters and lengths. These are stored in SPIFFS
#define PRINTER_NAME_LEN 40
#define EXTRUDER_ID_LEN 40
#define NO_SYNC_TIMEOUT_SECS_LEN 40
#define MAG_THRESHOLD_LEN 40
#define ALARM_CONDITION_LEN 40
#define GCODE_SENDER_IP_ADDR_LEN 40
#define GCODE_SENDER_PORT_LEN 40
#define REMOTE_PAUSE_PASSWORD_LEN 40
#define IFTTT_KEY_LEN 40
#define IFTTT_ALERT_IF_DONE_LEN 40
#define STATIC_IP_LEN 40
#define GATEWAY_IP_LEN 40
bool writeData=false;
char printer_name[PRINTER_NAME_LEN] = "Printer";
char extruder_id[EXTRUDER_ID_LEN] = "1";
char no_sync_timeout_secs[NO_SYNC_TIMEOUT_SECS_LEN] = "10";
char mag_threshold[MAG_THRESHOLD_LEN]="AUTO";
char alarm_condition[ALARM_CONDITION_LEN] = "LOW";
char gcode_sender_ip_addr[GCODE_SENDER_IP_ADDR_LEN] = "Not used";
char gcode_sender_port[GCODE_SENDER_PORT_LEN]= "27100";
char remote_pause_password[REMOTE_PAUSE_PASSWORD_LEN] = "JamSentryPSWD";
char ifttt_key[IFTTT_KEY_LEN] = "Not used";
char ifttt_alert_if_done[IFTTT_ALERT_IF_DONE_LEN] = "No";
char static_ip[STATIC_IP_LEN] = "Static IP Addr (or blank for DHCP)";
char gateway_ip[GATEWAY_IP_LEN] = "Gateway IP Addr (or blank for DHCP)";
//Hardware addresses and flags
bool tsl2561Exists=false;
byte tsl2461I2CAddr=0x39;
bool hcm5883Exists=false;
byte hcm5883I2CAddr=0x1E;  //Will try 0x0D if 0x1E doesn't work
bool resetOrNot = false;
float lightLevel = 0.0;
bool shouldSaveConfig = false;  //flag for saving data

ESP8266WebServer server(80);  //The server is hosted at port 80 by the Jam Sentry

const short int BUILTIN_LED1 = 0; //GPIO
const short int RUNOUT_ALARM = 15; //GPI15
//This state machine for JamSentry States.  State transitions are announced on the Serial Monitor
#define STATE_IDLE 0
#define STATE_POTENTIAL_EXTRUSION 1
#define STATE_EXTRUDING 2
#define STATE_POTENTIAL_JAM 3
#define STATE_JAMMED 4
#define STATE_DISABLED 5
#define STATE_NOT_WORKING 6
#define STATE_TESTING 7
#define STATE_CALIBRATING 8
int flowState = STATE_IDLE;
const int default_timeout = 10;   //Default timeout.  This is overwritten by whatever the user configures
const int default_gcode_sender_port=27100;   //Default port for sending pause requests.  This is overwritten by whatever the user configures
#define MILLISECONDS_BETWEEN_SAMPLES 50  //Sampling rate.  The ESP8266 seems to be able to handle this speed
const long continuousExtrudingSamplesThreshold = 60*1000/MILLISECONDS_BETWEEN_SAMPLES;  //Must be extruding this long before job is considered started
const long continuousIdleSamplesThreshold = 30*1000/MILLISECONDS_BETWEEN_SAMPLES;  //Must be idle this long before job is considered done
const int luminosityChangeThreshold = 2;  //Required change in encoder luminosity to be considered running.
float magnetometerChangeThreshold = -1;  //Required threshold of changing magnetic field.  Set dynamically during calibration step

//These contain the configured parameters after they have been validated
String validated_printer_name = "Printer Name";
String validated_extruder_id = "Extruder #";
int validated_no_sync_timeout_secs = 0;
long validated_mag_threshold = -1;
int validated_alarm_condition = LOW;  //RUNOUT_ALARM sent to this level when an alarm condition occurs
int validated_noalarm_condition = HIGH;  //RUNOUT_ALARM normally at this level
bool useRemoteGCodePause = false;  //Send pause alert to GCode Sender or not
IPAddress validated_gcode_sender_ip_addr(0, 0, 0, 0);
int validated_gcode_sender_port=27100;
String validated_remote_pause_password = "Pause Please";
bool useIFTTT = false;  //Whether IFTTT Will be used or not
String validated_ifttt_key = "MAKER IFTTT KEY";
bool validated_ifttt_alert_if_done = false;
bool useDHCP = true;  //Static IP or DHCP
IPAddress validated_static_ip(0, 0, 0, 0);
IPAddress validated_gateway_ip(0, 0, 0, 0);

String configParseMessage = "";
String ipParseMessage = "";
#define LUMINOSITY_ARRAY_SIZE 60*1000/MILLISECONDS_BETWEEN_SAMPLES
int luminosityArray[LUMINOSITY_ARRAY_SIZE];
#define MOTOR_ACTIVITY_ARRAY_SIZE 200
#define STEPPER_MIN_AUTO_VALUE 400
float motorActivityArray[MOTOR_ACTIVITY_ARRAY_SIZE];
int numArrayCells = 0;
int motorActivityIndex = 0;
int numMotorSamples = 0;
int luminosityIndex = 0;
bool suppressEarlyAlarms = true;
int alarmSuppressionTime = 20;
int suppressionTimer = 0;
int luminosityDifference = 0;
float currMagDiff = 0;
float avgMagDiff = 0;
float previousXMagField = 0;
float previousYMagField = 0;
float previousZMagField = 0;
float motorActivityCount = 0;
long continuousExtrudingSamples=0;
long continuousIdleSamples=0;
float motorThresholdToBeConsideredIdle = 0;  //Computed based on sampling speed.
float motorThresholdToBeConsideredExtruding = 5000;  //Computed based on sampling speed.
float maxMotorActivityCount = 10000; //Computed based on sampling speed.
int jamWatchCount = 0;
int testCount = 0;
int calibrationStepsPerformed = 0;
float peakCalibrationValue = 0;
#define CALIBRATION_CYCLES_REQUIRED 30

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);  //Define the luminosity sensor

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);  //Define the magnetometer.  Also intializes LSM303.

bool checkI2CAddr(byte addrToCheck) {  //This is used to check that the HW actually is connected
  byte error;
  Wire.begin(4,5);
  delay(20);
  Wire.beginTransmission(addrToCheck);
  error = Wire.endTransmission();

  if (error == 0){
    return true;
  }
  else {
    return false;
  }
}

void saveConfigCallback () {  //callback notifying us of the need to save config
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void saveConfig() {  //Saves configuration parameters to disk.
  Serial.println("saving config");
  DynamicJsonBuffer jsonBuffer;
  JsonObject& json = jsonBuffer.createObject();
  json["printer_name"] = printer_name;
  json["extruder_id"] = extruder_id;
  json["no_sync_timeout_secs"] = no_sync_timeout_secs;
  json["mag_threshold"] = mag_threshold;
  json["alarm_condition"] = alarm_condition;
  json["gcode_sender_ip_addr"] = gcode_sender_ip_addr;
  json["gcode_sender_port"] = gcode_sender_port;
  json["remote_pause_password"] = remote_pause_password;
  json["ifttt_key"] = ifttt_key;
  json["ifttt_alert_if_done"] = ifttt_alert_if_done;
  json["static_ip"] = static_ip;
  json["gateway_ip"] = gateway_ip;
  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    Serial.println("failed to open config file for writing");
  }
  json.printTo(Serial);
  json.printTo(configFile);
  configFile.close();
  //end save
}

void setActivityThresholds(){  //Sets the various thresholds that guide the machine between states
  numArrayCells = validated_no_sync_timeout_secs * 1000 / MILLISECONDS_BETWEEN_SAMPLES;
  numMotorSamples = 1 * 1000 / MILLISECONDS_BETWEEN_SAMPLES;
  motorThresholdToBeConsideredIdle = 1 * 1000 / MILLISECONDS_BETWEEN_SAMPLES;
  motorThresholdToBeConsideredExtruding = validated_no_sync_timeout_secs * 1000 / MILLISECONDS_BETWEEN_SAMPLES;
  maxMotorActivityCount = motorThresholdToBeConsideredExtruding + 2 * 1000 / MILLISECONDS_BETWEEN_SAMPLES; 
  if (validated_mag_threshold==-1) {
    if (flowState!=STATE_NOT_WORKING) {
      flowState=STATE_CALIBRATING;
      Serial.println("State changed to STATE_CALIBRATING");
      }
    calibrationStepsPerformed=0;
    peakCalibrationValue = 0;   
  }
  else {
    magnetometerChangeThreshold=validated_mag_threshold;
    flowState=STATE_IDLE;
    Serial.println("State changed to STATE_IDLE");
  }
  motorActivityCount=0;
  performUnjamAction;
}

void validateConfigParms() {  //Performs some rudimentary (non exhaustive) checks on user input values
  String printer_name_str(printer_name);
  String extruder_id_str(extruder_id);
  String no_sync_timeout_secs_str(no_sync_timeout_secs);
  String mag_threshold_str(mag_threshold);
  String alarm_condition_str(alarm_condition);
  String gcode_sender_ip_addr_str(gcode_sender_ip_addr);
  String gcode_sender_port_str(gcode_sender_port);
  String remote_pause_password_str(remote_pause_password);
  String ifttt_key_str(ifttt_key);
  String ifttt_alert_if_done_str(ifttt_alert_if_done);
  configParseMessage="\n\nCONFIGURATION LOG\n\n";

  validated_printer_name = printer_name_str;
  configParseMessage += "Supplied printer name was: \""  + printer_name_str + ".\"\n";
  if (printer_name_str == "Printer Name" || printer_name_str == "" ) {
    validated_printer_name = "Printer";
    configParseMessage += "Default printer name used: Printer.\n";
    Serial.println("Default printer name used: Printer. ");
  }
  validated_extruder_id = extruder_id_str;
  configParseMessage += "\n";
  configParseMessage += "Supplied extruder # was: \""  + extruder_id_str + "\".\n";
  if (extruder_id_str == "Extruder #" || extruder_id_str == "") {
    validated_extruder_id = "1";
    configParseMessage += "Default extruder used: \"1\".\n";
    Serial.println("Default extruder used: 1.  ");
  }
  configParseMessage += "\n";
  configParseMessage += "Supplied timeout was: \""  + no_sync_timeout_secs_str + "\".\n";
  if (no_sync_timeout_secs_str == "Timeout in secs" || no_sync_timeout_secs_str == "") {
    validated_no_sync_timeout_secs = default_timeout;
    configParseMessage += "Default timeout used: 10 secs.\n";
    Serial.println("Default timeout used: 10 secs. ");
  }
  else {
    validated_no_sync_timeout_secs = no_sync_timeout_secs_str.toInt();
  }
  if  (validated_no_sync_timeout_secs <= 0) {
    validated_no_sync_timeout_secs = default_timeout;
    configParseMessage += "Timeout value must be >=0, so default used: 10 secs.\n";
    Serial.println("Timeout value must be >=0, so default used: 10 secs ");
  }
  if  (validated_no_sync_timeout_secs > 60) {
    validated_no_sync_timeout_secs = 60;
    configParseMessage += "Timeout value must be <=60, so max used: 60 secs.\n";
    Serial.println("Timeout value must be <=60, so max used: 60 secs ");
  }
  configParseMessage += "\n";
  configParseMessage += "Supplied mag threshold was: \""  + mag_threshold_str + "\".\n";
  if (mag_threshold_str == "AUTO" || mag_threshold_str == "Auto" || mag_threshold_str == "") {
    validated_mag_threshold = -1;
    configParseMessage += "Mag Threshold will be determined automatically\n";
    Serial.println("Mag Threshold will be determined automatically. ");
  }
  else {
    validated_mag_threshold = mag_threshold_str.toInt();
  }
  if  (validated_mag_threshold <= 0) {
    validated_mag_threshold = -1;
    configParseMessage += "Mag Threshold must be >=0, so automatic used.\n";
    Serial.println("Mag Threshold must be >=0, so automatic used ");
  }
  if  (validated_mag_threshold > 100000) {
    validated_mag_threshold = -1;
    configParseMessage += "Mag Threshold <= 100000 , so automatic used.\n";
    Serial.println("Mag Threshold must be <=100000  , so automatic used. ");
  }
  configParseMessage += "\n";
  configParseMessage += "Supplied alarm condition was: \""  + alarm_condition_str + "\".\n";
  if  (alarm_condition_str == "Alarm LOW" || alarm_condition_str == "LOW" || alarm_condition_str == "0") {
    validated_alarm_condition = HIGH;  //The actual GPIO value is the opposite of the setting
    validated_noalarm_condition = LOW; //The actual GPIO value is the opposite of the setting
    configParseMessage += "Alarm condition (level to set when jam detected) set to LOW.\n";
    Serial.println("Alarm condition set to LOW ");
  }
  else {
    validated_alarm_condition = LOW; //The actual GPIO value is the opposite of the setting
    validated_noalarm_condition = HIGH; //The actual GPIO value is the opposite of the setting
    configParseMessage += "Alarm condition (level to set when jam detected) set to HIGH.\n";
    Serial.println("Alarm condition set to HIGH ");
  }
  configParseMessage += "\n";
  configParseMessage += "Supplied gcode sender ip was: \""  + gcode_sender_ip_addr_str + "\".\n";
  if (gcode_sender_ip_addr_str=="") {
    useRemoteGCodePause=false;
    configParseMessage += "Gcode sender alert will not be used.\n";
    Serial.println("Gcode sender alert will not be used");
  }
  else {
    if (validated_gcode_sender_ip_addr.fromString(gcode_sender_ip_addr_str)) {
      configParseMessage += "Gcode Sender IP address is valid.\n";
      Serial.println("GCode Sender IP found ");
      useRemoteGCodePause=true;
    }
    else {
      configParseMessage += "Invalid GCode Sender IP Addr so remote control of Sender not used.\n";
      Serial.println("Invalid GCode Sender IP Addr so remote control of Sender not used");
      useRemoteGCodePause=false;    
    }
  }

  configParseMessage += "\n";
  configParseMessage += "Supplied GCode Sender Port Was: \""  + gcode_sender_port_str + "\".\n";
  if (gcode_sender_port_str == "Unused" || gcode_sender_port_str == "") {
    validated_gcode_sender_port = default_gcode_sender_port;
    configParseMessage += "Default gcode sender port being used: 27100.\n";
    Serial.println("Default gcode sender port being used: 27100");
  }
  else {
    validated_gcode_sender_port = gcode_sender_port_str.toInt();
  }
  if  (validated_gcode_sender_port <= 0) {
    validated_gcode_sender_port = default_gcode_sender_port;
    configParseMessage += "Gcode sender port must be >=0, so default used: 27100.\n";
    Serial.println("Gcode sender port must be >=0, so default used: 27100");
  }
  if  (validated_gcode_sender_port >= 65535) {
    validated_gcode_sender_port = default_gcode_sender_port;
    configParseMessage += "Gcode sender port must be <=65535, so default used: 27100.\n";
    Serial.println("Gcode sender port must be <=65535, so default used: 27100");
  }
  
  configParseMessage += "\n";
  validated_remote_pause_password = remote_pause_password_str;
  configParseMessage += "\n";
  configParseMessage += "Supplied remote pause password # was: \""  + remote_pause_password_str + "\".\n";
  Serial.println("Supplied remote pause password # was: \""  + remote_pause_password_str + "\".");

  configParseMessage += "\n";
  configParseMessage += "Supplied IFTTT Key was: \""  + ifttt_key_str + "\".\n";
  if (ifttt_key_str == "MAKER IFTTT KEY" || ifttt_key_str == "" || ifttt_key_str == "None") {
    useIFTTT = false;
    configParseMessage += "No IFTTT Key so IFTTT disabled.\n";
    Serial.println("No IFTTT Key so IFTTT disabled");
  }
  else {
    useIFTTT = true;
    validated_ifttt_key = ifttt_key_str;
    configParseMessage += "IFTTT enabled using Key= \"" + validated_ifttt_key + "\".\n";
    Serial.println("IFTTT will be used");
  }
  configParseMessage += "\n";
  configParseMessage += "Supplied IFTTT Alert if Done was: \""  + ifttt_alert_if_done_str + "\".\n";
  if (ifttt_alert_if_done_str == "True" || ifttt_alert_if_done_str == "TRUE" || ifttt_alert_if_done_str == "YES" || ifttt_alert_if_done_str == "Yes" || ifttt_alert_if_done_str == "Y") {
    validated_ifttt_alert_if_done = true;
    configParseMessage += "IFTTT Alert if Done set to true.\n";
    Serial.println("IFTTT Alert if Done set to true.");
  }
  else {
    validated_ifttt_alert_if_done = false;
    configParseMessage += "IFTTT Alert if Done set to false.\n";
    Serial.println("IFTTT Alert if Done set to false.");
  }
  setActivityThresholds();  //New inputs, so recalculate the thresholds
}

void validateStaticIP() {  //Performs some rudimentary checks (like is it a valid format) on supplied IP address
  String static_ip_str(static_ip);
  String gateway_ip_str(gateway_ip);
  ipParseMessage="\n\nIP CONFIGURATION LOG\n\n";

  ipParseMessage += "Supplied static ip was: \""  + static_ip_str + "\".\n";
  if (validated_static_ip.fromString(static_ip_str)) {
    ipParseMessage += "Static IP address is valid.\n";
    Serial.println("Valid static IP found ");
    ipParseMessage += "Supplied gateway ip was: \""  + gateway_ip_str + "\".\n";
    if (validated_gateway_ip.fromString(gateway_ip_str)) {
      useDHCP = false;
      ipParseMessage += "Gateway IP address is valid.\n";
      ipParseMessage += "Static IP will be used.\n";
      Serial.println("Valid gateway IP found ");
    }
    else {
      useDHCP = true;
      ipParseMessage += "Invalid gateway IP so DHCP being used.\n";
      Serial.println("Invalid gateway IP so DHCP being used ");
    }
  }
  else {
    useDHCP = true;
    ipParseMessage += "Invalid static IP so DHCP being used.\n";
    Serial.println("Invalid static IP so DHCP being used ");
  }
  ipParseMessage += "\n";
}

void handleRoot() {  //This handles a root request to the server.
  String operationStatus = "NOT WORKING";
  String backgroundOperationColor = "FF69B4";
  if (flowState == STATE_IDLE) {
    operationStatus = "IDLE";
    backgroundOperationColor = "008000";
  }
  else if (flowState == STATE_POTENTIAL_EXTRUSION) {
    operationStatus = "POTENTIAL EXTRUSION";
    backgroundOperationColor = "00FFFF";
  }
  else if (flowState == STATE_EXTRUDING) {
    operationStatus = "EXTRUDING";
    backgroundOperationColor = "00BFFF";
  }
  else if (flowState == STATE_POTENTIAL_JAM) {
    operationStatus = "POTENTIAL JAM";
    backgroundOperationColor = "FFFF00";
  }
  else if (flowState == STATE_JAMMED) {
    operationStatus = "JAMMED";
    backgroundOperationColor = "FF0000";
  }
  else if (flowState == STATE_DISABLED) {
    operationStatus = "DISABLED";
    backgroundOperationColor = "D3D3D3";
  }
  else if (flowState == STATE_TESTING) {
    operationStatus = "TESTING";
    backgroundOperationColor = "FFA500";
  }
    else if (flowState == STATE_CALIBRATING) {
    operationStatus = "CALIBRATING";
    backgroundOperationColor = "C000FF";
  }

  String currMagValueStr="()";
  if (calibrationStepsPerformed>=CALIBRATION_CYCLES_REQUIRED * 1000/MILLISECONDS_BETWEEN_SAMPLES) {
    currMagValueStr="(" + String(magnetometerChangeThreshold) + ")";
  }
  String mag_threshold_str = "AUTO " + currMagValueStr;
  if (validated_mag_threshold!=-1) {
    mag_threshold_str=String(validated_mag_threshold);
  }

  String alarm_str = "LOW";
  if (validated_alarm_condition == LOW) {
    alarm_str = "HIGH";
  }
  String firstButton = "<a href=\"disable\"><button class=\"button buttonEnable\">DISABLE</button></a>";
  if (flowState == STATE_DISABLED) {
    firstButton = "<a href=\"enable\"><button class=\"button buttonDisable\">ENABLE</button></a>";
  }
  String gcode_sender_ip = "Not used";
  if (useRemoteGCodePause) {
    gcode_sender_ip= validated_gcode_sender_ip_addr.toString();
  }
  String key = "Not used";
  if (useIFTTT) {
    key = validated_ifttt_key;
  }
  String staticIPAddr = static_ip;
  String gatewayIPAddr = gateway_ip;
  if (useDHCP) {
    staticIPAddr = "DHCP Used";
    gatewayIPAddr = "DHCP Used";
  }
  String iftttAlertIfDone="No";
  if (validated_ifttt_alert_if_done) {
    iftttAlertIfDone="Yes";
  }
  String tsl2561Status="Not Found";
  String tsl2561Background="FF0000";
  if (tsl2561Exists) {tsl2561Status="Working"; tsl2561Background="00FF00";}
  String hcm5883Status="Not Found";
  String hcm5883Background="FF0000";
  if (hcm5883Exists) {hcm5883Status="Working"; hcm5883Background="00FF00";}

  String statusMsg = "<html><head><meta http-equiv='refresh' content='5; URL=\"/\"'/><title>" + validated_printer_name + " JamSentry Status</title>"
                     "<style>body { background-color: #d6eaf8; font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 2em;}</style>"
                     "<style>.button {font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 1em; width: 400px; height: 90px; border-radius: 25px;}"
                     ".buttonEnable {background-color: #0af915;} .buttonDisable {background-color: #f90a0a;}  .buttonUpdate {background-color: #fc8200;} .buttonTest {background-color: #808080;} "
                     ".buttonLog {background-color: #808080; font-size: .5em; width: 200px; height: 20px; border-radius: 10px;}</style>"
                     "<style>.logo-with-text { font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 0.5em; float: right}</style>"
                     "<style>table { font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 1em;}</style></head>"
                     "<body><div class=\"logo-with-text\">Brought to you by<p><a href=\"http://robogardens.com\"><img src=\""
                     "http://robogardens.com/wp-content/uploads/2017/06/cropped-RobogardensTransparent512-150x150.png\" alt=\"RoboGardens.com\"></a></div>"
                     "<h1>" + validated_printer_name + " JamSentry Status</h1>"
                     "<p style=\"color: #000000; background-color: #" + backgroundOperationColor + "\">Operational Status: " + operationStatus +
                     "</p>Filament Sensor Status : " + tsl2561Status + " (" + String(luminosityDifference) + ")"
                     ",  Stepper Sensor Status: " + hcm5883Status +  " (" + String(avgMagDiff) + ")"
                     "</p><table border=\"2\"><tr>Configuration Settings</tr><tr><td>Printer Name</td><td bgcolor=#ffffff>" + validated_printer_name +
                     "</td></tr><tr><td>Extruder</td><td bgcolor=#ffffff>" + validated_extruder_id +
                     "</td></tr></tr><td>Timeout in secs</td><td bgcolor=#ffffff>" + validated_no_sync_timeout_secs +
                     "</td></tr></tr><td>Stepper Threshold</td><td bgcolor=#ffffff>" + mag_threshold_str +
                     "</td></tr><tr><td>Alarm Condition</td><td bgcolor=#ffffff>" + alarm_str +
                     "</td></tr><tr><td>Gcode Sender IP Address</td><td bgcolor=#ffffff>" + gcode_sender_ip +
                     "</td></tr><tr><td>Gcode Sender Port</td><td bgcolor=#ffffff>" + String(validated_gcode_sender_port) +
                     "</td></tr><tr><td>Remote Pause Password</td><td bgcolor=#ffffff>" + validated_remote_pause_password +
                     "</td></tr><tr><td>MAKER IFTT KEY</td><td bgcolor=#ffffff>" + key +
                     "</td></tr><tr><td>IFTTT Alert if Done</td><td bgcolor=#ffffff>" + iftttAlertIfDone +
                     "</td></tr><tr><td>JamSentry IP Addr</td><td bgcolor=#ffffff>" + WiFi.localIP().toString()                                                                     +
                     "</td></tr></table><a href=\"log\"><button class=\"button buttonLog\">CONFIG LOG</button></a><p>" +
                     firstButton + "&nbsp<a href=\"update\"><button class=\"button buttonUpdate\">UPDATE CONFIG</button></a>"
                     "&nbsp<a href=\"test\"><button class=\"button buttonTest\">TEST JAM</button></a></p></body></html>";
  server.send(200, "text/html", statusMsg);
}

void handleNotFound() {  //Unrecognized request eq. 192.168.1.99/invalid
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {                                                 
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
}

void sendAlert(String hostToAlert, int portToAlert, String eventName) {  //Send an alert in IFTTT format to the given host and port
  String response = "";
  bool finishedHeaders = false;
  bool currentLineIsBlank = true;
  long now;
  bool avail;
  #define MAX_ALERT_ATTEMPTS 8
  int currentAttempt=0;
  bool attemptSuccessful=false;
  delay(100);
  String value3 = validated_remote_pause_password;
  if (hostToAlert==IFTTTHOST) {
    value3 = String(random(1000000));  //We need a random value for ITFFF otherwise, it will ignore a trigger that is the same as last time
  }
  while (!attemptSuccessful & currentAttempt<MAX_ALERT_ATTEMPTS) {
    WiFiClient client;
    DynamicJsonBuffer jsonBuffer;
    JsonObject& payload = jsonBuffer.createObject();
    payload["value1"] = validated_printer_name;
    if (flowState == STATE_TESTING) {
      payload["value2"] = "TESTING";
    }
    else {
      payload["value2"] = validated_extruder_id;
    }
    payload["value3"] = value3;
    hostToAlert.toCharArray(gcode_sender_ip_addr, GCODE_SENDER_IP_ADDR_LEN);
    if (client.connect(gcode_sender_ip_addr, portToAlert)) {
      //Serial.println(".... connected to server");
      String a = "";
      char c;
      int ch_count = 0;
      delay(100);
      Serial.println("POST /trigger/" + eventName + "/with/key/" + validated_ifttt_key);
      client.print("POST /trigger/" + eventName + "/with/key/" + validated_ifttt_key);
      client.println(" HTTP/1.1");
      // Host header
      client.print("Host:");
      client.println(hostToAlert);
      // JSON content type
      client.println("Content-Type: application/json");
      // Content length
      int length = payload.measureLength();
      client.print("Content-Length:");
      client.println(length);
      // End of headers
      client.println();
      // POST message body
      String out;
      payload.printTo(out);
      //Serial.println(out);
      client.println(out);
      now = millis();
      avail = false;
      //Serial.println("starting timer");
      while (millis() - now < 1500) {
        while (client.available()) {
          char c = client.read();
          //Serial.print(c);
          response = response + c;
        }
      }
      if (response) {
        Serial.println("IFTTT Successfully sent");
        attemptSuccessful=true;
      }
      else {
        Serial.println("IFTTT Failed");
        currentAttempt++;
      }
    }
    else {
      Serial.println("Cannot connect to IFTTT host");
      currentAttempt++;
    }
    client.stop();
    delay(100);
  }  
}

void performJamAction() {   //Do this when a Jam is detected
  writeData=false;
  digitalWrite(RUNOUT_ALARM, validated_alarm_condition);
  Serial.println("JAMMED");
  if (useRemoteGCodePause) {
    Serial.println("Sending to gcode sender");    
    sendAlert(validated_gcode_sender_ip_addr.toString(),validated_gcode_sender_port,"Jam");
  }
  if (useIFTTT) {
    Serial.println("Sending to ifttt");    
    sendAlert(IFTTTHOST,IFTTTPORT,"Jam");
  }
}

void performUnjamAction(){  //Do this when a Jam is over
  digitalWrite(RUNOUT_ALARM, validated_noalarm_condition);
  Serial.println("UNJAMMED");
}

bool isFilamentStopped() {  //Test to see if the filament is moving based on the sampling history
  int currLuminosity = 0;
  int minLuminosityValue = 9999;
  int maxLuminosityValue = 0;
  for (int x = 0; x < numArrayCells; x++) {
    currLuminosity = luminosityArray[x];
    if (currLuminosity > maxLuminosityValue) {
      maxLuminosityValue = currLuminosity;
    }
    if (currLuminosity < minLuminosityValue) {
      minLuminosityValue = currLuminosity;
    }
  }
  luminosityDifference = maxLuminosityValue - minLuminosityValue;
  return (luminosityDifference < luminosityChangeThreshold);
}

float getMotorActivity() {  //Get an average motor activity value based on the sampling history
  float totalSamples = 0;
  for (int x = 0; x < numMotorSamples; x++) {
    totalSamples = totalSamples + motorActivityArray[x];
  }
  float avgMotor = totalSamples / numMotorSamples;
  return avgMotor;
}

void setup(void) {
//The standard setup routine.  Note that this uses the wifi manager.  If it cannot set up a wifi client connection
//Then it comes up as a standalone AP (Named JamSentrySetup).  This hosts a capture portal that allows you to set
//The desired WiFi SSID, password, and also the static IP//gateway IP if you choose not to use DHCP.
//The first restart it usually comes up on the desired network, but with DHCP.
//The second restart it comes up with the correct IP address
  pinMode(BUILTIN_LED1, OUTPUT); // Initialize the BUILTIN_LED1 pin as an output
  pinMode(RUNOUT_ALARM, OUTPUT); // Initialize the alarm output pin as an output
  Serial.begin(115200);
  Serial.println("JamSentry Initializing"); Serial.println("");
  digitalWrite(BUILTIN_LED1, LOW); // Turn the LED ON by making the voltage LOW

  //read configuration from FS json
  Serial.println("mounting FS...");
  //SPIFFS.format();  //Uncomment this to reformat the disk and forget all your saved config parameters.

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(printer_name, json["printer_name"]);
          strcpy(extruder_id, json["extruder_id"]);
          strcpy(no_sync_timeout_secs, json["no_sync_timeout_secs"]);
          strcpy(mag_threshold, json["mag_threshold"]);
          strcpy(alarm_condition, json["alarm_condition"]);
          strcpy(gcode_sender_ip_addr, json["gcode_sender_ip_addr"]);
          strcpy(gcode_sender_port, json["gcode_sender_port"]);
          strcpy(remote_pause_password, json["remote_pause_password"]);
          strcpy(ifttt_key, json["ifttt_key"]);
          strcpy(ifttt_alert_if_done, json["ifttt_alert_if_done"]);
          strcpy(static_ip, json["static_ip"]);
          strcpy(gateway_ip, json["gateway_ip"]);

        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
    Serial.println("Formatting FS");
    SPIFFS.format();
    Serial.println("Rebooting");
    ESP.reset();
    delay(5000);
  }
  //end read

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_static_ip("static", "static ip", static_ip, STATIC_IP_LEN);
  WiFiManagerParameter custom_gateway_ip("gateway", "gateway ip", gateway_ip, GATEWAY_IP_LEN);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //add all your parameters here
  wifiManager.addParameter(&custom_static_ip);
  wifiManager.addParameter(&custom_gateway_ip);

  //get the static ip address if any
  strcpy(static_ip, custom_static_ip.getValue());
  strcpy(gateway_ip, custom_gateway_ip.getValue());
  validateStaticIP();

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimum quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(600);

  if (!useDHCP) {
    Serial.print('Static IP to use: ');
    Serial.println(validated_static_ip.toString());
    Serial.print('Gateway IP to use: ');
    Serial.println(validated_gateway_ip.toString());
    wifiManager.setSTAStaticIPConfig(validated_static_ip, validated_gateway_ip, IPAddress(255, 255, 255, 0));
  }
  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (!wifiManager.autoConnect("JamSentrySetup")) {
    Serial.println("failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connection established");

  strcpy(static_ip, custom_static_ip.getValue());
  strcpy(gateway_ip, custom_gateway_ip.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    saveConfig();
  }

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (checkI2CAddr(tsl2461I2CAddr)) {
    if (!tsl.begin())
      {
      Serial.println("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
      tsl2561Exists = false;
      flowState = STATE_NOT_WORKING;
      Serial.println("State changed to STATE_NOT_WORKING");
    }
    else {
      tsl2561Exists = true;
      tsl.enableAutoRange(TSL2561_GAIN_16X);            /* Auto-gain ... switches automatically between 1x and 16x */
      tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
     }
  }
  else {
    tsl2561Exists = false;
    flowState = STATE_NOT_WORKING;
    Serial.println("State changed to STATE_NOT_WORKING");
  }
  
  hcm5883Exists = false;
  if (checkI2CAddr(hcm5883I2CAddr)) {
    hcm5883Exists=true;
  }
  if (hcm5883Exists) {
    if (!mag.begin())
    {
      /* There was a problem detecting the HMC5883 ... check your connections */
      Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
      hcm5883Exists = false;
      flowState = STATE_NOT_WORKING;
      Serial.println("State changed to STATE_NOT_WORKING");
    }
    else {
      Serial.println("Magnetometer Sensor Found");
    }
  }
  else {
     flowState = STATE_NOT_WORKING;
    Serial.println("State changed to STATE_NOT_WORKING");    
  }

  validateConfigParms();
  //Set the alarm output appropriately
  digitalWrite(RUNOUT_ALARM, validated_noalarm_condition); // Set the unalarmed state

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp8266")) {
    Serial.println("MDNS responder started");
  }

//Various entry points for expected server responses
  server.on("/", handleRoot);

  server.on("/update", []() {
    String validated_mag_threshold_str = "AUTO";
    if (validated_mag_threshold!=-1) {
      validated_mag_threshold_str=String(validated_mag_threshold);
    }
    String alarm_str = "LOW";
    if (validated_alarm_condition == LOW) {
      alarm_str = "HIGH";
    }
    String gcode_sender_ip = "Not used";
    if (useRemoteGCodePause) {
      gcode_sender_ip= validated_gcode_sender_ip_addr.toString();
    }
    String iftttAlertIfDoneStr="No";
    if (validated_ifttt_alert_if_done) {iftttAlertIfDoneStr="Yes";}
    
    String updateQuery = "<html><head><title>Configuration Update</title>"
                         "<style>body { background-color: #808080; font-family: Arial, Helvetica,Sans-Serif; Color: #000088; font-size: 1.5em;}</style></head>"
                         "<style>.button {font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 1em; width: 400px; height: 90px; border-radius: 25px;}"
                         ".buttonReset {background-color: #fc8200;} .buttonCancel {background-color: #aaaaaa;}</style>"
                         "<style>input[type=text] {background-color: #fc8200; font-family: Arial, Helvetica,Sans-Serif; Color: #000088; font-size: 1em;} "
                         "input[type=submit] {font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 1em; width: 400px; height: 90px; border-radius: 25px; background-color: #ff0000;}</style>"
                         "<style>.container {width: 600px; clear: both; } .container input {width: 100%; clear: both;}</style>"
                         "Update non communication parameters here:<br>"
                         "<div class=\"container\"><form action = \"updateresults\" method=\"get\">Printer Name:<input type=\"text\" name=\"printer_name_update\" value=\"" + validated_printer_name +
                         "\"><br>Extruder ID:<input type=\"text\" name=\"extruder_id_update\" value=\"" + validated_extruder_id +
                         "\"><br>Mismatch Timeout in Secs:<input type=\"text\" name=\"validated_no_sync_timeout_secs_update\" value=\"" + String(validated_no_sync_timeout_secs) +
                         "\"><br>Stepper Threshold or AUTO:<input type=\"text\" name=\"validated_mag_threshold_update\" value=\"" + validated_mag_threshold_str +
                         "\"><br>Alarm Level High or Low:<input type=\"text\" name=\"alarm_condition_update\" value=\"" + alarm_str +
                         "\"><br>Gcode Sender IP Address (or blank to disable):<input type=\"text\" name=\"gcode_sender_ip_addr_update\" value=\"" + gcode_sender_ip +
                         "\"><br>Gcode Sender Port:<input type=\"text\" name=\"gcode_sender_port_update\" value=\"" + String(validated_gcode_sender_port) +
                         "\"><br>Remote Pause Password:<input type=\"text\" name=\"remote_pause_password_update\" value=\"" + validated_remote_pause_password +
                         "\"><br>IFTTT MAKER KEY (or blank to disable):<input type=\"text\" name=\"ifttt_key_update\" value=\"" + validated_ifttt_key +
                         "\"><br>IFTTT Alert if Done (Yes/No):<input type=\"text\" name=\"ifttt_alert_if_done\" value=\"" + iftttAlertIfDoneStr +
                         "\"><br><br><input type=\"submit\" value=\"SUBMIT\"></form></div>"
                         "&nbsp<a href=\"cancel\"><button class=\"button buttonCancel\">CANCEL</button></a><br>"
                         "Update WiFi and IP Parameters here:<br>&nbsp<a href=\"reset\"><button class=\"button buttonReset\">RESET WIFI</button></a></body></html>";
    server.send(200, "text/html", updateQuery);
  });

  server.on("/cancel", []() {
    handleRoot();
  });

  server.on("/updateresults", []() {
    server.arg(0).toCharArray(printer_name, PRINTER_NAME_LEN);
    server.arg(1).toCharArray(extruder_id, EXTRUDER_ID_LEN);
    server.arg(2).toCharArray(no_sync_timeout_secs, NO_SYNC_TIMEOUT_SECS_LEN);
    server.arg(3).toCharArray(mag_threshold, MAG_THRESHOLD_LEN);
    server.arg(4).toCharArray(alarm_condition, ALARM_CONDITION_LEN);
    server.arg(5).toCharArray(gcode_sender_ip_addr, GCODE_SENDER_IP_ADDR_LEN);
    server.arg(6).toCharArray(gcode_sender_port, GCODE_SENDER_PORT_LEN);
    server.arg(7).toCharArray(remote_pause_password, REMOTE_PAUSE_PASSWORD_LEN);
    server.arg(8).toCharArray(ifttt_key, IFTTT_KEY_LEN);
    server.arg(9).toCharArray(ifttt_alert_if_done, IFTTT_ALERT_IF_DONE_LEN);
    validateConfigParms();
    //Set the alarm output appropriately
    digitalWrite(RUNOUT_ALARM, validated_noalarm_condition); // Set the unalarmed state

    saveConfig();
    handleRoot();
  });

  server.on("/reset", []() {
    resetOrNot = true;
    String resetQuery = "<html><head><title>Wifi Reset</title>"
                        "<style>body { background-color: #fc8200; font-family: Arial, Helvetica,Sans-Serif; Color: #000088; font-size: 2em;}</style></head>"
                        "<style>.button {font-family: Arial, Helvetica, Sans-Serif; Color: #000088; font-size: 1em; width: 400px; height: 90px; border-radius: 25px;}"
                        ".buttonYes {background-color: #f9240a;} .buttonNo {background-color: #0af2f9;}</style>"
                        "Are you sure you want to reset the configuration? <br><a href=\"Yes\"><button class=\"button buttonYes\">YES</button></a>&nbsp<a href=\"No\"><button class=\"button buttonNo\">No</button></a></p></body></html>";
    server.send(200, "text/html", resetQuery);
  });

  server.on("/Yes", []() {
    if (resetOrNot) {
      server.send(200, "text/html", "Resetting. Connect WiFi to JamSentrySetup to reconfigure");
      WiFiManager wifiManager;
      wifiManager.resetSettings();
      ESP.reset();
      delay(5000);
    }
    else {
      handleRoot();
    }
  });

  server.on("/No", []() {
    resetOrNot = false;
    handleRoot();
  });

  server.on("/enable", []() {
    if (flowState != STATE_NOT_WORKING) {
      setActivityThresholds();
    }
    handleRoot();
  });

  server.on("/disable", []() {
    if (flowState != STATE_NOT_WORKING) {
      flowState = STATE_DISABLED;
      Serial.println("State changed to STATE_DISABLED");
    }
    handleRoot();
  });

  server.on("/log", []() {
    server.send(200, "text/plain", ipParseMessage+configParseMessage);
  });

  server.on("/test", []() {
    if (flowState != STATE_NOT_WORKING) {
      flowState = STATE_TESTING;
      performJamAction();
      Serial.println("State changed to STATE_TESTING");
      testCount = 0;
    }
    handleRoot();
  });

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop()
{
  if (hcm5883Exists & tsl2561Exists) {  //Only do this processing if the sensors exist.  Note the hcm5883 driver hangs if you attempt to access it and the hw doesnt exist
    sensors_event_t event;
    mag.getEvent(&event);
    float diffXSq = (previousXMagField - event.magnetic.x) * (previousXMagField - event.magnetic.x);
    float diffYSq = (previousYMagField - event.magnetic.y) * (previousYMagField - event.magnetic.y);
    float diffZSq = (previousZMagField - event.magnetic.z) * (previousZMagField - event.magnetic.z);
    previousXMagField = event.magnetic.x;
    previousYMagField = event.magnetic.y;
    previousZMagField = event.magnetic.z;
    currMagDiff= diffXSq + diffYSq + diffZSq;
    motorActivityArray[motorActivityIndex] = currMagDiff;
    if (++motorActivityIndex >= numMotorSamples) {
      motorActivityIndex = 0;
    }
    avgMagDiff = getMotorActivity();
    tsl.getEvent(&event);
    if (event.light) {
      lightLevel = event.light;
    }
    else
    {
      lightLevel = 0;
    }
    luminosityArray[luminosityIndex] = lightLevel;
    if (++luminosityIndex >= numArrayCells) {
      luminosityIndex = 0;
    }
  }
  bool motorMoving = (avgMagDiff >= magnetometerChangeThreshold);
  bool filamentMoving = !isFilamentStopped();
  if (writeData) {
    Serial.print("Light: "); Serial.print(lightLevel); Serial.print(", LAvg: "); Serial.print(luminosityDifference); Serial.print(", CurrMagDiff: "); Serial.print(currMagDiff);  Serial.print(",  Avg: ");  Serial.println(avgMagDiff);
//      Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print(",  Y: ");  Serial.print(event.magnetic.y); Serial.print(",  Z: ");  Serial.print(event.magnetic.z); Serial.print(",  Diff: ");  Serial.println(avgMagDiff);
  }
  //  Serial.print(luminosityIndex); Serial.print("  "); Serial.println(luminosityArray[luminosityIndex]);
  //  /* Display the results from the Sensors */
  //Serial.print("Mag diff: "); Serial.print(avgMagDiff); Serial.print(",  Luminosity: ");  Serial.print(lightLevel); Serial.print(",  Motor Activity Count: ");  Serial.println(motorActivityCount);
  //State machine
  if (flowState != STATE_DISABLED & flowState != STATE_NOT_WORKING) {
    if (flowState == STATE_IDLE) {
      ++continuousIdleSamples;
//      Serial.print("Continuous Idle: "); Serial.print(continuousIdleSamples); Serial.print(", Continuous Extruding: "); Serial.println(continuousExtrudingSamples);
      if (continuousIdleSamples>=continuousIdleSamplesThreshold & continuousExtrudingSamples>=continuousExtrudingSamplesThreshold) {
        if (useIFTTT & validated_ifttt_alert_if_done) {
          sendAlert(IFTTTHOST,IFTTTPORT,"Done");
          continuousExtrudingSamples=0;
        }
      }
      if (motorMoving) {
        motorActivityCount = motorActivityCount + 1.0;
        if (motorActivityCount >= motorThresholdToBeConsideredIdle) {
          flowState = STATE_POTENTIAL_EXTRUSION;
          Serial.println("State changed to STATE_POTENTIAL_EXTRUSION");
        }
      }
      else {
        motorActivityCount = motorActivityCount - 1.0;
        if (motorActivityCount < 0.0) {
          motorActivityCount = 0.0;
        }
      }
    }
    else if (flowState == STATE_POTENTIAL_EXTRUSION) {
      if (motorMoving) {
        motorActivityCount = motorActivityCount + 1.0;
        if (motorActivityCount >= motorThresholdToBeConsideredExtruding) {
          flowState = STATE_EXTRUDING;
          Serial.println("State changed to STATE_EXTRUDING");
        }
      }
      else {
        motorActivityCount = motorActivityCount - 1.0;
        if (motorActivityCount < motorThresholdToBeConsideredIdle) {
          flowState = STATE_IDLE;
          Serial.println("State changed to STATE_IDLE");
        }
      }
    }
    else if (flowState == STATE_EXTRUDING) {
      ++continuousExtrudingSamples;
      continuousIdleSamples=0;
      if (motorMoving) {
        motorActivityCount = motorActivityCount + 1.0;
        if (motorActivityCount > maxMotorActivityCount) {
          motorActivityCount = maxMotorActivityCount;
        }
      }
      else {
        motorActivityCount = motorActivityCount - 1.0;
        if (motorActivityCount < motorThresholdToBeConsideredExtruding) {
          flowState = STATE_POTENTIAL_EXTRUSION;
          Serial.println("State changed to STATE_POTENTIAL_EXTRUSION");
        }
      }
      if (flowState == STATE_EXTRUDING) {
        if (!filamentMoving) {
          flowState = STATE_POTENTIAL_JAM;
          Serial.println("State changed to STATE_POTENTIAL_JAM");
          jamWatchCount = 0;
        }
      }
    }
    else if (flowState == STATE_POTENTIAL_JAM) {
      if (motorMoving) {
        motorActivityCount = motorActivityCount + 1.0;
        if (motorActivityCount > maxMotorActivityCount) {
          motorActivityCount = maxMotorActivityCount;
        }
      }
      else {
        motorActivityCount = motorActivityCount - 1.0;
        if (motorActivityCount < motorThresholdToBeConsideredExtruding) {
          flowState = STATE_POTENTIAL_EXTRUSION;
          Serial.println("State changed to STATE_POTENTIAL_EXTRUSION");
        }
      }
      if (flowState == STATE_POTENTIAL_JAM) {
        if (filamentMoving) {
          flowState = STATE_EXTRUDING;
          Serial.println("State changed to STATE_EXTRUDING");
        }
        else if (jamWatchCount >= validated_no_sync_timeout_secs * 1000 / MILLISECONDS_BETWEEN_SAMPLES) {
          flowState = STATE_JAMMED;
          Serial.println("State changed to STATE_JAMMED");
          performJamAction();
        }
        else {
          jamWatchCount++;
        }
      }
    }
    else if (flowState == STATE_JAMMED) {
      continuousExtrudingSamples=0;
      continuousIdleSamples=0;
      if (motorMoving) {
        motorActivityCount = motorActivityCount + 1.0;
        if (motorActivityCount > maxMotorActivityCount) {
          motorActivityCount = maxMotorActivityCount;
        }
      }
      else {
        motorActivityCount = motorActivityCount - 1.0;
        if (motorActivityCount < motorThresholdToBeConsideredExtruding) {
        flowState = STATE_IDLE;
        motorActivityCount = 0.0;
        Serial.println("State changed to STATE_IDLE");
        performUnjamAction();
        }
      }
      if (flowState == STATE_JAMMED) {
        if (filamentMoving) {
          flowState = STATE_IDLE;
          motorActivityCount = 0.0;
          Serial.println("State changed to STATE_IDLE");
          performUnjamAction();
        }
      }
    }
    else if (flowState == STATE_TESTING) {
      testCount++;
      if (testCount > MILLISECONDS_BETWEEN_SAMPLES * 1) {
        performUnjamAction();
        motorActivityCount = 0.0;
        flowState = STATE_IDLE;
        Serial.println("State changed to STATE_IDLE");
      }
    }
    else if (flowState == STATE_CALIBRATING) {
      calibrationStepsPerformed++;
      if (calibrationStepsPerformed > CALIBRATION_CYCLES_REQUIRED * 1000/MILLISECONDS_BETWEEN_SAMPLES) {
        if (800*peakCalibrationValue<STEPPER_MIN_AUTO_VALUE) {
          magnetometerChangeThreshold=STEPPER_MIN_AUTO_VALUE;
        }
        else {
          magnetometerChangeThreshold=800*peakCalibrationValue;          
        }
        Serial.print("Magnetometer Threshold Set To: "); Serial.println(magnetometerChangeThreshold); 
        performUnjamAction();
        motorActivityCount = 0.0;
        flowState = STATE_IDLE;
        Serial.println("State changed to STATE_IDLE");
      }
      else {
        if (avgMagDiff>peakCalibrationValue & calibrationStepsPerformed>(1000/MILLISECONDS_BETWEEN_SAMPLES+1)) {
          peakCalibrationValue=avgMagDiff;
        }
      }
    }
  }
  server.handleClient();
  delay(MILLISECONDS_BETWEEN_SAMPLES);
}
