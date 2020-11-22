#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <FS.h>
#include "SerialComm.h"

const String rooms[] = { "Kitchen", "Room A", "Room B", "Room C", "Table", "Shower", "Toilet", "Stairs" };
const String areas[] = { "Lamp A", "Lamp B", "Lamp C", "Group" };
const String dtypes[] = { "lightLvl", "minLightLvl", "fadeRate", "fadeDelay", "msllt", "fanLvl", "minRH" };
const String resetTimerRequest = "Reset counter";

ESP8266WebServer server(80);
StaticJsonDocument<192> jsonBuffer;
VentilationConfig ShowerConfig = VentilationConfig(0x12, 0x13);
VentilationState ShowerState = VentilationState(0x11, 0);
FloorEndpoints floors[2] = {  
  { ControllerConfig(0x22, 0x23), ControllerState(0x21, 0), 
  ControllerChOnTime(0x24, 0), ControllerOTSet(0, 0x23) }, 
  { ControllerConfig(0x32, 0x33), ControllerState(0x31, 0), 
  ControllerChOnTime(0x34, 0), ControllerOTSet(0, 0x33) }
};
 
String readEndpoint(SerialComm& endpoint) {
  if (endpoint.RxAddress == 0)
    return "Endpoint can only be written";
  Serial.write(endpoint.RxAddress);
  delay(1);
  long currentMillis = millis();
  byte buff[64];
  byte count = 0;
  while (millis() - currentMillis < 15) {
    if (Serial.available() > 0)
      buff[count++] = Serial.read();
    if (count == endpoint.BufferSize) {
      if (endpoint.SetBuffer(buff))
        return "OK";
      else
        return "Bad CRC";
    }
  }
  return "Read timeout";
}

String readEndpointWithRetry(SerialComm& endpoint) {
  byte retryCount = 0;
  String result;
  while (retryCount < 3) {
    result = readEndpoint(endpoint);
    if (result.equals("OK"))
      return result;
    else
      retryCount++;
  }
  return result;
}

void writeEndpoint(SerialComm& endpoint) {
  if (endpoint.TxAddress == 0)
    return;
  Serial.write(endpoint.TxAddress);
  delay(1);
  byte buff[64];
  endpoint.GetBuffer(buff);
  Serial.write(buff, endpoint.BufferSize);
  Serial.flush();
}

String getRoomParams(RoomParams& result, const JsonObject& root) {
  String room = root["room"];
  result.roomID = 8;
  for (byte i = 0; i < 8; i++) {
    if (rooms[i].equals(room)) {
      result.roomID = i;
      break;
    }
  }
  if (result.roomID == 8)
    return "Unknown room";
  switch (result.roomID) {
    case 0: //Kitchen
      result.controllerID = 0;
      result.linkID = 0;
      result.channels[0] = 7;
      result.channels[1] = 6;
      result.channels[2] = 5;
    break;
    case 1: //Room A
      result.controllerID = 0;
      result.linkID = 1;
      result.channels[0] = 4;
      result.channels[1] = 3;
      result.channels[2] = 2;
    break;
    case 2: //Room B
      result.controllerID = 1;
      result.linkID = 0;
      result.channels[0] = 7;
      result.channels[1] = 6;
      result.channels[2] = 5;
    break;
    case 3: //Room C
      result.controllerID = 1;
      result.linkID = 1;
      result.channels[0] = 4;
      result.channels[1] = 3;
      result.channels[2] = 2;
    break;
    case 4: //Table
      result.controllerID = 0;
      result.linkID = 3;
      result.channels[0] = 8;
      result.channels[1] = 9;
      result.channels[2] = 9;
    break;
    case 5: //Shower
      result.controllerID = 1;
      result.linkID = 3;
      result.channels[0] = 8;
      result.channels[1] = 9;
      result.channels[2] = 9;
    break;
    case 6: //Toilet
      result.controllerID = 0;
      result.linkID = 2;
      result.channels[0] = 0;
      result.channels[1] = 9;
      result.channels[2] = 9;
    break;
    case 7: //Stairs
      result.controllerID = 1;
      result.linkID = 2;
      result.channels[0] = 1;
      result.channels[1] = 0;
      result.channels[2] = 9;
    break;
  }
  return "OK";
}

String getAreaID(byte& areaID, const JsonObject& root) {
  String area = root["area"];
  for (byte i = 0; i < 4; i++) {
    if (areas[i].equals(area)) {
      areaID = i;
      return "OK";
    }
  }
  return "Unknown area";
}

String getDataTypeID(byte& dtypeID, const JsonObject& root) {
  String dtype = root["dtype"];
  for (byte i = 0; i < 7; i++) {
    if (dtypes[i].equals(dtype)) {
      dtypeID = i;
      return "OK";
    }
  }
  return "Unknown data type";
}

String deserialize(JsonObject& root) {
  if (server.hasArg("plain") == false)
    return "Body was not received";
  DeserializationError err = deserializeJson(jsonBuffer, server.arg("plain"));
  if (err)
    return "Deserialization error: " + String(err.c_str());
  root = jsonBuffer.as<JsonObject>();
  return "OK";
}

void sendLight() { //post getl
  JsonObject root;
  String result = deserialize(root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  RoomParams params;
  result = getRoomParams(params, root); 
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  byte areaID;
  result = getAreaID(areaID, root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  
  result = readEndpointWithRetry(floors[params.controllerID].state);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  result = readEndpointWithRetry(floors[params.controllerID].conf); 
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  
  root = jsonBuffer.to<JsonObject>(); //Clear
  if (areaID < 3) { //Lamp A, B or C 
    result = readEndpointWithRetry(floors[params.controllerID].chOTGet);
    if (!result.equals("OK")) {
      server.send(503, "text/plain", result);
      return;
    }
    root["lightLvl"] = floors[params.controllerID].state.GetChLevel(params.channels[areaID]);
    root["minLightLvl"] = floors[params.controllerID].conf.MinLvlGet(params.channels[areaID]);
    root["lampOT"] = floors[params.controllerID].chOTGet.GetOnTime(params.channels[areaID]);
    root["lampCycles"] = floors[params.controllerID].chOTGet.GetSwCount(params.channels[areaID]);
  } else { //Group
    root["lightLvl"] = floors[params.controllerID].state.GetLinkLevel(params.linkID);
    root["minLightLvl"] = floors[params.controllerID].conf.MinLvlGet(params.channels[0]);
    root["fadeRate"] = floors[params.controllerID].conf.FadeRateGet(params.linkID);
    root["fadeDelay"] = floors[params.controllerID].conf.LinkDelayGet(params.linkID);
    root["msllt"] = floors[params.controllerID].conf.MSenLowTimeGet();
    root["msenState"] = floors[params.controllerID].state.MSenStateGet();
  }
  result = "";
  serializeJson(root, result);
  server.send(200, "application/json", result);
}

void updateLight() { //post setl
  JsonObject root;
  String result = deserialize(root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  RoomParams params;
  result = getRoomParams(params, root); 
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  byte areaID;
  result = getAreaID(areaID, root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  byte dtypeID;
  result = getDataTypeID(dtypeID, root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  byte dval = root["dval"];
  
  result = readEndpointWithRetry(floors[params.controllerID].conf); 
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  switch (dtypeID) {
    case 0: //Light level
      if (areaID < 3) { //Lamp A, B or C
        byte x = params.channels[areaID];
        byte channels[3] = { x, x, x };
        floors[params.controllerID].conf.OverrideLvlSet(channels, dval);
      } else { //Group
        floors[params.controllerID].conf.OverrideLvlSet(params.channels, dval);
      }
    break;
    case 1: //Min light level
      if (areaID < 3) { //Lamp A, B or C
        floors[params.controllerID].conf.MinLvlSet(params.channels[areaID], dval);
      } else { //Group
        for (byte i = 0; i < 3; i++)
          floors[params.controllerID].conf.MinLvlSet(params.channels[i], dval);
      }   
    break;
    case 2: //Fade rate
      floors[params.controllerID].conf.FadeRateSet(params.linkID, dval);
    break;
    case 3: //Fade delay
      floors[params.controllerID].conf.LinkDelaySet(params.linkID, dval);
    break;
    case 4: //Motion sensor low level time
      floors[params.controllerID].conf.MSenLowTimeSet(dval);
    break;
  }
  delay(1);
  writeEndpoint(floors[params.controllerID].conf);
  server.send(200, "text/plain", "OK");
}

void sendVentilation() { //get getv
  JsonObject root;
  root = jsonBuffer.to<JsonObject>(); //Clear
  
  String result = readEndpointWithRetry(ShowerState); 
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  result = readEndpointWithRetry(ShowerConfig);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  
  root["ventFRPM"] = ShowerState.RPMFront();
  root["ventRRPM"] = ShowerState.RPMRear();
  root["ventI"] = ShowerState.CurrentDraw();
  root["ventT"] = ShowerState.GetTemperature();
  root["ventRH"] = ShowerState.GetRH();
  root["minRH"] = ShowerConfig.MinRHGet();
  root["fanLvl"] = ShowerState.FanLevel();
  result = "";
  serializeJson(root, result);
  server.send(200, "application/json", result);
}

void updateVentilation() { //post setv
  JsonObject root;
  String result = deserialize(root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  byte dtypeID;
  result = getDataTypeID(dtypeID, root);
  if (dtypeID < 5)
    result = "Unknown data type";
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }

  byte dval = root["dval"];
  result = readEndpointWithRetry(ShowerConfig);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  if (dtypeID == 5) //Set Fan Level
    ShowerConfig.FanLevelSet(dval);
  else
    ShowerConfig.MinRHSet(dval);
  delay(1);
  writeEndpoint(ShowerConfig);
  server.send(200, "text/plain", "OK");
}

void sendStatistics() { //get getc
  JsonObject root;
  root = jsonBuffer.to<JsonObject>(); //Clear
  
  String result = readEndpointWithRetry(floors[0].state);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  result = readEndpointWithRetry(floors[1].state);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  
  root["contr1OT"] = floors[0].state.GetTicks();
  root["contr2OT"] = floors[1].state.GetTicks();
  result = "";
  serializeJson(root, result);
  server.send(200, "application/json", result);
}

void resetLightTimer() { //post resh
  JsonObject root;
  String result = deserialize(root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  RoomParams params;
  result = getRoomParams(params, root); 
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  byte areaID;
  result = getAreaID(areaID, root);
  if (!result.equals("OK")) {
    server.send(503, "text/plain", result);
    return;
  }
  String request = root["code"];
  if (request.equals(resetTimerRequest)) {
    floors[params.controllerID].chOTSet.ReplaceChOnTime(params.channels[areaID], 0, 0);
    writeEndpoint(floors[params.controllerID].chOTSet);
    server.send(200, "text/plain", "OK");
  } else {
    server.send(503, "text/plain", "Unknown operation code");
  }
}

void sendNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  pinMode(D0, OUTPUT); //Pink led
  digitalWrite(D0, HIGH); 
  Serial.begin(76400);

  // WiFiManager. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name "AutoConnectAP"
  // and goes into a blocking loop awaiting configuration
  wifiManager.setConfigPortalTimeout(180);
  if(!wifiManager.autoConnect("AutoConnectAP")) {
    ESP.reset(); //reset and try again
    delay(5000);
  }
  digitalWrite(D0, LOW);
  
  SPIFFS.begin();
  MDNS.begin("iot");

  server.on("/getl", sendLight);
  server.on("/setl", updateLight);
  server.on("/getv", sendVentilation);
  server.on("/setv", updateVentilation);
  server.on("/getc", sendStatistics);
  server.on("/resh", resetLightTimer);
  server.onNotFound(sendNotFound);
  
  server.serveStatic("/js", SPIFFS, "/js");
  server.serveStatic("/", SPIFFS, "/index.html");

  server.begin();
  MDNS.addService("http", "tcp", 80);
 
  wifi_set_sleep_type(MODEM_SLEEP_T); 
}

void loop() {
  // put your main code here, to run repeatedly:
  //MDNS.update();
  server.handleClient();
  delay(33);
}
