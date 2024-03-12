
#include <ESP8266WiFi.h>
#include <Arduino.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <typeinfo>
#include <PubSubClient.h>
#include <WiFiClientSecure.h>
#include <time.h>
#include <TimeLib.h>
#include <TZ.h>

#define EEPROM_SIZE 512
#define OPEN 1
#define CLOSE -1
#define PARK 0

#define MASKLENGTH 3

/* Put your SSID & Password */
const char* ssid = "Gate";  // Enter SSID here
const char* password = "12345678";  //Enter Password here
String wssid;
String wpass;

const char* mqtt_server = "51.20.133.218";
const int mqtt_port = 61616;
const char* mqtt_user = "artemis";
const char* mqtt_password = "aalbot";
const char* clientID = "fhd-test";

const char* topic = "RMTxTestV1-000111";
const char* dTopic = "CTRLxTestV1-000111"; 

const char* id="LATEST";

unsigned long LDODelay; 
unsigned long RDODelay;
unsigned long LDCDelay; 
unsigned long RDCDelay;
unsigned int Speed=0;
/* Put IP Address details */
IPAddress local_ip(192,168,1,25);
IPAddress gateway(192,168,1,25);
IPAddress subnet(255,255,255,0);

const int leftMotorPWM = D0;                   //Left Motor 
const int leftMotorDIR = D1;                   //Left motor
const int rightMotorPWM = D2;                   //Right Motor 
const int rightMotorDIR = D3;                   //Right Motor    

const int lockMotorT1 = D5;                   //Lock Motor 
const int lockMotorT2 = D6;                   //Lock motor

// const int interruptPin1 = D7;
const int interruptPin2 = D7; 
// const int interruptPin3 = 3;  //RX pin, D8 not working

const int LDOAddr = 0; 
const int RDOAddr = 4;
const int LDCAddr = 8; 
const int RDCAddr = 12;
const int WIFIMODEADDR = 16;

unsigned long LDTO;
unsigned long RDTO;
unsigned long LDTC;
unsigned long RDTC;

int WIFIMODE=-1;//0-AP 1-WIFI ONLY 2-WIFI+MQTT
int SYSSTAT=-1;//-1-note active,0-lock,1-leaf
long timePointer=0;
int signalFlag=-1;
int Pointer=-1;

bool Flag=false;
int Gravity=0;//-1 close,0 park,1 open
String STATE = "115A"; //005A-fullopened,115A-closed,003C-opened60(cycle)

int LeftGateTime=0;
int LeftGateDelay=0;
int RightGateTime=0;
int RightGateDelay=0;

int lockMotorDuration=1500; 

int LockStatus = -1;// -1-default state, 0-open ,1- close
int gateStatus=-1;// -1-default state, 0-open ,1- close,2-singleOpen,


ESP8266WebServer server(80);

WiFiClient espClient;
PubSubClient mqttClient(espClient);

ICACHE_RAM_ATTR void detectsMovement() {
  //TODO 
  delay(500);
  if(STATE=="115A"){  
     Serial.println("INTERRUPT SWITCH FOR OPENING");
     service("005A") ;   
   return;  
  }
  if(STATE=="005A"){ 
    Serial.println("INTERRUPT SWITCH FOR CLOSING"); 
    service("115A");
    return;
  }
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  EEPROM.begin(EEPROM_SIZE);
  //pinMode (leftMotorPWM, OUTPUT);
  pinMode (leftMotorDIR, OUTPUT);
 // pinMode (rightMotorPWM, OUTPUT);
  pinMode (rightMotorDIR, OUTPUT);
  pinMode (lockMotorT1, OUTPUT);
  pinMode (lockMotorT2, OUTPUT);
  pinMode(interruptPin2, INPUT_PULLUP); 
 
 // digitalWrite(leftMotorPWM, LOW);
  digitalWrite(leftMotorDIR, LOW);
 // digitalWrite(rightMotorPWM, LOW);
  digitalWrite(rightMotorDIR, LOW);
  digitalWrite(lockMotorT1, LOW);
  digitalWrite(lockMotorT2, LOW);
  //attachInterrupt(digitalPinToInterrupt(interruptPin2), detectsMovement, FALLING);
  Serial.println(">> setup");
  unsigned long timestamp = millis();  // Get the current timestamp in milliseconds
  Serial.println(timestamp);
  time_t currentTime = now();
  readWifiMode();
  delay(50);
  readDuration();
  readDelay();


  server.on("/v1", httpHandle);
  server.on("/v1/config",httpConfig);
  server.on("/v1/delay",httpDelay);
  server.on("/v1/Speed",httpSpeed);
  server.on("/v1/configWifi",httpConfigwifi);
  server.on("/v1/wifimode",httpWifiMode);

  server.begin();
  Serial.println("HTTP server started");
  //connect Mqtt
  if(WIFIMODE==2&&WiFi.status()== WL_CONNECTED){

      setDateTime();
      connectMqtt();
  }
}
void loop() {
  // put your main code here, to run repeatedly:
  server.handleClient();
  if(WIFIMODE==2){
    if((WiFi.status()== WL_CONNECTED)){
    if(mqttClient.connected()){
      mqttClient.loop();
    }else{      
        mqtt();
    }
    }
  }
  //DO WHAT WE
  if(Gravity!=0){   
    if(Flag){
      Serial.println("initiating render");
      timePointer=millis();
      //unlock 
      if(Gravity==1 && LockStatus!=0){
        lockmotorReverse();
        LockStatus=0;
      } //lock
      else if(Gravity==-1 && LockStatus!=1){
        lockmotorForward();
        LockStatus=1;
      }
      Flag=false;
      SYSSTAT=0;
    }else{
      long CT=millis()-timePointer;
      if(SYSSTAT==0){
        if(CT>=lockMotorDuration){
          lockmotorStop();
          //reset time pointer  - start time for opening and close
          timePointer=millis();
          SYSSTAT=1;
        }
      }else if(SYSSTAT==1){
        Serial.println("SYSSTAT => 1 -> in motor section");
        bool LMIsOn=true;
        bool RMIsOn=true;
        if(CT>=(LeftGateDelay+LeftGateTime)){
          M(false,leftMotorPWM,leftMotorDIR);
          LMIsOn=false;
          Serial.println("SYSSTAT => 1 -> leftfinished");
        }else if(CT>=LeftGateDelay){
          M(true,leftMotorPWM,leftMotorDIR);
          LMIsOn=true;
           Serial.println("SYSSTAT => 1 -> leftrunning");
        }
        if(CT>=(RightGateDelay+RightGateTime)){
          M(false,rightMotorPWM,rightMotorDIR);
          RMIsOn=false;
          Serial.println("SYSSTAT => 1 -> rightfinished");
        }else if(CT>=RightGateDelay){
          M(true,rightMotorPWM,rightMotorDIR);
          RMIsOn=true;
          Serial.println("SYSSTAT => 1 -> rightrunning");
        }
        if(!(LMIsOn ||RMIsOn) ){
        
          M(LOW,leftMotorPWM,leftMotorDIR);
          M(LOW,rightMotorPWM,rightMotorDIR);
          SYSSTAT=-1;
          Gravity=0;
          Serial.println("gravity => parked");
        }
      }
    }
  }
}
void setDateTime() {
  // You can use your own timezone, but the exact time is not used at all.
  // Only the date is needed for validating the certificates.
  configTime(TZ_Europe_Berlin, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    delay(100);
    Serial.print(".");
    now = time(nullptr);
  }
  Serial.println();
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  Serial.printf("%s %s", tzname[0], asctime(&timeinfo));
}
void AccessPointWifi(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("Access Point IP:");
  Serial.println(myIP);
  delay(50);
}
void Homewifi(){
  unsigned long startTime = millis();
  const unsigned long loopDuration = 10000;
  wssid = read_String(35);
  wpass = read_String(75);
  WiFi.mode(WIFI_STA);
  Serial.println("HomewiFi function Credentials.");
  Serial.println(wssid);
  Serial.println(wpass);
  if(WIFIMODE != 2){
    WiFi.config(local_ip, gateway, subnet);
  }
  WiFi.begin(wssid,wpass);
  while (WiFi.status()!= WL_CONNECTED)
  {
    Serial.println("Wifi not connected."); 
     delay(500);
     if (millis() - startTime >= loopDuration) {
        AccessPointWifi();
        break; 
      }
  }

  if(WiFi.status()== WL_CONNECTED)
  {
  Serial.println("WiFi Connected.");
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  delay(500);
  }
}
void connectMqtt(){
 // delay(2000);
 // #ifdef ESP8266
   // espClient.setInsecure();
  // #else
  //   espClient.setCACert(root_ca);      // enable this line and the the "certificate" code for secure connection
  // #endif
  
  mqttClient.setServer(mqtt_server,mqtt_port);
  mqttClient.setCallback(callback);
  mqtt();
}
void mqtt(){
    Serial.println(" Trying Mqtt.....");  
    if(mqttClient.connect(clientID,mqtt_user,mqtt_password)){
      Serial.println("Mqtt Connected.");
      mqttClient.subscribe(topic);
    }else{
       Serial.println("Mqtt Not Connected.");
    }
    delay(2000);
}
void callback(char* topics, byte* payload, unsigned int length){ 
    bool f=true;
    payload[length] = '\0';
    String value=String((char*) payload);    
   //TODO
    Serial.println(value);

    if(value=="00FF"){
        String wifiIP = WiFi.localIP().toString();
        mqttClient.publish(dTopic, ("{\"IP\":\"" + wifiIP + "\"}").c_str()); 
    }else{
      if(service(value)){
         mqttClient.publish(dTopic, "{\"code\":\"success\"}");
      }else{
        mqttClient.publish(dTopic, "{\"code\":\"error\"}");
      }
    }
} 
void lockmotorForward(){
    //lock [forward] ->
    Serial.println("Lock Motor Forward ie CLOSE ");
    digitalWrite(lockMotorT1, LOW);  
    digitalWrite(lockMotorT2, HIGH);  
}
void lockmotorReverse(){
    Serial.println("Lock Motor reverse ie OPEN ");
    digitalWrite(lockMotorT1, HIGH);  
    digitalWrite(lockMotorT2, LOW);
}
void lockmotorStop(){
   digitalWrite(lockMotorT1, LOW);  
   digitalWrite(lockMotorT2, LOW);
}
void startFrame(String signal,int gravity,int LDelay,int LDuration,int RDelay,int RDuration){
  Serial.println(">>startFrame");
  Gravity=gravity;
  STATE=signal;
  Serial.println("Sent Signal");
  Serial.println(STATE);

  LeftGateDelay=LDelay;
  LeftGateTime=LDuration;
  RightGateDelay=RDelay;
  RightGateTime=RDuration;
  Flag=true;
  SYSSTAT=-1;
}
void M(bool power,int Motor,int DIR){
  if(power){
    if(Gravity==OPEN){
       
      digitalWrite(DIR,HIGH);
      analogWrite(Motor,Speed);
    }else if(Gravity==CLOSE){
   
      digitalWrite(DIR,LOW);
      analogWrite(Motor,Speed);
     
    }
  }else{
     
     

     analogWrite(Motor,0);
  }
}
bool service(String signal){
   //TODO
   
    if(signal=="00AA"){ 
      //stop
      Gravity=PARK;
      M(LOW,leftMotorPWM,leftMotorDIR);
      M(LOW,rightMotorPWM,rightMotorDIR);
    }
    else if(signal=="003C"){
       //open6
       startFrame(signal,OPEN,LDODelay,LDTO,0,0);   
    }else if(signal=="005A"){
      //open
      startFrame(signal,OPEN,LDODelay,LDTO,RDODelay,RDTO);   
    }else if(signal=="111E"){
      signalFlag=7; 
    }else if(signal=="113C"){
      signalFlag=8;
      Serial.println("Resetting NodeMCU...");
      ESP.restart();   
    }else if(signal=="115A"){
      //close
       if(STATE=="003C"){
          Serial.println("LEFT CLOSE");
          startFrame(signal,CLOSE,LDCDelay,LDTC,0,0);
      }else{
          Serial.println("RIGHT CLOSE(full)");
          startFrame(signal,CLOSE,LDCDelay,LDTC,RDCDelay,RDTC);
      }
    }else if(signal=="AB05"){
      signalFlag=10; //closeintime
    }else{
     return false;
    }
    return true;
}
void httpHandle() {
  String msg="{\"code\":\"success\"}";
  int status=200;
  if(server.hasArg("msg")){
      String signal  = server.arg("msg");
      Serial.println(signal);
      if(!service(signal)){
        msg="{\"code\":\"error\"}";
        status=400;
      }
  }else{
    msg="{\"code\":\"error\"}'";
    status=400;
  } 
  server.send(status, "text/plain", msg);
}
void httpDelay(){
  String msg="{\"code\":\"success\"}";
  int status=200;
  if(server.hasArg("ldo")&&server.hasArg("ldc")&&server.hasArg("rdo")&&server.hasArg("rdc")){
      String ldo  = server.arg("ldo");
      String ldc  = server.arg("ldc");
      String rdo  = server.arg("rdo");
      String rdc  = server.arg("rdc");
      saveDelay(ldo,rdo,ldc,rdc);
      readDelay();
  }
  else{
    msg="{\"code\":\"error\"}'";
    status=400;
  }
  server.send(status, "text/plain", msg);
}
void httpSpeed(){
  String msg="{\"code\":\"success\"}";
  int status=200;
  if(server.hasArg("Speed")){
      String speed  = server.arg("Speed");     
      EEPROM.put(32, speed);
      EEPROM.commit();

      readSpeed();
  }
  else{
    msg="{\"code\":\"error\"}'";
    status=400;
  }
  server.send(status, "text/plain", msg);
}
void saveDelay(String ldo,String rdo,String ldc,String rdc){
  LDODelay = atol(ldo.c_str());
  RDODelay = atol(rdo.c_str());
  LDCDelay = atol(ldc.c_str());
  RDCDelay = atol(rdc.c_str());
  EEPROM.put(18, LDODelay);
  EEPROM.put(22, RDODelay);
  EEPROM.put(26, LDCDelay);
  EEPROM.put(30, RDCDelay);
  EEPROM.commit();
}
void readSpeed(){
   Serial.println("READ speed:");
  EEPROM.get(32, Speed);
  Serial.print(Speed);
}
void readDelay(){
  Serial.println(">>readDelay");
  EEPROM.get(18, LDODelay);
  EEPROM.get(22, RDODelay);
  EEPROM.get(26, LDCDelay);
  EEPROM.get(30, RDCDelay);
  Serial.println("READ DELAY function delays:");
  Serial.print("LDODelay");
  Serial.println(LDODelay);
  Serial.print("RDODelay");
  Serial.println(RDODelay);
  Serial.print("LDCDelay");
  Serial.println(LDCDelay);
  Serial.print("RDCDelay");
  Serial.println(RDCDelay);
}
void httpConfig(){
  String msg="{\"code\":\"success\"}";
  int status=200;
  if(server.hasArg("lo")&&server.hasArg("lc")&&server.hasArg("ro")&&server.hasArg("rc")){
      String lo  = server.arg("lo");
      String lc  = server.arg("lc");
      String ro  = server.arg("ro");
      String rc  = server.arg("rc");
      saveDuration(lo,ro,lc,rc);
      readDuration();
  }
  else{
    msg="{\"code\":\"error\"}'";
    status=400;
  }
  server.send(status, "text/plain", msg);
}
void httpConfigwifi(){
  String msg="{\"code\":\"success\"}";
  int status=200;
  if(server.hasArg("user")&&server.hasArg("pass")){
    String user = server.arg("user");
    String pass = server.arg("pass");
    writeString(35,user);
    delay(50);
    writeString(75,pass);
    wssid = read_String(35);
    wpass = read_String(75);
    Serial.println("wssid from eeprom");
    Serial.println(wssid);
    Serial.println("wpass from eeprom");
    Serial.println(wpass);
  }
  else{
    msg="{\"code\":\"error\"}";
    status=400;
  }
  server.send(status, "text/plain", msg);
}
void httpWifiMode(){
  String msg="{\"code\":\"success\"}";
  int status=200;
  if(server.hasArg("mode")){
      String wifimode  = server.arg("mode");
      saveWifiMode(wifimode);
  }
  else{
    msg="{\"code\":\"error\"}'";
    status=400;
  }
  server.send(status, "text/plain", msg);
}
void saveWifiMode(String wifimode){
  WIFIMODE = atol(wifimode.c_str());
  EEPROM.put(WIFIMODEADDR, WIFIMODE);
  EEPROM.commit();
  delay(50);
  Serial.println("Resetting NodeMCU...");
  ESP.restart();
}
void readWifiMode(){
  EEPROM.get(WIFIMODEADDR, WIFIMODE);
  Serial.println(">>Read WIFI MODE");
  Serial.println(WIFIMODE);
  if(WIFIMODE==0||WIFIMODE==-1){
    WiFi.disconnect();
    AccessPointWifi();
  }else{
    WiFi.disconnect();
    Homewifi();
  }
}
void saveDuration(String ldto,String rdto,String ldtc,String rdtc){
  LDTO = atol(ldto.c_str());
  RDTO = atol(rdto.c_str());
  LDTC = atol(ldtc.c_str());
  RDTC = atol(rdtc.c_str());
  EEPROM.put(LDOAddr, LDTO);
  EEPROM.put(RDOAddr, RDTO);
  EEPROM.put(LDCAddr, LDTC);
  EEPROM.put(RDCAddr, RDTC);
  EEPROM.commit();
}
void readDuration(){
  Serial.println(">>readDuration");
  EEPROM.get(LDOAddr, LDTO);
  EEPROM.get(RDOAddr, RDTO);
  EEPROM.get(LDCAddr, LDTC);
  EEPROM.get(RDCAddr, RDTC);
  Serial.println(LDTO);
  Serial.println(RDTO);
  Serial.println(LDTC);
  Serial.println(RDTC);
}
void writeString(int add,String data)
{
  int _size = data.length();
  int i;
  for(i=0;i<_size;i++)
  {
    EEPROM.write(add+i,data[i]);
  }
  EEPROM.write(add+_size,'\0');   //Add termination null character for String Data
  EEPROM.commit();
}
String read_String(int add)
 {
  int i;
  char data[100]; //Max 100 Bytes
  int len=0;
  unsigned char k;
  k=EEPROM.read(add);
  while(k != '\0' && len<500)   //Read until null character
  {  
    k=EEPROM.read(add+len);
    data[len]=k;
    len++;
  }
  return String(data);
}
