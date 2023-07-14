#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Nextion.h>
#include <JuncTek_BatteryMonitor.h>
#include "dischargeCurveForWEM.h"
#include <SD.h>
// #define nexSerial uart_DISP

//Display
// SoftwareSerial uart_DISP(21, 22); // RX, TX
HardwareSerial uart_DISP(2);
// NexText DispPSpeed = NexText(1,11,"x0");    //Pulse Speed
NexText DispGSpeed = NexText(1,12,"SPD");    //GPS Speed
NexText DispETime = NexText(1,13,"ETIME");     //Elapsed Time
NexText DispVoltage = NexText(1,14,"BV");   //Voltage
NexText DispCurrent = NexText(1,15,"BA");   //Current
NexText DispPower = NexText(1,16,"BP");     //Power
NexText DispICurrent = NexText(1,17,"AH");    //IntegratedCurrent
NexText DispGPSTime = NexText(1,19,"TIME");       //GPSTime
NexGauge DispPace = NexGauge(1,2,"PACE");
NexText DispIM05V = NexText(1,20,"IM05V");
NexText DispIP05V = NexText(1,21,"IP05V");



//BatteryMonitor
//TODO use hardwareserial and change connection
SoftwareSerial uart_BATT(16, 17); // RX, TX
// HardwareSerial uart_BATT(1);
// uint8_t TX_BAT_EN = 13;
BatteryMonitor bm;
// char query[] = ":R50=1,2,1,\r\n";

//GPS
// SoftwareSerial uart_GPS(14);  //RxOnly
HardwareSerial uart_GPS(1);
TinyGPSPlus gps;

//VESC
SoftwareSerial uart_VESC(26,27);

//SD
int fileName = 0;
char fineNameCHR[16];
File file;
char fileHeader[] = {"Time,Etime,Latitude,Longitude,Altitude,Direction,Speed,IdealVoltage,Voltage,Current,Power,DischargeCapacity"};
bool enableRecord = false;

void setup(){
  //DebugConsole
  Serial.begin(115200);

  //Display Setup
  // nexSerial.begin(115200);  // Display
  nexSerial.begin(115200,SERIAL_8N1,21,22);

  //BatteryMonitor Setup
  uart_BATT.setTransmitEnablePin(13);
  uart_BATT.begin(115200); // BatteryMonitor SerialPort
  bm.begin(1,uart_BATT);
  
  //VESC SetUP

  //SD Setup
  if(SD.begin()){
    for(fileName = 0; fileName < 100;fileName++){
      sprintf(fineNameCHR,"/%d.csv",fileName);
      if(!SD.exists(fineNameCHR))break;
    }
    Serial.printf("FILE:%s",fineNameCHR);
    file = SD.open(fineNameCHR,FILE_WRITE);
    if(file){
      enableRecord = true;
      file.println(fileHeader);
      file.close();
    }
    else{
      Serial.printf("ERROR File Open");
    }
  }
  // else{
  //   DispPower.setText("NO SD");
  //   while(1){

  //   }
  // }
  //GPS Setup
  //Connection 赤色=VCC、黒色=GND、橙色=TXD、緑色=RXD、茶色=1pps
  uart_GPS.begin(9600,SERIAL_8N1,14,4);
}

double BattVoltage;
double BattCurrent;
double IntCurrent;
double VoltageBuf;
int ElapsedTime = 0;
char bufTime[16];
char bufETime[16];
char bufSpeed[16];
char bufBattV[16];
char bufBattA[16];
char bufBattP[16];
char bufBattIA[16];
char bufIM05V[16];
char bufIP05V[16];
double im05v,ip05v;

void loop(){

  long time_Start = millis();

  //GPS
  bool gpsUpdated = false;
  while (gpsUpdated == false){
      while(uart_GPS.available() > 0 ){
        char c = uart_GPS.read();
        // Serial.print(c);
        gps.encode(c);
        gpsUpdated = gps.time.isUpdated();
      }
  }

  long time_gps = millis();

  //BatteryMonitor
  uart_BATT.listen();

  VoltageBuf = bm.getVoltage();
  //115200bpsのSoftwareSerialのため文字化けが頻発する
  //電圧が3.0以上の時更新する
  if(VoltageBuf >= 3.0){
    BattVoltage = bm.getVoltage();
    BattCurrent = bm.getCurrent();
  }

  double BattPower = BattVoltage * BattCurrent;
  IntCurrent+=BattCurrent;

  long time_bm = millis();

  sprintf(bufTime,"%02d:%02d:%02d",(gps.time.hour()+9)%24,gps.time.minute(),gps.time.second());
  sprintf(bufSpeed, "%2.1f",gps.speed.kmph());
  sprintf(bufBattV,"%2.1f",BattVoltage);
  sprintf(bufBattA,"%2.1f",BattCurrent);
  sprintf(bufBattP,"%3.1f",BattPower);
  sprintf(bufBattIA,"%2.1f",IntCurrent/3600);
  sprintf(bufETime,"%02d:%02d:%02d",ElapsedTime/3600,(ElapsedTime/60)%60,ElapsedTime%60);
  im05v = idealCurve[ElapsedTime]-0.5;
  ip05v = idealCurve[ElapsedTime]+0.5;
  sprintf(bufIM05V,"%2.1fV",im05v);
  sprintf(bufIP05V,"%2.1fV",ip05v);
  double VoltDiff = BattVoltage - idealCurve[ElapsedTime];

  long time_disp_sprint = millis();

  delay(10);  //delay 10ms for flush uart buffer...
  //DISPLAY
  // uart_DISP.listen();  //USE for SoftwareSerial
  DispGPSTime.setText(bufTime);
  DispGSpeed.setText(bufSpeed);
  DispVoltage.setText(bufBattV);
  DispCurrent.setText(bufBattA);
  DispPower.setText(bufBattP);
  DispICurrent.setText(bufBattIA);
  DispETime.setText(bufETime);
  DispIM05V.setText(bufIM05V);
  DispIP05V.setText(bufIP05V);
  if(BattVoltage <= im05v){
      DispPace.setValue(0);
  }
  else if(ip05v <= BattVoltage){
      DispPace.setValue(100);
  }
  else{
      DispPace.setValue((int)((BattVoltage - im05v) * 100));
  }

  long time_disp = millis();

  //Write to SD
  if(enableRecord){
    file = SD.open(fineNameCHR,FILE_APPEND);
    if(file){
      char buf[256];
      sprintf(buf,"%s,%s,%f,%f,%f,%f,%s,%f,%s,%s,%s,%s",
      bufTime,bufETime,gps.location.lat(),gps.location.lng(),gps.altitude.meters(),gps.course.deg(),bufSpeed,
      idealCurve[ElapsedTime],bufBattV,bufBattA,bufBattP,bufBattIA);
      file.println(buf);
      file.close();
    }
  }

  long time_sd = millis();

  ElapsedTime++;


  Serial.printf("GPS:%3ld,BM:%3ld,SPRINT:%3ld,DISP:%3ld,SD:%3ld\n",
  time_gps - time_Start,time_bm - time_gps,time_disp_sprint - time_bm,time_disp - time_disp_sprint,time_sd - time_disp);

  // uart_GPS.listen();
  // delay(500);
}


/*NextionHMI TEST*/
/*
NexNumber x0 = NexNumber(1,12,"x0");

void setup(){
// Serial.begin(9600);
nexSerial.begin(9600);
}

void loop(){
    uint32_t number;
    x0.getValue(&number);
    
    delay(1000);
    number += 1;
    
    x0.setValue(number);
}*/


/*Board test*/
/*
void setup() {
  // initialize digital pin 33 as an output.
  pinMode(33, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(33, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(33, LOW);   // turn the LED off by making the voltage LOW
  delay(1000);                      // wait for a second
}*/



/* *****GPS Sample******/
/*
SoftwareSerial gpsio(2,4);
TinyGPSPlus gpsParser;

void setup() {
  put your setup code here, to run once:
  gpsio.begin(9600);
  Serial.begin(9600);
}


void loop() {
  put your main code here, to run repeatedly:
  portTickType xLastWakeTime = xTaskGetTickCount();
  while(gpsio.available())
  {
    if(gpsParser.encode(gpsio.read())){
      Serial.println(gpsParser.satellites.value());
      gpsParser.time.
    }
  }
  Serial.println("ReceiveComplete or notAvailable");
  vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_RATE_MS );
}*/

/* *****KG-F RS485 loopback sample*****/
/*
SoftwareSerial uart_BATT(12, 13); // RX, TX
uint8_t TX_BAT_EN = 21;
BatteryMonitor bm;
void setup() {
  uart_BATT.begin(115200); // ソフトウェアシリアルの初期化
  bm.begin(1,uart_BATT);
  Serial.begin(9600);   //コンソールデバッガ用
  pinMode(TX_BAT_EN,OUTPUT);
  while(!uart_BATT){}
  uart_BATT.println("Hello, world?");
  digitalWrite(TX_BAT_EN,LOW);    //disable DE

}

void loop() {

    Serial.println(bm.getVoltage(),2);
    delay(500);
    // int incomingByte = 0;
    // if (uart_BATT.available()) { // reply only when you receive data
    //     incomingByte = uart_BATT.read(); // read the incoming byte
    //     digitalWrite(TX_BAT_EN,HIGH);    //enable DE
    //     uart_BATT.write(incomingByte);  // send the incoming byte
    //     digitalWrite(TX_BAT_EN,LOW);    //disable DE
    // }
}*/




// string getBattMonitor()
// {
//     string _message;
//     int t_start=millis();
//     while(!bm_serial->available() && millis()-t_start < SERIAL_TIMEOUT);
//     while(bm_serial->available() && _msgState != crlf){
//         c=bm_serial->read();
//         debug(c);
//         _message=_message+String(c);
//         //Serial.print(c);
//         if (c=='\r') {
//           _msgState=cr;
//           //Serial.print("{cr}");
//         }
//         if (_msgState==cr && c=='\n') _msgState=crlf;
        
//         //if (_msgState==reading) Serial.print("[r]");
//         //if (_msgState==cr) Serial.print("[CR]");
//         //if (_msgState==crlf) Serial.println("[CRLF]");
//     }
//     return _message;
// }