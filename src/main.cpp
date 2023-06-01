#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#include <Nextion.h>
#include <JuncTek_BatteryMonitor.h>
// #define nexSerial uart_DISP

//Display
//16 is RX_GUI,17 is TX_GUI 
SoftwareSerial uart_DISP(21, 22); // RX, TX
NexNumber DispPSpeed = NexNumber(1,11,"x0");    //Pulse Speed
NexNumber DispGSpeed = NexNumber(1,12,"x1");    //GPS Speed
NexNumber DispETime = NexNumber(1,13,"x2");     //Elapsed Time
NexNumber DispVoltage = NexNumber(1,14,"x3");   //Voltage
NexNumber DispCurrent = NexNumber(1,15,"x4");   //Current
NexNumber DispPower = NexNumber(1,16,"x5");     //Power
NexNumber DispIPower = NexNumber(1,17,"x6");    //IntegratedPower
NexText DispGPSTime = NexText(1,19,"t9");       //GPSTime

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

// void getBattMonValue()
// {
//   uart_BATT.write(":R50=1,2,1,\r\n"); //Send Command
//   String _message = "";
//   char c;
//   while(uart_BATT.available())  //Wait Data
//   {
//     c=uart_BATT.read();         //Receive
//     _message = _message+String(c);
//   }
//   Serial.println(_message);

// }


void setup(){
    //DebugConsole
    Serial.begin(115200);

    //Display Setup
    nexSerial.begin(9600);  // Display

    //BatteryMonitor Setup
    uart_BATT.setTransmitEnablePin(21);
    uart_BATT.begin(115200); // BatteryMonitor SerialPort
    // uart_BATT.setHwFlowCtrlMode(HW_FLOWCTRL_RTS);
    // uart_BATT.setPins(16,17,-1,21); //RX:16,TX:17,RTS:21
    // uart_BATT.begin(115200,SWSERIAL_8N1,16,17);
    // uart_BATT.setTransmitEnablePin(21);
    bm.begin(1,uart_BATT);
    
    // bm.begin(1,Serial1);
    // pinMode(TX_BAT_EN,OUTPUT);
    // while(!uart_BATT){}
    // digitalWrite(TX_BAT_EN,LOW);    //disable DE

    //VESC SetUP

    //SD Setup

    //GPS Setup
    //Connection 赤色=VCC、黒色=GND、橙色=TXD、緑色=RXD、茶色=1pps
    uart_GPS.begin(9600,SERIAL_8N1,14,4);
    // uart_GPS.listen();
}
    double BattVoltage = bm.getVoltage();
    double BattCurrent = bm.getCurrent();
void loop(){

    //GPS
    char buf[64];
    bool gpsUpdated = false;
    while (gpsUpdated == false){
        while(uart_GPS.available() > 0 ){
          char c = uart_GPS.read();
          // Serial.print(c);
          gps.encode(c);
          gpsUpdated = gps.time.isUpdated();
        }
    }
    sprintf(buf,"%02d:%02d:%02d speed %2.1f, heading to %3f",gps.time.hour()+9,gps.time.minute(),gps.time.second(),gps.speed.kmph(),gps.course.deg());

    // Serial.print(buf);
    //BatteryMonitor
    uart_BATT.listen();
    // uart_BATT.flush();
    // getBattMonValue();

    double BattVoltage = bm.getVoltage();
    double BattCurrent = bm.getCurrent();
    double BattPower = BattVoltage * BattCurrent;
    DispVoltage.setValue((uint32_t)(floor(BattVoltage*100.0 )));

    DispGPSTime.setText(buf);
    Serial.printf("V=%2.1f,A=%f,P=%f, %s\n",BattVoltage,BattCurrent,BattPower,buf);
    // Serial.printf("%s\n",buf);

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