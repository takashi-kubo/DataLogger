
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