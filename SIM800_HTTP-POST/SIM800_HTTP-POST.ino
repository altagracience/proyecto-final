//For more Information visit www.aeq-web.com?ref=arduinoide

#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial mySerial(3, 2);            // RX, TX Pins
String apn = "datos.personal.com";                       //APN
String apn_u = "gprs";                     //APN-Username
String apn_p = "adgj";                     //APN-Password
String url = "http://jammer-detector.ml/cargar_info.php";  //URL for HTTP-POST-REQUEST
String id, id2, id3;   //String for the first Paramter (e.g. Sensor1)
String ubicacion;   //String for the second Paramter (e.g. Sensor2)
String rssi, rssi2, rssi3;
String inhibicion, inhibicion2, inhibicion3;


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
  delay(10000);
}

void loop() { // run over and over
  
    id = "A";
    id2 = "B";
    id3 = "C";
    ubicacion = "PruebaZoom";
    rssi = "-90";
    rssi2 = "-80";
    rssi3 = "-70";
    inhibicion = "0";
    inhibicion2 = "0";
    inhibicion3 = "1";
    gsm_sendhttp(); //Start the GSM-Modul and start the transmisson
    delay(10000); //Wait one minute

}


void gsm_sendhttp() {
  
  
 
  mySerial.println("AT");
  runsl();//Print GSM Status an the Serial Output;
  delay(4000);
  mySerial.println("AT+SAPBR=3,1,Contype,GPRS");
  runsl();
  delay(100);
  mySerial.println("AT+SAPBR=3,1,APN," + apn);
  runsl();
  delay(100);
  mySerial.println("AT+SAPBR=3,1,USER," + apn_u); //Comment out, if you need username
  runsl();
  delay(100);
  mySerial.println("AT+SAPBR=3,1,PWD," + apn_p); //Comment out, if you need password
  runsl();
  delay(100);
  mySerial.println("AT+SAPBR =1,1");
  runsl();
  delay(100);
  mySerial.println("AT+SAPBR=2,1");
  runsl();
  delay(2000);
  mySerial.println("AT+HTTPINIT");
  runsl();
  delay(100);
  mySerial.println("AT+HTTPPARA=CID,1");
  runsl();
  delay(100);
  mySerial.println("AT+HTTPPARA=URL," + url);
  runsl();
  delay(100);
  mySerial.println("AT+HTTPPARA=CONTENT,application/x-www-form-urlencoded");
  runsl();
  delay(100);
  mySerial.println("AT+HTTPDATA=192,10000");
  runsl();
  delay(100);
  mySerial.println("params='" + id + "','" + id2 + "','" + id3 + "','" + ubicacion + "'," + rssi + "," + rssi2 + "," + rssi3 + "," + inhibicion + "," + inhibicion2 + "," + inhibicion3 + ")");
  runsl();
  delay(10000);
  mySerial.println("AT+HTTPACTION=1");
  runsl();
  delay(5000);
  mySerial.println("AT+HTTPREAD");
  runsl();
  delay(100);
  mySerial.println("AT+HTTPTERM");
  runsl(); 
}

//Print GSM Status
void runsl() {
  while (mySerial.available()) {
    Serial.write(mySerial.read());
  }

}
