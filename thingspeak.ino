#include <SPI.h>
#include <dht.h>     
#define dht_dpin A0              //定義訊號要從Pin A8 進來  
#include <Wire.h>  // Arduino IDE 內建

dht DHT;   

#define DEBUG true
// Wi-Fi接入點的SSID和密碼
const  char*  ssid  =  "Your SSID";
const  char*  password  =  "Your WIFI Password";

//const char* host = "184.106.153.149";
//const char* host = "api.thingspeak.com";
const char* host = "35.171.214.17";
const char* Write_API_Key = "Your WIFI Write_API_Key";

const int httpsPort = 443;
const int httpPort = 80;

bool F = false;
String sendData(String command, const int timeout, boolean debug){
    String response = "";
    Serial1.print(command); // send the read character to the esp8266
    long int time = millis();
    while( (time+timeout) > millis()){
      while(Serial1.available()){
        // The esp has data so display its output to the serial window 
        char c = Serial1.read(); // read the next character.
        response+=c;
      }  
    }
    if(DEBUG)    {
Serial.print(response);
    }
    return response;
}

void setup(){  
 
  Serial1.begin(115200);//RX1 TX1，即 Pin18 及 Pin19 
  Serial.begin(115200);
  while (!Serial) ;       // 等待序列埠開啟完成
  
  sendData("AT+RST\r\n",2000,DEBUG);
  sendData("AT+CWMODE=3\r\n",200,DEBUG);
  String commend = "AT+CWJAP=\"";
  commend += ssid;
  commend += "\",\"";
  commend += password;
  commend += "\"\r\n";
  if (sendData(commend,5000,DEBUG)){  
    Serial.print("Join AP success\r\n");
    Serial.print("IP:");
    sendData("AT+CIFSR\r\n",5000,DEBUG);
    sendData("AT+CIPSERVER=0\r\n",200,DEBUG); 
    sendData("AT+CIPMUX=0\r\n",200,DEBUG);
    sendData("AT+CIFSR\r\n",5000,DEBUG);
    sendData("AT+CIFSR\r\n",5000,DEBUG);
    
  }
}
void loop(){   
  DHT.read11(dht_dpin);                    //去library裡面找DHT.read11  
    
  String commends = "AT+CIPSTART=\"TCP\",\"";
  commends += host;
  commends += "\",";
  commends += httpPort; //80
  commends += "\r\n";
  sendData(commends,1000,DEBUG);
  
  String GET = "GET /update?api_key=";
  GET += Write_API_Key;
  GET += "&field1=";
  GET += DHT.temperature;
  GET += "&field2=";
  GET += DHT.humidity;
  GET += " HTTP/1.1\r\nHost:";
  GET += host;
  GET += "\r\n\r\n";
  Serial.println(GET);
  String cipSend = "AT+CIPSEND=";

  cipSend += GET.length();
  cipSend += "\r\n";
  sendData(cipSend,500,DEBUG);
  sendData(GET,500,DEBUG);
  delay(200);
  sendData("AT+CIPCLOSE\r\n",500,DEBUG);
  delay(1000);   
  sendData("AT+CIFSR\r\n",5000,DEBUG); 
  delay(15000);
  //每1000ms更新一次   
} 
