#define DEBUG true

const  char *  ssid  =  "ChienNan";
const  char *  password  =  "1234567890";

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

void setup()
{
  Serial.begin(115200);
  while(!Serial);
  Serial1.begin(115200);
  sendData("AT+RST\r\n",2000,DEBUG);
  sendData("AT+CWMODE=3\r\n",2000,DEBUG);
  String commend = "AT+CWJAP=\"";
  commend += ssid;  //"AP-SSID";
  commend += "\",\"";
  commend += password; //"Password";
  commend += "\"\r\n";
  if (sendData(commend,5000,DEBUG)){  
    Serial.print("Join AP success\r\n");
    Serial.print("IP:");
    sendData("AT+CIFSR\r\n",5000,DEBUG);
    sendData("AT+CIPSERVER=0\r\n",200,DEBUG);
    sendData("AT+CIPMUX=0\r\n",200,DEBUG);
  }
}

void loop()
{
  if(Serial1.available()){
    Serial.write(Serial1.read());
  }
  if(Serial.available()){
    Serial1.write(Serial.read());
  }
} 