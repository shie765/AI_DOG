#include <dht.h>     
#define dht_dpin A0 	           //�w�q�T���n�qPin A0 �i��  
dht DHT;   

String a;
String Sch_no = "123";

void setup(){  
  Serial.begin(9600);   
  delay(300);                                            //Let system settle   
  Serial.println("Humidity and temperature");   
  delay(700);                       //Wait rest of 1000ms recommended delay before   
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop(){   
  while(Serial.available()) {
    a = Serial.readString();// read the incoming data as string
    if (a == Sch_no) {
      DHT.read11(dht_dpin);                    //�hlibrary�̭���DHT.read11  
      Serial.print("Humidity = ");   
      Serial.print(DHT.humidity);   
      Serial.print("% ");   
      Serial.print("temperature = ");   
      Serial.print(DHT.temperature);   
      Serial.println("C ");   
      delay(1000);                                       //�C1000ms��s�@�� 
      digitalWrite(LED_BUILTIN, LOW);
    }
    else
      digitalWrite(LED_BUILTIN, HIGH); 
  }
} 
