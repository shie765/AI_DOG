//https://twgo.io/gxgmd
#include "esp_camera.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include <EEPROM.h>            // read and write from flash memory
#include <WiFi.h>
#include <WiFiClientSecure.h>

// ------ 以下修改成你的設定 ------
int vFlip = 0; //1=上下翻轉
int hMirror = 0; //1=左右翻轉

char* ssid = "AIOT2";
char* password = "0277388000";
//LineNotify密碼，請至Line官方取得，網址：https://notify-bot.line.me/zh_TW/
String myLineNotifyToken = "";
//超音波腳位
int trigPin = 15;               //請將Trig接GPIO15
int echoPin = 14;               //Echo Pin 接GPIO14

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  Serial.begin(115200);
  //初始化相機
  setupCam();
  //設定超音波腳位
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
}

void loop() {
  //WiFi連線
  if (WiFi.status() != WL_CONNECTED) WiFiConnect();
  //計算距離
  //digitalWrite(trigPin, LOW);
  //delayMicroseconds(5);
  //digitalWrite(trigPin, HIGH);     // 給 Trig 高電位，持續 10微秒
  //delayMicroseconds(10);
  //digitalWrite(trigPin, LOW);
  //long cm = pulseIn(echoPin, HIGH);   // 收到高電位時的時間
  //cm = (cm / 2) / 29.1;       // 將時間換算成距離 cm 或 inch
  //Serial.println(cm);
  // if (cm <= 10) {
    //傳照片
    //Serial.println("starting to Line");
    String payload = sendImage2LineNotify("倉庫508入侵者通知......", myLineNotifyToken);
    Serial.println(payload);
    delay(10000);//傳送後休息10秒
    Unflash();
//  }
//  delay(100);
}

//關閉閃光燈
void  Unflash(){
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);
}

//連線到WiFi
void WiFiConnect() {
  Serial.print("開始連線到:"); Serial.print(ssid);
  WiFi.begin(ssid, password);
  int WifiTryCount = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (WifiTryCount++ >= 20)  ESP.restart();
  }
  Serial.println("");
  Serial.print("WiFi connected,IP address:"); Serial.println(WiFi.localIP());
}

//鏡頭設定
void setupCam() {  
  // #define CAMERA_MODEL_AI_THINKER
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  //                default = 20000000;
  config.xclk_freq_hz = 5000000; //10000000 or 5000000
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;     // FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
  config.jpeg_quality = 10; //< Quality of JPEG output. 0-63 lower means higher quality
  config.fb_count = 2; //Number of frame buffers to be allocated. If more than one, then each frame will be acquired (double speed)

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  s = esp_camera_sensor_get();

  s->set_brightness(s, 1);   //亮度 -2 to 2
  s->set_contrast(s, 1);      //對比 -2 to 2
  s->set_saturation(s, 1);    //飽和, Hue 色相
  s->set_wb_mode(s, 0);       // 0: auto 自動, 1: sun 太陽, 2: cloud 雲, 3: indoors 室內
  //s->set_exposure_ctrl(s, 1);
  //s->set_aec_value(s, -2);
  //s->set_ae_level(s, 100);
  //s->set_gain_ctrl(s, 100);
  //s->set_pixformat(s, PIXFORMAT_RGB565);
  //s->set_pixformat(s, PIXFORMAT_JPEG);
  s->set_vflip(s, vFlip);  //垂直翻轉
  s->set_hmirror(s, hMirror);  //水平鏡像
  s->set_framesize(s, FRAMESIZE_VGA);      //640x480

  Serial.println("Camera Setup OK");
}

//傳送影像到Line
String sendImage2LineNotify(String msg, String LineNotifyToken) {
  //切換解析度(會當機)
  //sensor_t * s;
  //s->set_framesize(s, FRAMESIZE_VGA);// 640x480
  //delay(1000);//保留切換時間，避免進光量不足

  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();//取得相機影像，並放置fb
  if (!fb) {
    delay(100);
    Serial.println("Camera capture failed, Reset");
    ESP.restart();
  }
  WiFiClientSecure client_tcp;//啟動SSL wificlient
  Serial.println("Connect to notify-api.line.me");
  if (client_tcp.connect("notify-api.line.me", 443)) {
    Serial.println("Connection successful");
    String head = "--Taiwan\r\nContent-Disposition: form-data; name=\"message\"; \r\n\r\n" + msg + "\r\n--Taiwan\r\nContent-Disposition: form-data; name=\"imageFile\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--Taiwan--\r\n";
    uint16_t imageLen = fb->len;
    uint16_t extraLen = head.length() + tail.length();
    uint16_t totalLen = imageLen + extraLen;
    //開始POST傳送訊息
    client_tcp.println("POST /api/notify HTTP/1.1");
    client_tcp.println("Connection: close");
    client_tcp.println("Host: notify-api.line.me");
    client_tcp.println("Authorization: Bearer " + LineNotifyToken);
    client_tcp.println("Content-Length: " + String(totalLen));
    client_tcp.println("Content-Type: multipart/form-data; boundary=Taiwan");
    client_tcp.println();
    client_tcp.print(head);
    uint8_t *fbBuf = fb->buf;
    size_t fbLen = fb->len;
    Serial.println("Data Sending....");
    //檔案太大，分段傳送
    for (size_t n = 0; n < fbLen; n = n + 2048) {
      if (n + 2048 < fbLen) {
        client_tcp.write(fbBuf, 2048);
        fbBuf += 2048;
      } else if (fbLen % 2048 > 0) {
        size_t remainder = fbLen % 2048;
        client_tcp.write(fbBuf, remainder);
      }
    }
    client_tcp.print(tail);
    client_tcp.println();
    String getResponse = "", Feedback = "";
    boolean state = false;
    int waitTime = 3000;   // 最多等3秒
    long startTime = millis();
    delay(1000);
    Serial.print("Get Response");
    while ((startTime + waitTime) > millis())    {
      Serial.print(".");
      delay(100);
      bool jobdone = false;
      while (client_tcp.available()) {
        //當有收到回覆資料時
        jobdone = true;
        char c = client_tcp.read();
        if (c == '\n') {
          if (getResponse.length() == 0) state = true;
          getResponse = "";
        }
        else if (c != '\r')  getResponse += String(c);
        if (state == true) Feedback += String(c);
        startTime = millis();
      }
      if (jobdone) break;
    }
    client_tcp.stop();
    esp_camera_fb_return(fb);//清除緩衝區
    Unflash();
    return Feedback;
  }
  else {
    esp_camera_fb_return(fb);
    Unflash();
    return "Send failed.";
  }
}
