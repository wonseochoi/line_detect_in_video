#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
//안정안정안정
Servo myESC;
Servo myServo;

const int escPin = 1;
const int servoPin = 2;


typedef struct struct_message {
  int throttle_pulse;
  int servo_pulse;
} struct_message;

struct_message myData;

void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  // 수신된 데이터의 크기가 예상과 다른 경우 무시 (안전장치)
  if (len != sizeof(myData)) {
    Serial.println("Error: Data size mismatch.");
    return;
  }
    
  memcpy(&myData, incomingData, sizeof(myData));
  
  // [수정됨] 수신된 값을 명확히 출력 (디버깅용)
  Serial.printf("Received> Throttle: %d us, Servo: %d us\n", myData.throttle_pulse, myData.servo_pulse);
  
  // 값의 유효 범위 확인 (안전장치)
  if (myData.throttle_pulse >= 1000 && myData.throttle_pulse <= 2000 &&
      myData.servo_pulse >= 500 && myData.servo_pulse <= 2400) {
      
    myESC.writeMicroseconds(myData.throttle_pulse);
    myServo.writeMicroseconds(myData.servo_pulse);
  } else {
    Serial.println("Error: Received values are out of range!");
  }
}
 
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  
  myESC.attach(escPin, 1000, 2000);
  myServo.attach(servoPin, 500, 2400);

  Serial.println("Arming ESC and Centering Servo...");
  //------------------------------------------------------나중에 메인에는 이거값 쓰로틀 중앙값 1500으로
  myESC.writeMicroseconds(1000);//
  myServo.writeMicroseconds(1500); // 서보 중앙 정렬
  delay(3000);
  Serial.println("Ready.");

  if (esp_now_init() != ESP_OK) { return; }
  esp_now_register_recv_cb(OnDataRecv);
}
 
void loop() {
  // No code here
}
