#include <esp_now.h>
#include <WiFi.h>
//안정안정안정
// 약속된 수신 보드 MAC 주소
uint8_t broadcastAddress[] = {0x64, 0xE8, 0x33, 0xAF, 0xB6, 0xBC};

typedef struct struct_message {
  int throttle_pulse;
  int servo_pulse;
} struct_message;

struct_message myData;

void OnDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // 전송 상태는 시리얼 모니터로 확인하지 않으므로 비워두거나 간단한 메시지만 남깁니다.
  if (status != ESP_NOW_SEND_SUCCESS) {
    // Serial.println("Send Fail"); // 디버깅 시 필요하면 활성화
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) { return; }
  esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);
  
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ return; }
}


void loop() {
  if (Serial.available() > 0) {
    // [수정됨] 안정적인 파싱을 위해 C 스타일 함수 사용
    char buffer[32];
    int bytesRead = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1);
    buffer[bytesRead] = '\0'; // 문자열 끝에 NULL 추가

    // 콤마(,)를 기준으로 문자열을 두 부분으로 나눔
    char* first = strtok(buffer, ",");
    char* second = strtok(NULL, ",");

    if (first != NULL && second != NULL) {
      // 파이썬이 "서보,스로틀" 순서로 보내므로, 순서에 맞게 변환
      int servo_val = atoi(first);
      int throttle_val = atoi(second);

      // 수신된 값 확인 (디버깅용)
      Serial.printf("Parsed> Servo: %d, Throttle: %d\n", servo_val, throttle_val);
      
      myData.servo_pulse = servo_val;
      myData.throttle_pulse = throttle_val;

      esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
    }
  }
}
