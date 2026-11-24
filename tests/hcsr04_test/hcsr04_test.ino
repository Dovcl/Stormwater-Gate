/*
 * HC-SR04 단독 점검용 (본 코드와 별개)
 * 배선: VCC → 5V, Gnd → GND, Trig → 12, Echo → 11  (본 코드와 같음)
 * 시리얼 모니터 115200 baud. 0.5초마다 결과를 한 줄씩 출력.
 *
 *  - "거리 123 mm"         → 센서·배선 정상
 *  - "응답 없음 (Echo가 한 번도 안 올라옴)" → 선(Trig/Echo 바뀜, 빠짐) 또는 전원 문제
 *  - "응답 없음 (Echo가 HIGH에 멈춤)"     → 앞에 반사될 물체가 없음(4 m 이상) 또는 센서 불량
 */
const uint8_t PIN_TRIG = 12;
const uint8_t PIN_ECHO = 11;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.println(F("HC-SR04 점검 시작. 센서 앞 10~30 cm에 손이나 책을 대 보세요."));
}

void loop() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(20);
  digitalWrite(PIN_TRIG, LOW);

  // Echo가 올라오는지 따로 확인 (최대 30 ms)
  unsigned long t0 = micros();
  bool rose = false;
  while (micros() - t0 < 30000UL) { if (digitalRead(PIN_ECHO) == HIGH) { rose = true; break; } }
  unsigned long startDelay = micros() - t0;

  if (!rose) {
    Serial.println(F("응답 없음 (Echo가 한 번도 안 올라옴) → 배선/전원 확인"));
  } else {
    unsigned long t1 = micros();
    while (digitalRead(PIN_ECHO) == HIGH && micros() - t1 < 40000UL) {}
    unsigned long us = micros() - t1;
    if (us >= 40000UL) {
      Serial.println(F("응답 없음 (Echo가 HIGH에 멈춤) → 앞에 물체가 너무 멀거나 센서 불량"));
    } else {
      Serial.print(F("거리 ")); Serial.print(us * 343UL / 2000UL);
      Serial.print(F(" mm   (에코 ")); Serial.print(us);
      Serial.print(F(" us, 시작 지연 ")); Serial.print(startDelay); Serial.println(F(" us)"));
    }
  }
  delay(500);
}
