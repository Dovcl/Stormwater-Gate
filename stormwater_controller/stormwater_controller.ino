/*
 * 초기우수 선택 처리 빗물받이 – 차단판 자동 제어
 * ------------------------------------------------------------
 * 보드   : Arduino UNO R3
 * 부품   : 탁도센서 DFRobot SEN0189 (아날로그 모드)
 *          초음파 거리센서 HC-SR04 (Trig/Echo, 물방울 덮개 필요)
 *          서보모터 FT5330M (별도 전원)
 *
 * 동작 요약 (자세한 설명은 ../README.md)
 *   FILL    : 물 채우는 중. 판 닫힘.
 *   MEASURE : 수위 확보 후 안정화 + 첫 판단. 판 닫힘.
 *   TREAT   : 탁함 → 판 열림, 하부 필터로 처리.
 *   BYPASS  : 맑음 → 판 닫힘, 측면 월류로 우회.
 *   ALARM   : 비상 수위 / 수위센서 이상. 판 닫힘. 수동 해제.
 *   (시험 모드: 시리얼에 treat/bypass 등 상황을 입력하면 그 상태로 고정)
 *
 * 0.1초마다: 센서 읽기 → 경보 판정 → 수위 판정 → 수질 판정 → 서보 출력
 * 1초마다  : 시리얼 CSV 로그
 */

#include <Servo.h>
#include <EEPROM.h>

// ============================================================
// 1. 핀
// ============================================================
const uint8_t PIN_TURB   = A0;   // SEN0189 신호 (A0–GND 100kΩ 풀다운 권장)
const uint8_t PIN_ECHO   = 11;   // HC-SR04 Echo
const uint8_t PIN_TRIG   = 12;   // HC-SR04 Trig
const uint8_t PIN_SERVO  = 9;    // FT5330M 신호선

// ============================================================
// 2. 설정값 – 실험하면서 바꾸는 값은 전부 여기에 있음
// ============================================================
// --- 주기 ---
const unsigned long TICK_MS = 100;    // 제어 주기
const unsigned long LOG_MS  = 1000;   // 로그 주기

// --- 수위 (차단판 윗면 = 0 mm) ---
// HC-SR04를 챔버 왼쪽 벽 위(스크린 아래)에 아래를 향하게 단다.
// H_MM은 도면 기준 추정값. 설치 후 "센서 원통 앞면 ~ 차단판 윗면"을 자로 재서 바꾼다.
// HC-SR04는 약 20~30 mm보다 가까우면 값이 불안정하므로 D_MIN_MM보다 짧으면 버린다.
// → 잴 수 있는 최고 수위 = H_MM - D_MIN_MM (125 - 25 = 100 mm, 비상 80 mm보다 여유 있음)
const int H_MM     = 125;             // 센서면 ~ 차단판 윗면 높이 (실측 후 수정)
const int D_MIN_MM = 25;              // 유효 거리 하한 (근거리 불안정 구간)
const int D_MAX_MM = H_MM + 30;       // 유효 거리 상한 (바닥보다 조금 아래까지 허용)

const int LV_STOP  = 20;              // 이보다 낮으면 측정 중단 → FILL
const int LV_START = 40;              // 이 이상이 유지되면 측정 시작
const int LV_ALARM = 80;              // 비상 수위
const int LV_RESET = 50;              // 경보 해제 가능 수위

const unsigned long LV_START_HOLD_MS = 1000;  // 40 mm 유지 시간 (보고서 0.3 s → 1 s)
const unsigned long LV_ALARM_HOLD_MS = 300;   // 80 mm 유지 시간 (1회 튐 오경보 방지)
const unsigned long LV_RESET_HOLD_MS = 3000;  // 해제 전 안전상태 유지 시간

// 경보 사용 여부. 지금은 실험 편의를 위해 끔(false).
//  - false: 비상 수위는 로그에 '주의'로만 표시. 수위값이 끊기면 판을 닫고 채움에서 기다렸다가
//           값이 돌아오면 자동으로 이어서 판단 (R 입력 필요 없음)
//  - true : 비상 수위(80 mm)·수위센서 이상 시 경보 → 판 닫고 멈춤, 시리얼 R로 해제
const bool USE_ALARM = false;
const unsigned long LEVEL_TIMEOUT_MS = 1000;  // 정상 거리값이 이만큼 없으면 센서 이상
const unsigned long BOOT_GRACE_MS    = 2000;  // 전원 투입 직후 센서 이상 판정 유예

const uint8_t LEVEL_MEDIAN_N = 5;             // 수위 중앙값 표본 수
const unsigned long ECHO_TIMEOUT_US = 12000;  // pulseIn 대기 한도 (약 2 m). 일부 호환품은 응답이 늦게 시작해 여유를 둠.
                                              // 챔버보다 먼 값은 D_MAX_MM에서 걸러짐

// --- 탁도 ---
const uint8_t  TURB_N          = 10;   // 이동 표본 수 (0.1 s × 10 = 1 s)
const uint8_t  TURB_OVERSAMPLE = 8;    // 한 표본당 analogRead 횟수
const int      CLEAR_ADC_DEFAULT = 860; // 보정 전 임시값 (반드시 'C'로 보정)
const unsigned long SETTLE_MS  = 3000; // 측정 시작 후 안정화 시간

const float T_LOW  = 15.0;             // 이하 = 맑음
const float T_HIGH = 25.0;             // 이상 = 탁함 (우회 중 처리 전환 기준)
const unsigned long HOLD_TREAT_MS  = 2000;  // 처리로 바꾸기 전 유지 시간
const unsigned long HOLD_BYPASS_MS = 5000;  // 우회로 바꾸기 전 유지 시간

// --- 강우 이벤트 ---
// 수위 20 mm 미만이 이 시간 넘게 이어지면 새 강우로 보고 판단 기억을 지운다.
const unsigned long EVENT_GAP_MS = 10UL * 60UL * 1000UL;  // 10분 (시험 시 줄여서 사용)

// --- 서보 ---
// 조립 후 구멍이 어긋나는 각도/겹치는 각도를 찾아 바꾼다 ('M' 수동 모드 사용).
// 기구의 끝 멈춤에 닿지 않는 각도로 잡아야 서보가 버티며 과열되지 않는다.
const int SERVO_CLOSED_DEG = 90;
const int SERVO_OPEN_DEG   = 110;
const unsigned long SERVO_STEP_MS    = 15;    // 1°씩 움직이는 간격 (충격 완화)
const unsigned long SERVO_MIN_GAP_MS = 3000;  // 열기 동작 최소 간격 (반복 개폐 보호)

// ============================================================
// 3. 상태 정의
// ============================================================
enum State : uint8_t { ST_FILL, ST_MEASURE, ST_TREAT, ST_BYPASS, ST_ALARM, ST_MANUAL };
enum Memory : uint8_t { MEM_NONE, MEM_TREAT, MEM_BYPASS };

const char *stateName(State s) {
  switch (s) {
    case ST_FILL:    return "FILL";
    case ST_MEASURE: return "MEASURE";
    case ST_TREAT:   return "TREAT";
    case ST_BYPASS:  return "BYPASS";
    case ST_ALARM:   return "ALARM";
    default:         return "MANUAL";
  }
}
const char *memName(Memory m) {
  return m == MEM_TREAT ? "TREAT" : (m == MEM_BYPASS ? "BYPASS" : "NONE");
}

// 조건이 정해진 시간 동안 "끊김 없이" 유지됐는지 확인하는 타이머.
// 조건이 한 번이라도 깨지면 처음부터 다시 잰다.
struct Hold {
  bool on = false;
  unsigned long t0 = 0;
  bool check(bool cond, unsigned long now, unsigned long dur) {
    if (!cond) { on = false; return false; }
    if (!on)   { on = true; t0 = now; }
    return now - t0 >= dur;
  }
  void reset() { on = false; }
};

// ============================================================
// 4. 전역 변수
// ============================================================
Servo gate;

State  state = ST_FILL;
Memory mem   = MEM_NONE;
unsigned long stateSince = 0;
const char *reason = "BOOT";        // 마지막 전환 이유 / 경보 원인
bool testMode = false;              // 시험 모드: 센서 판단을 끄고, 입력한 상황으로 고정
bool logCsv   = false;              // false = 한글로 읽기 쉬운 로그, true = 엑셀용 CSV (L로 전환)

// 입력값 모드: 센서 대신 사람이 넣은 값으로 자동 판단을 돌려 본다 (t30, h60 / toff, hoff)
bool  simTurbOn  = false;  float simTurb  = 0;   // 탁도 지수 %
bool  simLevelOn = false;  int   simLevel = 0;   // 수위 mm

Hold hLevelStart, hAlarmHigh, hResetSafe, hToTreat, hToBypass;

// 수위
int  distBuf[LEVEL_MEDIAN_N];
uint8_t distCount = 0, distIdx = 0;
int  lastDistRaw = -1;              // 이번 측정 원시값 (-1 = 무응답)
int  distMedian  = -1;
int  levelMm     = 0;
bool levelOk     = false;
unsigned long lastValidDistMs = 0;
unsigned long lastWetMs = 0;        // 수위가 LV_STOP 이상이었던 마지막 시각

// 탁도
int  turbBuf[TURB_N];
uint8_t turbCount = 0, turbIdx = 0;
int  rawAdc   = 0;
float avgAdc  = 0;
float turbIdxPct = 0;
bool turbReady = false;
int  clearAdc = CLEAR_ADC_DEFAULT;
bool clearFromEeprom = false;

// 서보
int  servoDeg = SERVO_CLOSED_DEG, servoTarget = SERVO_CLOSED_DEG;
bool gateOpen = false;
unsigned long lastServoStepMs = 0, lastOpenMs = 0;
bool openedOnce = false;

// 경보 해제 요청 (시리얼 'R')
bool resetRequested = false;

// 타이머
unsigned long lastTickMs = 0, lastLogMs = 0;
unsigned long graceUntil = BOOT_GRACE_MS;   // 이 시각 전에는 수위센서 이상 판정 안 함

// ============================================================
// 5. EEPROM (맑은 물 기준값 저장)
// ============================================================
const uint16_t EE_MAGIC = 0xA55A;
const int EE_ADDR_MAGIC = 0, EE_ADDR_CLEAR = 2;

void loadClearAdc() {
  uint16_t magic; int v;
  EEPROM.get(EE_ADDR_MAGIC, magic);
  EEPROM.get(EE_ADDR_CLEAR, v);
  if (magic == EE_MAGIC && v > 100 && v <= 1023) {
    clearAdc = v; clearFromEeprom = true;
  }
}

void saveClearAdc(int v) {
  EEPROM.put(EE_ADDR_MAGIC, EE_MAGIC);
  EEPROM.put(EE_ADDR_CLEAR, v);
  clearAdc = v; clearFromEeprom = true;
}

// ============================================================
// 6. 센서 읽기
// ============================================================
int readTurbOnce() {
  long sum = 0;
  for (uint8_t i = 0; i < TURB_OVERSAMPLE; i++) sum += analogRead(PIN_TURB);
  return (int)(sum / TURB_OVERSAMPLE);
}

void resetTurb() { turbCount = 0; turbIdx = 0; turbReady = false; }

// 최근 10개 중 최댓값·최솟값을 뺀 평균.
// 기포는 한쪽(어둡게 = 탁하게)으로만 튀기 때문에 단순 평균보다 강하다.
void addTurbSample() {
  rawAdc = readTurbOnce();
  turbBuf[turbIdx] = rawAdc;
  turbIdx = (turbIdx + 1) % TURB_N;
  if (turbCount < TURB_N) turbCount++;

  long sum = 0; int mn = 1023, mx = 0;
  for (uint8_t i = 0; i < turbCount; i++) {
    int v = turbBuf[i];
    sum += v; if (v < mn) mn = v; if (v > mx) mx = v;
  }
  if (turbCount >= 3) avgAdc = (float)(sum - mn - mx) / (turbCount - 2);
  else                avgAdc = (float)sum / turbCount;

  turbIdxPct = (clearAdc - avgAdc) / (float)clearAdc * 100.0;
  turbReady  = (turbCount >= TURB_N);

  if (simTurbOn) { turbIdxPct = simTurb; turbReady = true; }   // 입력값이 있으면 그 값 사용
}

int readDistanceMm() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(4);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(20);   // 규격 10 µs 이상, 여유 있게 20 µs
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return -1;                     // 무응답
  return (int)(us * 343UL / 2000UL);          // 음속 343 m/s, 왕복 → mm
}

int medianOf(int *a, uint8_t n) {
  int t[LEVEL_MEDIAN_N];
  for (uint8_t i = 0; i < n; i++) t[i] = a[i];
  for (uint8_t i = 1; i < n; i++) {           // 삽입 정렬 (n ≤ 5)
    int k = t[i]; int8_t j = i - 1;
    while (j >= 0 && t[j] > k) { t[j + 1] = t[j]; j--; }
    t[j + 1] = k;
  }
  return t[n / 2];
}

// 유효 범위 안의 값만 버퍼에 넣고, 중앙값으로 수위를 만든다.
// 정상값이 LEVEL_TIMEOUT_MS 동안 없으면 levelOk = false.
void updateLevel(unsigned long now) {
  if (simLevelOn) {                           // 입력값: 센서를 읽지 않고 그 수위로 판단
    levelMm = simLevel; distMedian = H_MM - simLevel; lastDistRaw = distMedian;
    levelOk = true; lastValidDistMs = now;
    return;
  }
  lastDistRaw = readDistanceMm();
  bool valid = (lastDistRaw >= D_MIN_MM && lastDistRaw <= D_MAX_MM);

  if (valid) {
    distBuf[distIdx] = lastDistRaw;
    distIdx = (distIdx + 1) % LEVEL_MEDIAN_N;
    if (distCount < LEVEL_MEDIAN_N) distCount++;
    lastValidDistMs = now;
  }

  bool fresh = (distCount > 0) && (now - lastValidDistMs < LEVEL_TIMEOUT_MS);
  if (!fresh) { distCount = 0; distIdx = 0; }   // 오래된 값으로 판단하지 않도록 비움

  levelOk = fresh && distCount >= 3;
  if (levelOk) {
    distMedian = medianOf(distBuf, distCount);
    levelMm = H_MM - distMedian;
  }
}

// ============================================================
// 7. 상태 전환
// ============================================================
void logEvent(const char *msg) {
  Serial.print(F("# ")); Serial.print(millis()); Serial.print(F(" "));
  Serial.println(msg);
}

void enterState(State s, const char *why, unsigned long now) {
  // 경보 관련 타이머는 경보에 들어가거나 나올 때만 초기화
  // (FILL→MEASURE 같은 일반 전환 때문에 80 mm 유지 시간이 끊기지 않게)
  if (s == ST_ALARM || state == ST_ALARM) { hAlarmHigh.reset(); hResetSafe.reset(); }
  state = s; reason = why; stateSince = now;
  hLevelStart.reset(); hToTreat.reset(); hToBypass.reset();

  if (s == ST_MEASURE) resetTurb();           // 물에 잠긴 뒤의 값만 쓰도록 새로 모음
  if (s == ST_TREAT)   mem = MEM_TREAT;
  if (s == ST_BYPASS)  mem = MEM_BYPASS;

  Serial.print(F("# ")); Serial.print(now);
  Serial.print(F(" -> ")); Serial.print(stateName(s));
  Serial.print(F(" (")); Serial.print(why);
  Serial.print(F(") mem=")); Serial.println(memName(mem));
}

void enterAlarm(const char *cause, unsigned long now) {
  enterState(ST_ALARM, cause, now);           // 최초 원인을 reason에 고정
}

// ============================================================
// 8. 제어 판단 (0.1초마다)
// ============================================================
void controlStep(unsigned long now) {
  if (state == ST_MANUAL || testMode) return;   // 수동·시험 모드에서는 자동 판단 안 함

  // 경보가 아닐 때 들어온 R은 버린다 (나중 경보가 저절로 풀리지 않게)
  if (state != ST_ALARM) resetRequested = false;

  // ---- (1) 경보: 다른 모든 판단보다 먼저 ----
  if (!USE_ALARM) {
    // 경보 끔: 수위값이 없으면 판을 닫고 채움에서 대기. 판단 기억은 유지.
    if (!levelOk) {
      bool graceOver = ((long)(now - graceUntil) >= 0);
      if (graceOver && state != ST_FILL) enterState(ST_FILL, "NO_LEVEL", now);
      return;
    }
  }
  else if (state != ST_ALARM) {
    bool graceOver = ((long)(now - graceUntil) >= 0);
    if (graceOver && !levelOk) { enterAlarm("LEVEL_SENSOR", now); return; }
    if (levelOk && hAlarmHigh.check(levelMm >= LV_ALARM, now, LV_ALARM_HOLD_MS)) {
      enterAlarm("HIGH_WATER", now); return;
    }
  } else {
    // 안전상태(센서 정상 + 50 mm 이하)가 3초 유지된 뒤에 새로 들어온 R만 인정
    bool safe = hResetSafe.check(levelOk && levelMm <= LV_RESET, now, LV_RESET_HOLD_MS);
    if (resetRequested) {
      resetRequested = false;
      if (safe) {
        mem = MEM_NONE;                       // 해제 후엔 처음부터 판단
        enterState(ST_FILL, "RESET", now);
      } else {
        logEvent("RESET_DENIED (need level<=50mm & sensor OK for 3s)");
      }
    }
    return;
  }

  if (!levelOk) return;                       // 부팅 유예 중: FILL에서 대기

  if (levelMm >= LV_STOP) lastWetMs = now;
  bool low = (levelMm < LV_STOP);

  // ---- (2) 수위 / (3) 수질 ----
  switch (state) {
    case ST_FILL:
      // 오래 말라 있었으면 새 강우로 보고 기억 삭제
      if (mem != MEM_NONE && now - lastWetMs > EVENT_GAP_MS) {
        mem = MEM_NONE; logEvent("EVENT_GAP -> mem=NONE");
      }
      if (hLevelStart.check(levelMm >= LV_START, now, LV_START_HOLD_MS))
        enterState(ST_MEASURE, "LEVEL_UP", now);
      break;

    case ST_MEASURE:
      if (low) { enterState(ST_FILL, "LEVEL_LOW", now); break; }
      if (now - stateSince < SETTLE_MS || !turbReady) break;   // 안정화 중

      if (mem == MEM_BYPASS) {                // 직전이 우회였으면 우회로 복귀
        enterState(ST_BYPASS, "RESUME", now);  // (탁해지면 BYPASS의 25% 규칙이 처리)
        break;
      }
      // 기억 없음 또는 직전 처리: 단일 기준 15%
      if (hToTreat.check(turbIdxPct > T_LOW, now, HOLD_TREAT_MS))
        enterState(ST_TREAT, "TURB_HIGH", now);
      else if (hToBypass.check(turbIdxPct <= T_LOW, now, HOLD_BYPASS_MS))
        enterState(ST_BYPASS, "TURB_LOW", now);
      break;

    case ST_TREAT:
      // 판이 열려 물이 빠지면 닫고 다시 채움. 기억(TREAT)은 유지.
      if (low) { enterState(ST_FILL, "LEVEL_LOW", now); break; }
      if (hToBypass.check(turbIdxPct <= T_LOW, now, HOLD_BYPASS_MS))
        enterState(ST_BYPASS, "TURB_LOW", now);
      break;

    case ST_BYPASS:
      if (low) { enterState(ST_FILL, "LEVEL_LOW", now); break; }
      if (hToTreat.check(turbIdxPct >= T_HIGH, now, HOLD_TREAT_MS))
        enterState(ST_TREAT, "TURB_HIGH", now);
      break;

    default: break;
  }
}

// ============================================================
// 9. 출력 (서보)
// ============================================================
// TREAT일 때만 연다. 닫기는 즉시, 열기는 직전 열기로부터 SERVO_MIN_GAP_MS 뒤에만.
void updateGateCommand(unsigned long now) {
  if (state == ST_MANUAL) return;
  bool wantOpen = (state == ST_TREAT);

  if (!wantOpen && gateOpen) {
    gateOpen = false; servoTarget = SERVO_CLOSED_DEG;
  } else if (wantOpen && !gateOpen) {
    if (!openedOnce || now - lastOpenMs >= SERVO_MIN_GAP_MS) {
      gateOpen = true; servoTarget = SERVO_OPEN_DEG;
      lastOpenMs = now; openedOnce = true;
    }
  }
}

// 목표 각도까지 1°씩 이동 (매 loop 호출)
void stepServo(unsigned long now) {
  if (servoDeg == servoTarget) return;
  if (now - lastServoStepMs < SERVO_STEP_MS) return;
  lastServoStepMs = now;
  servoDeg += (servoTarget > servoDeg) ? 1 : -1;
  gate.write(servoDeg);
}

// ============================================================
// 10. 입력 (시리얼 명령)
// ============================================================
void printConfig() {
  Serial.println(F("# ---- config ----"));
  Serial.print(F("# clearAdc=")); Serial.print(clearAdc);
  Serial.println(clearFromEeprom ? F(" (EEPROM)") : F(" (DEFAULT! run 'C')"));
  Serial.print(F("# H_MM=")); Serial.print(H_MM);
  Serial.print(F(" D_valid=")); Serial.print(D_MIN_MM); Serial.print('~'); Serial.println(D_MAX_MM);
  Serial.print(F("# level stop/start/alarm/reset=")); Serial.print(LV_STOP); Serial.print('/');
  Serial.print(LV_START); Serial.print('/'); Serial.print(LV_ALARM); Serial.print('/'); Serial.println(LV_RESET);
  Serial.print(F("# turb low/high=")); Serial.print(T_LOW); Serial.print('/'); Serial.println(T_HIGH);
  Serial.print(F("# servo closed/open=")); Serial.print(SERVO_CLOSED_DEG); Serial.print('/');
  Serial.println(SERVO_OPEN_DEG);
  Serial.println(F("# cmds: C=calibrate clear water, R=reset alarm, P=print config,"));
  Serial.println(F("#       M=manual servo mode (then s<deg>, e.g. s100), M again=exit"));
  Serial.println(F("# test: fill / measure / treat / bypass / alarm (or 채움/측정/처리/우회/경보),"));
  Serial.println(F("#       auto (or 자동) = back to sensor control"));
  Serial.println(F("# log : L = 읽기 쉬운 로그 <-> 엑셀용 CSV 전환 (csv / txt 로 직접 지정)"));
  Serial.println(F("# sim : t<탁도%> 예) t30, h<수위mm> 예) h60 → 그 값으로 자동 판단"));
  Serial.println(F("#       toff / hoff = 실제 센서로 복귀"));
}

// 맑은 물에 센서를 담근 상태에서 실행. 5초간 평균해 EEPROM에 저장.
void calibrateClear() {
  servoTarget = SERVO_CLOSED_DEG; gateOpen = false;
  Serial.println(F("# CAL: sensor must be in CLEAR water. sampling 5 s..."));
  long sum = 0; int mn = 1023, mx = 0; const int N = 50;
  for (int i = 0; i < N; i++) {
    int v = readTurbOnce();
    sum += v; if (v < mn) mn = v; if (v > mx) mx = v;
    delay(100);
  }
  int v = (int)((sum - mn - mx) / (N - 2));
  Serial.print(F("# CAL: mean=")); Serial.print(v);
  Serial.print(F(" min=")); Serial.print(mn); Serial.print(F(" max=")); Serial.println(mx);
  if (v < 200) {
    Serial.println(F("# CAL: FAILED (too low – wiring or not clear water). not saved."));
    return;
  }
  saveClearAdc(v);
  Serial.println(F("# CAL: saved to EEPROM."));
}

// 시험 모드 단어 → 상태. 영어 또는 한글. (PSTR: 문자열을 RAM 대신 플래시에 둠)
bool parseTestWord(const char *w, State &s) {
  if (!strcmp_P(w, PSTR("fill"))    || !strcmp_P(w, PSTR("채움"))) { s = ST_FILL;    return true; }
  if (!strcmp_P(w, PSTR("measure")) || !strcmp_P(w, PSTR("측정"))) { s = ST_MEASURE; return true; }
  if (!strcmp_P(w, PSTR("treat"))   || !strcmp_P(w, PSTR("처리"))) { s = ST_TREAT;   return true; }
  if (!strcmp_P(w, PSTR("bypass"))  || !strcmp_P(w, PSTR("우회")) ||
      !strcmp_P(w, PSTR("바이패스")))                               { s = ST_BYPASS;  return true; }
  if (!strcmp_P(w, PSTR("alarm"))   || !strcmp_P(w, PSTR("경보"))) { s = ST_ALARM;   return true; }
  return false;
}

// 명령 한 줄 처리
void processCommand(char *buf, unsigned long now) {
  for (char *p = buf; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;   // 영문은 소문자로
  bool single = (buf[1] == 0);

  // ---- 시험 모드: 상황 이름을 입력하면 그 상태로 고정, 서보도 그 상태대로 움직임 ----
  State ts;
  if (parseTestWord(buf, ts)) {
    if (state == ST_MANUAL) { servoTarget = SERVO_CLOSED_DEG; gateOpen = false; }
    if (!testMode) {
      testMode = true;
      Serial.println(F("# TEST: sensors ignored. type a situation, 'auto' to exit."));
    }
    enterState(ts, "TEST", now);
    if (ts == ST_TREAT && openedOnce && now - lastOpenMs < SERVO_MIN_GAP_MS)
      Serial.println(F("# TEST: opening waits until 3 s after last open"));
    return;
  }
  if (!strcmp_P(buf, PSTR("auto")) || !strcmp_P(buf, PSTR("자동"))) {
    if (testMode) {
      testMode = false; mem = MEM_NONE;
      enterState(ST_FILL, "TEST_EXIT", now);
    }
    return;
  }

  // ---- 로그 형식 직접 지정 (제어판 페이지가 연결 직후 csv를 보냄) ----
  if (!strcmp_P(buf, PSTR("csv"))) { logCsv = true; printCsvHeader(); return; }
  if (!strcmp_P(buf, PSTR("txt"))) { logCsv = false; Serial.println(F("# 읽기 쉬운 로그로 전환")); return; }

  // ---- 입력값 모드: t<탁도%>, h<수위mm>, toff, hoff ----
  char c0 = buf[0];
  bool num = (buf[1] >= '0' && buf[1] <= '9') || buf[1] == '-' || buf[1] == '.';
  if ((c0 == 't' || c0 == 'h') && (num || !strcmp_P(buf + 1, PSTR("off")))) {
    bool on = num;
    if (c0 == 't') {
      simTurbOn = on;
      if (on) simTurb = constrain(atof(buf + 1), -50.0, 100.0);
      else    resetTurb();                    // 실제 센서 값으로 다시 모음
      Serial.print(F("# SIM: turb="));
      if (on) { Serial.print(simTurb, 1); Serial.println('%'); } else Serial.println(F("sensor"));
    } else {
      simLevelOn = on;
      if (on) simLevel = constrain(atoi(buf + 1), 0, H_MM);
      else {                                  // 실제 센서로: 버퍼를 비우고 잠깐 유예
        distCount = 0; distIdx = 0; levelOk = false;
        graceUntil = now + BOOT_GRACE_MS;
      }
      Serial.print(F("# SIM: level="));
      if (on) { Serial.print(simLevel); Serial.println(F("mm")); } else Serial.println(F("sensor"));
    }
    if (testMode) {                           // 입력값은 자동 판단으로 확인하는 것이므로 시험 모드 해제
      testMode = false; mem = MEM_NONE;
      enterState(ST_FILL, "SIM", now);
    }
    return;
  }

  // ---- 한 글자 명령 ----
  char cmd = buf[0];
  if (single && cmd == 'c') {
    calibrateClear();                       // 5 s 동안 멈춤 → 끝나면 처음부터
    unsigned long t = millis();
    graceUntil = t + BOOT_GRACE_MS;         // 멈춘 동안 수위값이 끊긴 걸 센서 이상으로 보지 않게
    lastValidDistMs = 0; distCount = 0;
    mem = MEM_NONE; testMode = false;
    enterState(ST_FILL, "CAL", t);
  }
  else if (single && cmd == 'r') resetRequested = true;
  else if (single && cmd == 'p') printConfig();
  else if (single && cmd == 'l') {
    logCsv = !logCsv;
    if (logCsv) printCsvHeader();
    else Serial.println(F("# 읽기 쉬운 로그로 전환"));
  }
  else if (single && cmd == 'm') {
    if (state != ST_MANUAL) {
      enterState(ST_MANUAL, "MANUAL", now);
      Serial.println(F("# MANUAL: control paused. 's<deg>' to move, 'M' to exit."));
    } else {
      servoTarget = SERVO_CLOSED_DEG; gateOpen = false;
      mem = MEM_NONE; testMode = false;
      enterState(ST_FILL, "MANUAL_EXIT", now);
    }
  }
  else if (cmd == 's' && buf[1] >= '0' && buf[1] <= '9' && state == ST_MANUAL) {
    int d = constrain(atoi(buf + 1), 0, 180);
    servoTarget = d;
    Serial.print(F("# MANUAL: servo -> ")); Serial.println(d);
  }
  else { Serial.print(F("# unknown cmd: ")); Serial.println(buf); }
}

// 줄바꿈(Enter)으로 명령을 끊는다. 시리얼 모니터가 "줄 끝 없음"이어도
// 마지막 글자 후 0.2초가 지나면 한 명령으로 보고 처리한다.
void handleSerial(unsigned long now) {
  static char buf[24]; static uint8_t n = 0;
  static unsigned long lastCharMs = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastCharMs = now;
    if (c != '\n' && c != '\r') { if (n < sizeof(buf) - 1) buf[n++] = c; continue; }
    if (n == 0) continue;
    buf[n] = 0; n = 0;
    processCommand(buf, now);
  }
  if (n > 0 && now - lastCharMs > 200) {
    buf[n] = 0; n = 0;
    processCommand(buf, now);
  }
}

// ============================================================
// 11. 로그 (1초마다, 엑셀에 붙여넣기용 CSV)
//   '#'로 시작하는 줄은 이벤트/메시지 → 엑셀에서 필터로 제외
// ============================================================
// 한글 상태 이름 (읽기 쉬운 로그용)
const __FlashStringHelper *stateKo(State s) {
  switch (s) {
    case ST_FILL:    return F("채움");
    case ST_MEASURE: return F("측정");
    case ST_TREAT:   return F("처리");
    case ST_BYPASS:  return F("우회");
    case ST_ALARM:   return F("경보");
    default:         return F("수동");
  }
}
const __FlashStringHelper *memKo(Memory m) {
  return m == MEM_TREAT ? F("처리") : (m == MEM_BYPASS ? F("우회") : F("없음"));
}

// 예) [  20초] 처리(시험) | 판: 열림 | 탁도 32.5% (ADC 580) | 수위 45mm (거리 80mm) | 기억: 처리 | 이유: TEST
void logReadable(unsigned long now) {
  unsigned long sec = now / 1000;
  Serial.print(F("["));
  if (sec < 1000) Serial.print(' ');
  if (sec < 100)  Serial.print(' ');
  if (sec < 10)   Serial.print(' ');
  Serial.print(sec); Serial.print(F("초] "));

  Serial.print(stateKo(state));
  if (testMode) Serial.print(F("(시험)"));
  Serial.print(F(" | 판: "));  Serial.print(gateOpen ? F("열림") : F("닫힘"));
  Serial.print(F(" (")); Serial.print(servoDeg); Serial.print(F("°)"));

  Serial.print(F(" | 탁도 "));
  if (turbReady) { Serial.print(turbIdxPct, 1); Serial.print('%'); }
  else Serial.print(F("모으는 중"));
  if (simTurbOn) Serial.print(F(" (입력값)"));
  else { Serial.print(F(" (ADC ")); Serial.print((int)avgAdc); Serial.print(')'); }

  Serial.print(F(" | 수위 "));
  if (levelOk) { Serial.print(levelMm); Serial.print(F("mm")); }
  else Serial.print(F("측정불가"));
  if (simLevelOn) Serial.print(F(" (입력값)"));
  else if (lastDistRaw < 0) Serial.print(F(" (센서 응답 없음)"));
  else { Serial.print(F(" (거리 ")); Serial.print(lastDistRaw); Serial.print(F("mm)")); }

  Serial.print(F(" | 기억: ")); Serial.print(memKo(mem));
  Serial.print(F(" | 이유: ")); Serial.print(reason);

  if (!USE_ALARM && levelOk && levelMm >= LV_ALARM) Serial.print(F(" | 주의: 비상 수위 넘음 (경보 꺼짐)"));
  if (!clearFromEeprom)            Serial.print(F(" | 주의: 맑은물 보정 안 함(C)"));
  if (avgAdc > clearAdc * 1.10)    Serial.print(F(" | 주의: 기준보다 맑게 읽힘"));
  if (turbCount > 0 && avgAdc < 5) Serial.print(F(" | 주의: 탁도센서 0 (선 빠짐?)"));
  Serial.println();
}

void logData(unsigned long now) {
  if (logCsv) logCsvLine(now); else logReadable(now);
}

void printCsvHeader() {
  Serial.println(F("ms,state,test,mem,raw_adc,avg_adc,turb_index,dist_mm,level_mm,open,servo_deg,reason,sim,warn"));
}

void logCsvLine(unsigned long now) {
  Serial.print(now);                    Serial.print(',');
  Serial.print(stateName(state));       Serial.print(',');
  Serial.print(testMode ? 1 : 0);       Serial.print(',');
  Serial.print(memName(mem));           Serial.print(',');
  Serial.print(rawAdc);                 Serial.print(',');
  Serial.print(avgAdc, 1);              Serial.print(',');
  if (turbReady) Serial.print(turbIdxPct, 1); else Serial.print(F("NA"));
  Serial.print(',');
  Serial.print(lastDistRaw);            Serial.print(',');
  if (levelOk) Serial.print(levelMm); else Serial.print(F("NA"));
  Serial.print(',');
  Serial.print(gateOpen ? 1 : 0);       Serial.print(',');
  Serial.print(servoDeg);               Serial.print(',');
  Serial.print(reason);                 Serial.print(',');
  if (simTurbOn || simLevelOn) {        // 입력값 사용 여부: T=탁도, L=수위
    if (simTurbOn)  Serial.print('T');
    if (simLevelOn) Serial.print('L');
  } else Serial.print('-');
  Serial.print(',');

  // 경고 (동작에는 영향 없음, 기록만)
  bool any = false;
  if (!clearFromEeprom)                 { Serial.print(F("CAL_DEFAULT ")); any = true; }
  if (avgAdc > clearAdc * 1.10)         { Serial.print(F("ADC_ABOVE_CLEAR ")); any = true; }
  if (turbCount > 0 && avgAdc < 5)      { Serial.print(F("ADC_ZERO ")); any = true; }
  if (!any) Serial.print(F("-"));
  Serial.println();
}

// ============================================================
// 12. setup / loop
// ============================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  // attach 전에 닫힘 각도를 먼저 써서, 첫 펄스부터 닫힘 위치로 가게 함
  gate.write(SERVO_CLOSED_DEG);
  gate.attach(PIN_SERVO);

  loadClearAdc();

  Serial.println(F("# stormwater gate controller"));
  printConfig();
  if (logCsv) printCsvHeader();
  enterState(ST_FILL, "BOOT", millis());
}

void loop() {
  unsigned long now = millis();

  handleSerial(now);

  if (now - lastTickMs >= TICK_MS) {
    lastTickMs = now;
    updateLevel(now);
    addTurbSample();
    controlStep(now);
    updateGateCommand(now);
  }

  stepServo(now);

  if (now - lastLogMs >= LOG_MS) {
    lastLogMs = now;
    logData(now);
  }
}
