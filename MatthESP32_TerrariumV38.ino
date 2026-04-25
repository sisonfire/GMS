/******************************************************
 * UNO R4 WiFi (ABX00087) – Terrarium tropical (orchidées)
 * - HTTP minimal (WiFiServer) : UI HTML + API JSON
 * - SHT85 (SHT3x) via I2C
 * - 3 relais: Brassage / Extraction / Brumisateur
 * - PWM 0–10V via convertisseur (pin D9)
 * - Rampes lever/coucher, brumisation programmée (6 créneaux)
 * - Persistance: EEPROM
 * - LOG circulaire + purge
 * - Historique climatique 24h (1 point / 5 min)
 * - Pulses brassage + anti-rebond
 * - Robustesse HTTP & WiFi
 ******************************************************/

#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>
#include <string.h>
#include <Arduino.h>
#include <time.h>

#if defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS_UNO)
  #include <malloc.h>
#endif

#ifndef FPSTR
  #define FPSTR(pstr) (reinterpret_cast<const __FlashStringHelper *>(pstr))
#endif

/* ====== WIFI ====== */
#define FW_VERSION "v3.8"
#define WIFI_SSID     "BBhotel"
#define WIFI_PASSWORD "canard0408"

#ifndef WIFI_RETRY_ATTEMPTS
#define WIFI_RETRY_ATTEMPTS 5
#endif
#ifndef WIFI_CONNECT_TIMEOUT_MS
#define WIFI_CONNECT_TIMEOUT_MS 15000UL
#endif

/* ====== RÉSEAU : IP fixe (optionnel) ====== */
#define USE_STATIC_IP true

IPAddress STATIC_IP      (192,168,1,131);
IPAddress STATIC_GATEWAY (192,168,1,1);
IPAddress STATIC_SUBNET  (255,255,255,0);
IPAddress STATIC_DNS     (192,168,1,1);

/* ====== BROCHAGE ====== */
#define SHT85_ADDR           0x44

#define PIN_RELAY_FAN        2   // Brassage interne
#define PIN_RELAY_EXTRACT    3   // Extraction
#define PIN_RELAY_MISTER     4   // Brumisateur
#define PIN_PWM_LIGHT        9   // PWM lumière

const bool RELAY_ACTIVE_LOW = false;

/* ====== HORLOGE / NTP ====== */
unsigned long unixTime = 0;
unsigned long lastMillisSync = 0;
unsigned long lastNtpSync = 0;
int TZ_OFFSET_STD = 3600;   // +1h
int TZ_OFFSET_DST = 7200;   // +2h
bool DST_enabled   = true;

#ifndef NTP_INTERVAL_MS
const unsigned long NTP_INTERVAL_MS = 30UL * 60UL * 1000UL;
#endif

/* ====== JOUR/NUIT ====== */
int DAY_BEGIN_H = 8;
int DAY_END_H   = 20;

/* ====== HTTP watchdog ====== */
#ifndef HTTP_CLIENT_TIMEOUT_MS
#define HTTP_CLIENT_TIMEOUT_MS 1000UL
#endif
#ifndef HTTP_IDLE_RESTART_MS
#define HTTP_IDLE_RESTART_MS   (10UL * 60UL * 1000UL)
#endif

unsigned long lastHttpActivity = 0;
unsigned long httpReqsSinceBoot = 0;

/* ====== MODE ====== */
unsigned long lastModeChangeMs = 0;
const unsigned long MODE_GRACE_MS = 3000UL;

/* ====== SERVER & SENSOR ====== */
WiFiServer server(80);
WiFiUDP ntpUDP;
Adafruit_SHT31 sht = Adafruit_SHT31();
static const uint16_t NTP_LOCAL_PORT = 2390;
IPAddress NTP_SERVER_IP(0,0,0,0);

/* ====== LOG ====== */
struct LogEntry {
  unsigned long ts;
  char msg[48];
};

const int LOG_CAP = 10;
LogEntry logBuf[LOG_CAP];
int logHead = 0, logCount = 0;

/* ====== HISTORIQUE CLIMAT ====== */
struct ClimateEntry {
  unsigned long ts;
  float t;
  float rh;
};

const int CLIMATE_CAP = 288; // 24h à 1 point / 5 min
const unsigned long CLIMATE_PERIOD_MS = 5UL * 60UL * 1000UL;

ClimateEntry climateBuf[CLIMATE_CAP];
int climateHead = 0;
int climateCount = 0;
unsigned long lastClimateSampleMs = 0;

/* ====== FORMS ====== */
struct FormKV {
  char key[40][16];
  char val[40][32];
  int n = 0;
};

/* ====== ETATS ====== */
int relayLevel(bool on) {
  return (RELAY_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

struct RelayState {
  bool on = false;
  bool lockedManual = false;
};

struct LightState {
  float levelPct = 0.0f;
  bool lockedManual = false;
};

RelayState fan, extract, mister;
LightState light;

/* ====== DURÉES MIN / MAX ====== */
const unsigned long FAN_PULSE_PERIOD_MS = 30UL * 60UL * 1000UL;
const unsigned long FAN_PULSE_ON_MS     = 5UL  * 60UL * 1000UL;
unsigned long lastFanPulseStart = 0;
bool fanPulseActive = false;
const unsigned long FAN_MIN_ON_MS  = 8000UL;
const unsigned long FAN_MIN_OFF_MS = 8000UL;

const unsigned long EXTRACT_MIN_ON_MS  = 60000UL;
const unsigned long EXTRACT_MIN_OFF_MS = 60000UL;
unsigned long extractOnSince  = 0;
unsigned long extractOffSince = 0;

const unsigned long MISTER_MIN_REST_MS = 3UL  * 60UL * 1000UL;
const unsigned long MISTER_MIN_ON_MS   = 4000UL;
const unsigned long MISTER_HARD_MAX_MS = 8UL * 60UL * 1000UL;

unsigned long fanOnSince=0, fanOffSince=0;
unsigned long misterOnSince=0, misterOffSince=0;

/* ====== MESURES ====== */
float curT = NAN, curRH = NAN;
unsigned long lastShtMs = 0;
const unsigned long SHT_PERIOD_MS = 2000UL;
const unsigned long SENSOR_TIMEOUT_MS = 20UL * 1000UL;
unsigned long lastAmbOkMs = 0;

/* ====== CIBLES ====== */
struct Range {
  float minT, maxT, minRH, maxRH;
};

Range dayRange   = {24.0, 27.0, 70.0, 85.0};
Range nightRange = {18.0, 22.0, 80.0, 95.0};

const float HYST_T  = 0.4f;
const float HYST_RH = 2.0f;

#define TARGET_RANGE  (dayEffective ? dayRange : nightRange)

/* ====== LUMIERE ====== */
int   SUNRISE_MIN     = 45;
int   SUNSET_MIN      = 45;
float LIGHT_DAY_PCT   = 100.0f;
float LIGHT_NIGHT_PCT = 0.0f;
int PWM_RESBITS = 13;
int PWM_FULLSCALE = (1<<13)-1;

/* ====== PLAGES BRUMISATION ====== */
struct MistSlot {
  bool enabled;
  int hour;
  int minute;
  int durationS;
  int lastRunYDay;
};

MistSlot mist[6] = {
  {false, 8,  0, 20, -1},
  {false, 12, 0, 20, -1},
  {false, 16, 0, 20, -1},
  {false, 20, 0, 20, -1},
  {false, 0,  0, 20, -1},
  {false, 0,  0, 20, -1},
};

bool misterScheduledActive = false;
unsigned long misterScheduledStartMs = 0;
unsigned long misterScheduledDurMs = 0;

/* ====== BRUMISATION vs VENTILATION ====== */
const unsigned long MISTER_BLOCK_VENT_MS = 15UL * 60UL * 1000UL; // 15 min
bool misterPrevOn = false;
bool postMistBlockActive = false;
unsigned long postMistBlockStartMs = 0;

/* ====== PERSISTENCE ====== */
struct Persist {
  uint32_t magic;
  Range day, night;
  int dayBegH, dayEndH;
  int sunriseMin, sunsetMin;
  float lightDayPct, lightNightPct;
  MistSlot m[6];
} persist;

const uint32_t PERSIST_MAGIC = 0x54EEA4F1;

/* ====== Jour/Nuit avec grâce ====== */
const unsigned long DAYMODE_GRACE_MS = 120000UL;
static bool dayEffective = false;
static bool dayRaw = false;
static unsigned long lastDaySwitchMs = 0;

/* ====== MODE GLOBAL ====== */
enum Mode { AUTO_MODE, MANUAL_MODE };
Mode globalMode = AUTO_MODE;

/* ====== PROTOS ====== */
struct ClimateStats;              // forward declaration
void addLog(const String& m);
bool isDaytime();
unsigned long nowEpoch();
String tsToStr(unsigned long epoch);
ClimateStats computeClimateStats24h();

/* ====== RESET ====== */
void doRestart() {
  NVIC_SystemReset();
}

/* ====== SHT85 durcissement ====== */
static uint8_t shtBadReads = 0;
const uint8_t SHT_BAD_READ_MAX = 6;

void recoverSHT() {
  addLog("SHT85: recovery I2C");
  Wire.end();
  delay(5);
  Wire.begin();
  Wire.setClock(100000);
  sht.begin(SHT85_ADDR);
  sht.heater(false);
  shtBadReads = 0;
}

/* ====== HISTORIQUE CLIMAT ====== */
void addClimateSample(float t, float rh) {
  climateBuf[climateHead].ts = nowEpoch();
  climateBuf[climateHead].t  = t;
  climateBuf[climateHead].rh = rh;

  climateHead = (climateHead + 1) % CLIMATE_CAP;
  if (climateCount < CLIMATE_CAP) climateCount++;
}

void maybeStoreClimateSample() {
  if ((millis() - lastClimateSampleMs) < CLIMATE_PERIOD_MS) return;
  lastClimateSampleMs = millis();

  if (!timeIsValid()) return;

  if (!isnan(curT) && !isnan(curRH)) {
    addClimateSample(curT, curRH);
  }
}

void clearClimateHistory() {
  climateHead = 0;
  climateCount = 0;
  addLog("Historique climat purge");
}

struct ClimateStats {
  bool valid;
  float minT, maxT;
  float minRH, maxRH;
  float trendT;
  float trendRH;
};

ClimateStats computeClimateStats24h() {
  ClimateStats s;
  s.valid = false;
  s.minT = 9999.0f;
  s.maxT = -9999.0f;
  s.minRH = 9999.0f;
  s.maxRH = -9999.0f;
  s.trendT = 0.0f;
  s.trendRH = 0.0f;

  if (climateCount <= 0) return s;

  float firstT = NAN, firstRH = NAN;
  float lastT = NAN, lastRH = NAN;

  for (int i = 0; i < climateCount; i++) {
    int idx = (climateHead - climateCount + i + CLIMATE_CAP) % CLIMATE_CAP;
    float t = climateBuf[idx].t;
    float rh = climateBuf[idx].rh;

    if (isnan(t) || isnan(rh)) continue;

    if (!s.valid) {
      s.valid = true;
      firstT = t;
      firstRH = rh;
    }

    if (t < s.minT) s.minT = t;
    if (t > s.maxT) s.maxT = t;
    if (rh < s.minRH) s.minRH = rh;
    if (rh > s.maxRH) s.maxRH = rh;

    lastT = t;
    lastRH = rh;
  }

  if (s.valid && !isnan(firstT) && !isnan(lastT)) {
    s.trendT = lastT - firstT;
    s.trendRH = lastRH - firstRH;
  }

  return s;
}

/* ====== JOUR / NUIT ====== */
void dayModeUpdate() {
  bool r = isDaytime();
  if (r != dayRaw) {
    dayRaw = r;
    lastDaySwitchMs = millis();
    addLog(String("Transition ") + (r ? "Nuit->Jour" : "Jour->Nuit") + " grace");
  }
  if (dayEffective != dayRaw && (millis() - lastDaySwitchMs >= DAYMODE_GRACE_MS)) {
    dayEffective = dayRaw;
    addLog(String("Mode ") + (dayEffective ? "Jour" : "Nuit") + " applique");
  }
}

/* ====== UTILITAIRES ====== */
unsigned long nowEpoch() {
  return unixTime + (millis() - lastMillisSync) / 1000UL;
}

bool roughlyDST_EuropeParis(unsigned long epoch) {
  tm t;
  time_t tt = epoch;
  gmtime_r(&tt, &t);
  int m = t.tm_mon + 1;
  if (m >= 4 && m <= 9) return true;
  if (m <= 2 || m >= 11) return false;
  if (m == 3) return (t.tm_mday >= 25);
  if (m == 10) return (t.tm_mday < 25);
  return false;
}

String tsToStr(unsigned long epoch) {
  time_t tt = epoch + (DST_enabled ? TZ_OFFSET_DST : TZ_OFFSET_STD);
  tm t;
  gmtime_r(&tt, &t);
  char buf[24];
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
           t.tm_year + 1900, t.tm_mon + 1, t.tm_mday,
           t.tm_hour, t.tm_min, t.tm_sec);
  return String(buf);
}

void addLog(const String& m) {
  unsigned long ts = nowEpoch();
  logBuf[logHead].ts = ts;
  strncpy(logBuf[logHead].msg, m.c_str(), sizeof(logBuf[logHead].msg) - 1);
  logBuf[logHead].msg[sizeof(logBuf[logHead].msg) - 1] = '\0';
  logHead = (logHead + 1) % LOG_CAP;
  if (logCount < LOG_CAP) logCount++;

  Serial.print("[LOG] ");
  Serial.print(tsToStr(ts));
  Serial.print(" | ");
  Serial.println(logBuf[(logHead + LOG_CAP - 1) % LOG_CAP].msg);
}

bool isDaytime() {
  time_t tt = nowEpoch() + (DST_enabled ? TZ_OFFSET_DST : TZ_OFFSET_STD);
  tm t;
  gmtime_r(&tt, &t);
  int h = t.tm_hour;
  return (DAY_BEGIN_H < DAY_END_H)
    ? (h >= DAY_BEGIN_H && h < DAY_END_H)
    : !(h >= DAY_END_H && h < DAY_BEGIN_H);
}

int dayMinutesLocal() {
  time_t tt = nowEpoch() + (DST_enabled ? TZ_OFFSET_DST : TZ_OFFSET_STD);
  tm t;
  gmtime_r(&tt, &t);
  return t.tm_hour * 60 + t.tm_min;
}

bool timeIsValid() {
  return nowEpoch() > 1700000000UL;
}

void setRelay(int pin, bool on, RelayState &rs, unsigned long &onSince, unsigned long &offSince, const char* name = nullptr) {
  if (rs.on != on) {
    digitalWrite(pin, relayLevel(on));
    rs.on = on;
    unsigned long now = millis();
    if (on) onSince = now; else offSince = now;
    if (name) addLog(String("Relais ") + name + " -> " + (on ? "ON" : "OFF"));
  } else {
    rs.on = on;
    digitalWrite(pin, relayLevel(on));
  }
}

void setRelay(int pin, bool on, RelayState &rs, const char* name = nullptr) {
  if (rs.on != on) {
    digitalWrite(pin, relayLevel(on));
    rs.on = on;
    if (name) addLog(String("Relais ") + name + " -> " + (on ? "ON" : "OFF"));
  } else {
    rs.on = on;
    digitalWrite(pin, relayLevel(on));
  }
}

void setLightLevelPct(float pct) {
  pct = constrain(pct, 0.0f, 100.0f);
  if (abs(light.levelPct - pct) >= 1.0f) {
    addLog(String("Lumiere -> ") + String(pct, 0) + " %");
  }
  light.levelPct = pct;
  int duty = (int)round((pct / 100.0f) * PWM_FULLSCALE);
  analogWrite(PIN_PWM_LIGHT, duty);
}

float computeAutoLightPct() {
  int nowMin = dayMinutesLocal();
  int startMin = DAY_BEGIN_H * 60;
  int endMin = DAY_END_H * 60;

  if (DAY_BEGIN_H < DAY_END_H) {
    if (nowMin < startMin || nowMin >= endMin) return LIGHT_NIGHT_PCT;
    int sunriseEnd = startMin + SUNRISE_MIN;
    int sunsetStart = endMin - SUNSET_MIN;

    if (nowMin < sunriseEnd) {
      float k = float(nowMin - startMin) / max(1, SUNRISE_MIN);
      return LIGHT_NIGHT_PCT + k * (LIGHT_DAY_PCT - LIGHT_NIGHT_PCT);
    } else if (nowMin >= sunsetStart) {
      float k = float(endMin - nowMin) / max(1, SUNSET_MIN);
      return LIGHT_NIGHT_PCT + k * (LIGHT_DAY_PCT - LIGHT_NIGHT_PCT);
    } else {
      return LIGHT_DAY_PCT;
    }
  } else {
    bool inDay = (nowMin >= startMin || nowMin < endMin);
    if (!inDay) return LIGHT_NIGHT_PCT;

    int sunriseEnd = startMin + SUNRISE_MIN;
    int sunsetStart = endMin - SUNSET_MIN;

    if (nowMin >= startMin) {
      if (nowMin < sunriseEnd) {
        float k = float(nowMin - startMin) / max(1, SUNRISE_MIN);
        return LIGHT_NIGHT_PCT + k * (LIGHT_DAY_PCT - LIGHT_NIGHT_PCT);
      } else {
        return LIGHT_DAY_PCT;
      }
    } else {
      if (nowMin >= sunsetStart) {
        float k = float(endMin - nowMin) / max(1, SUNSET_MIN);
        return LIGHT_NIGHT_PCT + k * (LIGHT_DAY_PCT - LIGHT_NIGHT_PCT);
      } else {
        return LIGHT_DAY_PCT;
      }
    }
  }
}

/* ====== PERSISTENCE ====== */
void loadPersist() {
  EEPROM.get(0, persist);
  if (persist.magic != PERSIST_MAGIC) {
    persist.magic = PERSIST_MAGIC;
    persist.day = dayRange;
    persist.night = nightRange;
    persist.dayBegH = DAY_BEGIN_H;
    persist.dayEndH = DAY_END_H;
    persist.sunriseMin = SUNRISE_MIN;
    persist.sunsetMin = SUNSET_MIN;
    persist.lightDayPct = LIGHT_DAY_PCT;
    persist.lightNightPct = LIGHT_NIGHT_PCT;
    for (int i = 0; i < 6; i++) persist.m[i] = mist[i];
    EEPROM.put(0, persist);
    addLog("EEPROM initialisee");
  } else {
    dayRange = persist.day;
    nightRange = persist.night;
    DAY_BEGIN_H = persist.dayBegH;
    DAY_END_H = persist.dayEndH;
    SUNRISE_MIN = persist.sunriseMin;
    SUNSET_MIN = persist.sunsetMin;
    LIGHT_DAY_PCT = persist.lightDayPct;
    LIGHT_NIGHT_PCT = persist.lightNightPct;
    for (int i = 0; i < 6; i++) {
      mist[i] = persist.m[i];
      mist[i].lastRunYDay = -1;
    }
    addLog("Config chargee EEPROM");
  }
}

void savePersist() {
  persist.magic = PERSIST_MAGIC;
  persist.day = dayRange;
  persist.night = nightRange;
  persist.dayBegH = DAY_BEGIN_H;
  persist.dayEndH = DAY_END_H;
  persist.sunriseMin = SUNRISE_MIN;
  persist.sunsetMin = SUNSET_MIN;
  persist.lightDayPct = LIGHT_DAY_PCT;
  persist.lightNightPct = LIGHT_NIGHT_PCT;
  for (int i = 0; i < 6; i++) persist.m[i] = mist[i];
  EEPROM.put(0, persist);
  addLog("Configuration enregistree");
}

/* ====== FORMS ====== */
static bool formHas(const FormKV& F, const String& k) {
  for (int i = 0; i < F.n; i++) if (strcmp(F.key[i], k.c_str()) == 0) return true;
  return false;
}

static String formGet(const FormKV& F, const String& k) {
  for (int i = 0; i < F.n; i++) if (strcmp(F.key[i], k.c_str()) == 0) return F.val[i];
  return String();
}

FormKV parseFormUrlEncoded(WiFiClient &client, int contentLen) {
  FormKV F;
  String body;
  const int MAX_BODY = 2048; // Limit body size to prevent memory exhaustion
  int readLen = min(contentLen, MAX_BODY);
  while ((int)body.length() < readLen && client.connected()) {
    while (client.available() && (int)body.length() < readLen) body += (char)client.read();
  }

  int p = 0;
  while (p < (int)body.length() && F.n < 40) {
    int amp = body.indexOf('&', p);
    if (amp < 0) amp = body.length();
    int eq = body.indexOf('=', p);

    if (eq > p && eq < amp) {
      String keyStr = body.substring(p, eq);
      String valStr = body.substring(eq + 1, amp);
      if (keyStr.length() < 16 && valStr.length() < 32) {
        strncpy(F.key[F.n], keyStr.c_str(), 15);
        F.key[F.n][15] = '\0';
        strncpy(F.val[F.n], valStr.c_str(), 31);
        F.val[F.n][31] = '\0';
        F.n++;
      }
    }
    p = amp + 1;
  }
  return F;
}

bool parseIntS(const String& s, int &out) {
  char* e;
  long v = strtol(s.c_str(), &e, 10);
  if (*e == 0) { out = (int)v; return true; }
  return false;
}

bool parseFloatS(const String& s, float &out) {
  char* e;
  float v = strtod(s.c_str(), &e);
  if (*e == 0) { out = v; return true; }
  return false;
}

void applyConfigFromParams(const FormKV& F, bool &changed) {
  float fv;
  int iv;

  if (formHas(F,"dMinT")  && parseFloatS(formGet(F,"dMinT"),fv))  { dayRange.minT=fv; changed=true; }
  if (formHas(F,"dMaxT")  && parseFloatS(formGet(F,"dMaxT"),fv))  { dayRange.maxT=fv; changed=true; }
  if (formHas(F,"dMinRH") && parseFloatS(formGet(F,"dMinRH"),fv)) { dayRange.minRH=fv; changed=true; }
  if (formHas(F,"dMaxRH") && parseFloatS(formGet(F,"dMaxRH"),fv)) { dayRange.maxRH=fv; changed=true; }
  if (formHas(F,"nMinT")  && parseFloatS(formGet(F,"nMinT"),fv))  { nightRange.minT=fv; changed=true; }
  if (formHas(F,"nMaxT")  && parseFloatS(formGet(F,"nMaxT"),fv))  { nightRange.maxT=fv; changed=true; }
  if (formHas(F,"nMinRH") && parseFloatS(formGet(F,"nMinRH"),fv)) { nightRange.minRH=fv; changed=true; }
  if (formHas(F,"nMaxRH") && parseFloatS(formGet(F,"nMaxRH"),fv)) { nightRange.maxRH=fv; changed=true; }

  if (formHas(F,"dayBegH") && parseIntS(formGet(F,"dayBegH"),iv) && iv>=0&&iv<=23) { DAY_BEGIN_H=iv; changed=true; }
  if (formHas(F,"dayEndH") && parseIntS(formGet(F,"dayEndH"),iv) && iv>=0&&iv<=23) { DAY_END_H=iv; changed=true; }
  if (formHas(F,"sunriseMin") && parseIntS(formGet(F,"sunriseMin"),iv) && iv>=0&&iv<=240) { SUNRISE_MIN=iv; changed=true; }
  if (formHas(F,"sunsetMin")  && parseIntS(formGet(F,"sunsetMin"),iv)  && iv>=0&&iv<=240) { SUNSET_MIN=iv; changed=true; }
  if (formHas(F,"lightDayPct")   && parseFloatS(formGet(F,"lightDayPct"),fv)   && fv>=0&&fv<=100) { LIGHT_DAY_PCT=fv; changed=true; }
  if (formHas(F,"lightNightPct") && parseFloatS(formGet(F,"lightNightPct"),fv) && fv>=0&&fv<=100) { LIGHT_NIGHT_PCT=fv; changed=true; }

  for (int i = 0; i < 6; i++) {
    String enK=String("msEn")+i, hK=String("msH")+i, mK=String("msM")+i, dK=String("msD")+i;
    if (formHas(F,enK)) { String v=formGet(F,enK); mist[i].enabled=(v=="1"||v=="true"); changed=true; }
    if (formHas(F,hK) && parseIntS(formGet(F,hK),iv)) { mist[i].hour=constrain(iv,0,23); changed=true; }
    if (formHas(F,mK) && parseIntS(formGet(F,mK),iv)) { mist[i].minute=constrain(iv,0,59); changed=true; }
    if (formHas(F,dK) && parseIntS(formGet(F,dK),iv)) { mist[i].durationS=max(1,iv); changed=true; }
  }
}

/* ====== NTP ====== */
static bool ntpSyncOnce() {
  if (NTP_SERVER_IP == IPAddress(0,0,0,0)) {
    addLog("NTP: pas d'adresse serveur");
    return false;
  }

  const int NTP_PACKET_SIZE = 48;
  byte packetBuffer[NTP_PACKET_SIZE];
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  packetBuffer[0] = 0b11100011;

  if (!ntpUDP.beginPacket(NTP_SERVER_IP, 123)) {
    addLog("NTP beginPacket KO");
    return false;
  }
  if (ntpUDP.write(packetBuffer, NTP_PACKET_SIZE) != NTP_PACKET_SIZE) {
    addLog("NTP write incomplet");
    ntpUDP.endPacket();
    return false;
  }
  if (!ntpUDP.endPacket()) {
    addLog("NTP endPacket KO");
    return false;
  }

  unsigned long t0 = millis();
  while (millis() - t0 < 1000) { // Reduced timeout to 1s
    int size = ntpUDP.parsePacket();
    if (size >= NTP_PACKET_SIZE) {
      ntpUDP.read(packetBuffer, NTP_PACKET_SIZE);
unsigned long secsSince1900 =
  ((unsigned long)packetBuffer[40] << 24) |
  ((unsigned long)packetBuffer[41] << 16) |
  ((unsigned long)packetBuffer[42] << 8)  |
  (unsigned long)packetBuffer[43];

const unsigned long seventyYears = 2208988800UL;

// 🔴 AJOUT 1 : validation brute NTP
if (secsSince1900 < seventyYears) {
  addLog("NTP invalide (underflow)");
  return false;
}

unsigned long newUnixTime = secsSince1900 - seventyYears;
unsigned long oldUnixTime = nowEpoch();

// 🔴 AJOUT 2 : détection de saut d'horloge
if (timeIsValid()) {
  long delta = (long)newUnixTime - (long)oldUnixTime;

  if (abs(delta) > 3600) {  // > 1 heure
    climateHead = 0;
    climateCount = 0;
    addLog("Saut horaire -> purge historique climat");
  }
}

// 🔴 puis mise à jour normale
unixTime = newUnixTime;
lastMillisSync = millis();
DST_enabled = roughlyDST_EuropeParis(unixTime);
addLog(String("NTP OK via ") + NTP_SERVER_IP.toString());
return true;
    }
    delay(10);
  }

  addLog(String("NTP timeout via ") + NTP_SERVER_IP.toString());
  return false;
}

/* ====== WATCHDOGS ====== */
static void printIP() {
  IPAddress ip = WiFi.localIP();
  Serial.print("IP: ");
  Serial.println(ip);
  addLog(String("WiFi connecte: ") + ip.toString());
}

static bool connectWiFiBlocking(const char* ssid, const char* pass) {
  WiFi.end();
  delay(200);

  if (USE_STATIC_IP) {
    WiFi.config(STATIC_IP, STATIC_DNS, STATIC_GATEWAY, STATIC_SUBNET);
    addLog(String("IP fixe demandee: ") + STATIC_IP.toString());
  }

  for (int attempt = 1; attempt <= WIFI_RETRY_ATTEMPTS; ++attempt) {
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssid, pass);

    uint32_t t0 = millis();
    while (millis() - t0 < WIFI_CONNECT_TIMEOUT_MS) {
      if (WiFi.status() == WL_CONNECTED) break;
      delay(250);
    }
    if (WiFi.status() == WL_CONNECTED) {
      printIP();
      return true;
    }
  }

  addLog("WiFi KO assoc");
  return false;
}

void wifiWatchdog() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    addLog("WiFi perdu -> reco");
    if (connectWiFiBlocking(WIFI_SSID, WIFI_PASSWORD)) {
      server.begin();
      ntpUDP.stop();
      ntpUDP.begin(NTP_LOCAL_PORT);
      addLog("WiFi retabli");
      lastHttpActivity = millis();
    } else {
      addLog("WiFi KO reco");
    }
  }
}

void httpWatchdog() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 5000) return;
  lastCheck = millis();

  if (millis() - lastHttpActivity > HTTP_IDLE_RESTART_MS) {
    addLog("HTTP inactif -> restart");
    server.begin();
    lastHttpActivity = millis();
  }
}

/* ====== BRUMISATION PROGRAMMEE ====== */
void maybeStartScheduledMisting() {
  if (misterScheduledActive) return;
  if (mister.on) return;
  if ((millis() - misterOffSince) < MISTER_MIN_REST_MS) return;

  time_t tt = nowEpoch() + (DST_enabled ? TZ_OFFSET_DST : TZ_OFFSET_STD);
  tm ti;
  gmtime_r(&tt, &ti);

  for (int i = 0; i < 6; i++) {
    if (!mist[i].enabled) continue;
    if (mist[i].hour == ti.tm_hour && mist[i].minute == ti.tm_min) {
      if (mist[i].lastRunYDay != ti.tm_yday) {
        misterScheduledActive = true;
        misterScheduledStartMs = millis();
        misterScheduledDurMs = (unsigned long)max(1, mist[i].durationS) * 1000UL;
        if (misterScheduledDurMs < MISTER_MIN_ON_MS) misterScheduledDurMs = MISTER_MIN_ON_MS;
        mist[i].lastRunYDay = ti.tm_yday;

        setRelay(PIN_RELAY_MISTER, true, mister, misterOnSince, misterOffSince, "Brumisateur slot");
        addLog(String("Brumisation slot#") + String(i + 1) + " demarree");
        break;
      }
    }
  }
}

/* ====== LOGIQUE AUTO ====== */
void autoControl() {
  unsigned long now = millis();

  if (globalMode == AUTO_MODE && (now - lastModeChangeMs) < MODE_GRACE_MS) return;

  // suivi brumisation pour blocage post-brumisation
  if (mister.on && !misterPrevOn) {
    postMistBlockActive = false;
  }
  if (!mister.on && misterPrevOn) {
    postMistBlockActive = true;
    postMistBlockStartMs = now;
    addLog("Brumisation OFF -> blocage ventilation");
  }
  misterPrevOn = mister.on;

  // pulses brassage
  if (!fan.lockedManual) {
    if (!fanPulseActive && (now - lastFanPulseStart >= FAN_PULSE_PERIOD_MS)) {
      fanPulseActive = true;
      lastFanPulseStart = now;
      addLog("Pulse brassage demarre");
    }
    if (fanPulseActive && (now - lastFanPulseStart > FAN_PULSE_ON_MS)) {
      fanPulseActive = false;
      addLog("Pulse brassage termine");
    }
  }

  bool ambStale = (now - lastAmbOkMs) > SENSOR_TIMEOUT_MS;
  Range target = TARGET_RANGE;

  bool postMistBlock = postMistBlockActive && ((now - postMistBlockStartMs) < MISTER_BLOCK_VENT_MS);
  if (postMistBlockActive && !postMistBlock) {
    postMistBlockActive = false;
    addLog("Fin blocage ventilation apres brumisation");
  }

  bool ventilationBlocked = mister.on || postMistBlock;

  // sécurités capteurs
  if (ambStale || isnan(curT) || isnan(curRH)) {
    if (!mister.lockedManual) {
      setRelay(PIN_RELAY_MISTER, false, mister, misterOnSince, misterOffSince, "Brumisateur");
      misterScheduledActive = false;
    }
  } else {
    // gestion fin de slot
    if (misterScheduledActive) {
      if (now - misterScheduledStartMs >= misterScheduledDurMs) {
        if (millis() - misterOnSince >= MISTER_MIN_ON_MS) {
          misterScheduledActive = false;
          setRelay(PIN_RELAY_MISTER, false, mister, misterOnSince, misterOffSince, "Brumisateur fin slot");
          addLog("Brumisation programmee terminee");
        }
      }
    }

    // hard max
    if (!mister.lockedManual && mister.on && (now - misterOnSince >= MISTER_HARD_MAX_MS)) {
      setRelay(PIN_RELAY_MISTER, false, mister, misterOnSince, misterOffSince, "Brumisateur hard cut");
      addLog("Brumisateur coupe hard max");
      misterScheduledActive = false;
    }
  }

  // Extraction
  static bool extractDemand = false;

  if (!extract.lockedManual) {
    bool wantExtract = false;

    if (!ventilationBlocked) {
      const float ON_RH_THRESH  = target.maxRH + HYST_RH;
      const float OFF_RH_THRESH = target.maxRH;

      if (!isnan(curRH)) {
        if (curRH > ON_RH_THRESH)        extractDemand = true;
        else if (curRH < OFF_RH_THRESH)  extractDemand = false;
      }

      bool tempDemand = (!isnan(curT) && curT > (target.maxT + HYST_T));
      wantExtract = (extractDemand || tempDemand);
    }

    unsigned long since   = extract.on ? extractOnSince : extractOffSince;
    unsigned long dwellMs = extract.on ? EXTRACT_MIN_ON_MS : EXTRACT_MIN_OFF_MS;
    bool change = (wantExtract != extract.on) && (millis() - since >= dwellMs);

    if (change) {
      setRelay(PIN_RELAY_EXTRACT, wantExtract, extract, "Extraction");
      if (wantExtract) extractOnSince = millis();
      else             extractOffSince = millis();
    }
  }

  // démarrage des slots
  maybeStartScheduledMisting();

  // lumière
  if (!light.lockedManual) setLightLevelPct(computeAutoLightPct());

  // Brassage
  bool wantFan = (!ambStale && !ventilationBlocked) ? (fanPulseActive || extract.on) : false;

  if (!fan.lockedManual) {
    bool willChange = (wantFan != fan.on);
    if (willChange) {
      unsigned long since = fan.on ? fanOnSince : fanOffSince;
      unsigned long dwell = fan.on ? FAN_MIN_ON_MS : FAN_MIN_OFF_MS;
      if (millis() - since < dwell) willChange = false;
    }
    if (willChange) {
      setRelay(PIN_RELAY_FAN, wantFan, fan, "Brassage");
      if (wantFan) fanOnSince = millis();
      else         fanOffSince = millis();
    }
  }
}

/* ====== FREE MEMORY ====== */
#if defined(ARDUINO_ARCH_AVR)
extern "C" char* sbrk(int incr);
int freeMemory() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}
#elif defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_ARCH_RENESAS_UNO)
int freeMemory() {
  struct mallinfo mi = mallinfo();
  return mi.fordblks;
}
#else
int freeMemory() { return -1; }
#endif

/* ===== WEB UI ===== */
const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="fr"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Terrarium Tropical – UNO R4 WiFi</title>
<style>
body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Ubuntu;max-width:1100px;margin:24px auto;padding:0 12px}
.card{border:1px solid #ddd;border-radius:12px;padding:16px;margin:12px 0;box-shadow:0 2px 6px rgba(0,0,0,.05)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.btn{padding:10px 14px;border-radius:10px;border:1px solid #ccc;background:#f8f8f8;cursor:pointer}
.badge{display:inline-block;padding:2px 8px;border-radius:999px;background:#eee;margin-left:8px}
.switch{cursor:pointer;display:inline-block;padding:8px 12px;border-radius:999px;border:1px solid #bbb}
input[type=number]{width:92px}
input[type=range]{width:220px}
small{color:#555}
label{display:inline-block;min-width:140px}
table{width:100%;border-collapse:collapse}
th,td{border-bottom:1px solid #eee;padding:6px 4px;text-align:left}
#log{max-height:260px;overflow:auto;background:#fafafa;border:1px solid #eee;border-radius:8px;padding:8px;font-family:ui-monospace,Consolas,monospace;font-size:12px}
.logline{white-space:pre}
canvas{width:100%;max-width:100%;height:280px;border:1px solid #eee;border-radius:8px;background:#fff}
.muted{color:#666}
</style></head><body>

<h1 id="pageTitle">Terrarium Tropical – UNO R4 WiFi</h1>

<div class="card">
  <div class="row">
    <strong>Mode:</strong>
    <button id="modeBtn" class="switch">…</button>
    <span id="dayBadge" class="badge">…</span>
    <span id="fwBadge" class="badge">…</span>
  </div>
  <div style="margin-top:10px">
    <div>Température amb. : <strong id="t">…</strong></div>
    <div>Humidité amb. : <strong id="rh">…</strong></div>
    <div>Plage cible : <span id="range">…</span></div>
    <div>Capteurs : <span id="sensor">…</span></div>
    <div>Heure (NTP) : <strong id="now">…</strong> <span id="timeOk" class="badge">…</span></div>
  </div>
</div>

<div class="card">
  <h3>Historique climatique (24 h)</h3>
  <div class="grid">
    <div>Temp min/max : <strong id="cMinMaxT">…</strong></div>
    <div>RH min/max : <strong id="cMinMaxRH">…</strong></div>
    <div>Tendance T : <strong id="cTrendT">…</strong></div>
    <div>Tendance RH : <strong id="cTrendRH">…</strong></div>
  </div>
  <div style="margin-top:12px">
    <canvas id="climateChart" width="1000" height="280"></canvas>
    <div class="muted" style="margin-top:8px">Courbe 24 h : température (bleu) et humidité relative (vert).</div>
  </div>
</div>

<div class="card"><h3>Relais</h3>
  <div class="grid">
    <div><div><strong>Brassage</strong></div><div class="row"><button class="btn" id="fanOn">ON</button><button class="btn" id="fanOff">OFF</button><span class="badge" id="fanState">…</span></div></div>
    <div><div><strong>Extraction</strong></div><div class="row"><button class="btn" id="extOn">ON</button><button class="btn" id="extOff">OFF</button><span class="badge" id="extState">…</span></div></div>
    <div><div><strong>Brumisateur</strong></div><div class="row"><button class="btn" id="mistOn">ON</button><button class="btn" id="mistOff">OFF</button><span class="badge" id="mistState">…</span></div></div>
  </div>
  <p style="margin-top:8px"><small>En <strong>Auto</strong>, les boutons mettent le relais en manuel (verrou) jusqu'au retour en Auto.</small></p>
</div>

<div class="card"><h3>Lumière (0–10 V)</h3>
  <div class="row">
    <div>Niveau : <strong id="lightPct">…</strong></div>
    <input id="lightSlider" type="range" min="0" max="100" step="1" value="0">
    <button class="btn" id="lightSet">Appliquer</button>
    <button class="btn" id="lightOff">OFF</button>
    <span class="badge" id="lightLock">…</span>
  </div>
</div>

<div class="card" id="cfgCard">
  <h3>Configuration (Cibles & Brumisation)</h3>
  <div class="grid">
    <div>
      <div><strong>Plages cibles – Jour</strong></div>
      <div class="row">
        <label>Temp min</label><input id="dMinT" type="number" step="0.1">
        <label>Temp max</label><input id="dMaxT" type="number" step="0.1">
      </div>
      <div class="row">
        <label>RH min</label><input id="dMinRH" type="number" step="1">
        <label>RH max</label><input id="dMaxRH" type="number" step="1">
      </div>
    </div>
    <div>
      <div><strong>Plages cibles – Nuit</strong></div>
      <div class="row">
        <label>Temp min</label><input id="nMinT" type="number" step="0.1">
        <label>Temp max</label><input id="nMaxT" type="number" step="0.1">
      </div>
      <div class="row">
        <label>RH min</label><input id="nMinRH" type="number" step="1">
        <label>RH max</label><input id="nMaxRH" type="number" step="1">
      </div>
    </div>
  </div>

  <div style="margin:12px 0">
    <div class="row">
      <label>Début journée (h)</label><input id="dayBegH" type="number" min="0" max="23">
      <label>Fin journée (h)</label><input id="dayEndH" type="number" min="0" max="23">
    </div>
    <div class="row">
      <label>Lever (min)</label><input id="sunriseMin" type="number" min="0" max="240">
      <label>Coucher (min)</label><input id="sunsetMin" type="number" min="0" max="240">
    </div>
  </div>

  <div>
    <strong>Brumisation – 6 plages</strong>
    <table id="mistTbl">
      <thead><tr><th>#</th><th>Actif</th><th>Heure</th><th>Minute</th><th>Durée (s)</th></tr></thead>
      <tbody></tbody>
    </table>
  </div>

  <div class="row" style="margin-top:10px">
    <button class="btn" id="cfgLoad">Charger</button>
    <button class="btn" id="cfgSave">Enregistrer</button>
    <span class="badge" id="cfgStatus"> </span>
  </div>
</div>

<div class="card"><h3>Journal (LOG)</h3>
  <div class="row" style="margin-bottom:8px">
    <button class="btn" id="logRefresh">Rafraîchir</button>
    <button class="btn" id="logClear">Vider</button>
  </div>
  <div id="log"></div>
</div>

<script>
let lastClimate = [];
let hoveredClimateIndex = -1;

async function getStatus(){
  try{
    const s = await (await fetch('/api/status')).json();

    document.getElementById('pageTitle').textContent = `Terrarium Tropical – UNO R4 WiFi (${s.version || 'n/a'})`;
    fwBadge.textContent = s.version || 'n/a';

    t.textContent = isNaN(s.t) ? '—' : s.t.toFixed(1)+' °C';
    rh.textContent = isNaN(s.rh) ? '—' : s.rh.toFixed(0)+' %';
    range.textContent = s.daytime
      ? `Jour: ${s.day.minT}–${s.day.maxT} °C / ${s.day.minRH}–${s.day.maxRH} %`
      : `Nuit: ${s.night.minT}–${s.night.maxT} °C / ${s.night.minRH}–${s.night.maxRH} %`;

    sensor.textContent = s.sensor_ok ? 'OK' : 'EN ATTENTE…';
    now.textContent = s.time || '—';
    timeOk.textContent = s.time_ok ? 'heure OK' : 'en attente…';

    fanState.textContent = s.fan.on ? 'ON' : 'OFF';
    extState.textContent = s.extract.on ? 'ON' : 'OFF';
    mistState.textContent = s.mister.on ? 'ON' : 'OFF';
    modeBtn.textContent = s.mode === 'auto' ? 'Auto' : 'Manuel';
    dayBadge.textContent = s.daytime ? 'JOUR' : 'NUIT';

    lightPct.textContent = `${s.light.level}%`;
    lightSlider.value = s.light.level;
    lightLock.textContent = s.light.locked ? 'LOCK' : 'AUTO';

    if (s.climate24h && s.climate24h.valid) {
      cMinMaxT.textContent = `${s.climate24h.minT.toFixed(1)} / ${s.climate24h.maxT.toFixed(1)} °C`;
      cMinMaxRH.textContent = `${s.climate24h.minRH.toFixed(0)} / ${s.climate24h.maxRH.toFixed(0)} %`;
      cTrendT.textContent = `${s.climate24h.trendT >= 0 ? '+' : ''}${s.climate24h.trendT.toFixed(1)} °C`;
      cTrendRH.textContent = `${s.climate24h.trendRH >= 0 ? '+' : ''}${s.climate24h.trendRH.toFixed(0)} %`;
    } else {
      cMinMaxT.textContent = '—';
      cMinMaxRH.textContent = '—';
      cTrendT.textContent = '—';
      cTrendRH.textContent = '—';
    }
  }catch(e){
    console.log(e);
  }
}

async function relay(which,state){
  await fetch(`/api/relay/${which}/${state}`, {method:'POST'});
  await getStatus();
}

async function setMode(){
  await fetch('/api/mode/toggle', {method:'POST'});
  await getStatus();
}

async function setLight(level){
  await fetch(`/api/light/set?level=${level}`, {method:'POST'});
  await getStatus();
}

async function getLogs(){
  try{
    const l = await (await fetch('/api/logs')).json();
    const cont = document.getElementById('log');
    cont.innerHTML = '';
    l.forEach(line=>{
      const d = document.createElement('div');
      d.className='logline';
      d.textContent = `[${line.ts}] ${line.msg}`;
      cont.appendChild(d);
    });
    cont.scrollTop = cont.scrollHeight;
  }catch(e){
    console.log(e);
  }
}

async function clearLogs(){
  await fetch('/api/logs/clear', {method:'POST'});
  await getLogs();
}

function buildMistRows(){
  const tb = document.querySelector('#mistTbl tbody');
  tb.innerHTML = '';
  for(let i=0;i<6;i++){
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${i+1}</td>
      <td><input type="checkbox" id="msEn${i}"></td>
      <td><input type="number" id="msH${i}" min="0" max="23"></td>
      <td><input type="number" id="msM${i}" min="0" max="59"></td>
      <td><input type="number" id="msD${i}" min="1"></td>`;
    tb.appendChild(tr);
  }
}

async function loadConfigUI(){
  const s = await (await fetch('/api/config')).json();
  dMinT.value=s.dMinT; dMaxT.value=s.dMaxT; dMinRH.value=s.dMinRH; dMaxRH.value=s.dMaxRH;
  nMinT.value=s.nMinT; nMaxT.value=s.nMaxT; nMinRH.value=s.nMinRH; nMaxRH.value=s.nMaxRH;
  dayBegH.value=s.dayBegH; dayEndH.value=s.dayEndH;
  sunriseMin.value=s.sunriseMin; sunsetMin.value=s.sunsetMin;

  for(let i=0;i<6;i++){
    document.getElementById('msEn'+i).checked=!!s['msEn'+i];
    document.getElementById('msH'+i).value=s['msH'+i];
    document.getElementById('msM'+i).value=s['msM'+i];
    document.getElementById('msD'+i).value=s['msD'+i];
  }
  cfgStatus.textContent='Chargé';
}

async function saveConfigUI(){
  const p=new URLSearchParams();
  ['dMinT','dMaxT','dMinRH','dMaxRH','nMinT','nMaxT','nMinRH','nMaxRH','dayBegH','dayEndH','sunriseMin','sunsetMin']
    .forEach(k=>p.append(k,document.getElementById(k).value));

  for(let i=0;i<6;i++){
    p.append('msEn'+i, document.getElementById('msEn'+i).checked?'true':'false');
    p.append('msH'+i, document.getElementById('msH'+i).value);
    p.append('msM'+i, document.getElementById('msM'+i).value);
    p.append('msD'+i, document.getElementById('msD'+i).value);
  }

  await fetch('/api/config/update',{
    method:'POST',
    headers:{'Content-Type':'application/x-www-form-urlencoded'},
    body:p.toString()
  });
  cfgStatus.textContent='Enregistré';
}

function formatClimateTime(ts){
  if (!ts) return '—';
  return ts.slice(11, 16); // "HH:MM"
}

function drawClimateChart(data){
  const canvas = document.getElementById('climateChart');
  const ctx = canvas.getContext('2d');
  const W = canvas.width;
  const H = canvas.height;

  ctx.clearRect(0,0,W,H);
  ctx.fillStyle = '#ffffff';
  ctx.fillRect(0,0,W,H);

  if(!data || !data.length){
    ctx.fillStyle = '#777';
    ctx.font = '14px sans-serif';
    ctx.fillText('Pas encore assez de données', 20, 30);
    return;
  }

  const padL = 50, padR = 20, padT = 20, padB = 70;
  const plotW = W - padL - padR;
  const plotH = H - padT - padB;

  const temps = data.map(p=>p.t).filter(v=>!isNaN(v));
  const rhs   = data.map(p=>p.rh).filter(v=>!isNaN(v));

  if(!temps.length || !rhs.length){
    ctx.fillStyle = '#777';
    ctx.font = '14px sans-serif';
    ctx.fillText('Données invalides', 20, 30);
    return;
  }

  let minT = Math.min(...temps);
  let maxT = Math.max(...temps);
  let minRH = Math.min(...rhs);
  let maxRH = Math.max(...rhs);

  minRH -= 0.5;
  maxRH += 0.5;

  if(maxRH - minRH < 2) {
    maxRH += 1;
    minRH -= 1;
  }

  if(maxT - minT < 1) { maxT += 0.5; minT -= 0.5; }
  if(maxRH - minRH < 2) { maxRH += 1; minRH -= 1; }

  ctx.strokeStyle = '#ddd';
  ctx.lineWidth = 1;
  for(let i=0;i<=4;i++){
    const y = padT + (plotH/4)*i;
    ctx.beginPath();
    ctx.moveTo(padL, y);
    ctx.lineTo(padL + plotW, y);
    ctx.stroke();
  }

  ctx.fillStyle = '#444';
  ctx.font = '12px sans-serif';

  for(let i=0;i<=4;i++){
    const y = padT + plotH - (plotH/4)*i;
    const tVal = minT + ((maxT-minT)/4)*i;
    ctx.fillText(tVal.toFixed(1)+'°', 4, y+4);
  }

  for(let i=0;i<=4;i++){
    const y = padT + plotH - (plotH/4)*i;
    const rhVal = minRH + ((maxRH-minRH)/4)*i;
    const txt = rhVal.toFixed(0)+'%';
    const tw = ctx.measureText(txt).width;
    ctx.fillText(txt, W - tw - 2, y+4);
  }
  const firstEpoch = Number(data[0]?.epoch || 0);
  const lastEpoch = Number(data[data.length-1]?.epoch || firstEpoch);
  const spanEpoch = Math.max(1, lastEpoch - firstEpoch);
  const xFor = i => {
    const e = Number(data[i]?.epoch || 0);
    return padL + ((e - firstEpoch) / spanEpoch) * plotW;
  };
  const yForT = v => padT + plotH - ((v - minT) / (maxT - minT)) * plotH;
  const yForRH = v => padT + plotH - ((v - minRH) / (maxRH - minRH)) * plotH;

  const screenPoints = data.map((p, i) => ({
  i,
  x: xFor(i),
  yT: yForT(Number(p.t)),
  yRH: yForRH(Number(p.rh)),
  ts: p.ts,
  t: Number(p.t),
  rh: Number(p.rh)
}));

 ctx.strokeStyle = '#cfcfcf';
  ctx.lineWidth = 1;
  const tickCount = 6;
  for (let i = 0; i <= tickCount; i++) {
    const x = padL + (plotW / tickCount) * i;
    ctx.beginPath();
    ctx.moveTo(x, padT);
    ctx.lineTo(x, padT + plotH);
    ctx.stroke();

    const epoch = firstEpoch + (spanEpoch / tickCount) * i;
    const d = new Date(epoch * 1000);
    const hh = String(d.getHours()).padStart(2, '0');
    const mm = String(d.getMinutes()).padStart(2, '0');
    const label = `${hh}:${mm}`;
    const tw = ctx.measureText(label).width;
    ctx.fillStyle = '#444';
    ctx.fillText(label, x - tw / 2, H - 32);
  }


  ctx.strokeStyle = 'rgba(229,62,62,0.7)'; // rouge temp'
  ctx.lineWidth = 2;
  ctx.beginPath();
  screenPoints.forEach((p,i)=>{
    if(i===0) ctx.moveTo(p.x, p.yT); else ctx.lineTo(p.x, p.yT);
  });
  ctx.stroke();

  ctx.fillStyle = '#e53e3e';
  screenPoints.forEach((p)=>{
    ctx.beginPath();
    ctx.arc(p.x, p.yT, 3, 0, Math.PI * 2);
    ctx.fill();
  });


  ctx.strokeStyle = 'rgba(49,130,206,0.7)'; // bleu humidité
  ctx.lineWidth = 2;
  ctx.beginPath();
  screenPoints.forEach((p,i)=>{
    if(i===0) ctx.moveTo(p.x, p.yRH); else ctx.lineTo(p.x, p.yRH);
  });
  ctx.stroke();

  ctx.fillStyle = '#3182ce';
  screenPoints.forEach((p)=>{
    ctx.beginPath();
    ctx.arc(p.x, p.yRH, 3, 0, Math.PI * 2);
    ctx.fill();
  });

const legendYBox = H - 18;
const legendYText = H - 8;

// Température (rouge)
ctx.fillStyle = '#e53e3e';
ctx.fillRect(padL, H-18, 12, 12);
ctx.fillStyle = '#333';
ctx.fillText('Température', padL+18, H-8);

// Humidité (bleu)
ctx.fillStyle = '#3182ce';
ctx.fillRect(padL+120, H-18, 12, 12);
ctx.fillStyle = '#333';
ctx.fillText('Humidité', padL+138, H-8);

if (
  hoveredClimateIndex >= 0 &&
  hoveredClimateIndex < screenPoints.length
) {
  const p = screenPoints[hoveredClimateIndex];

  // ligne verticale de repère
  ctx.strokeStyle = '#999';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(p.x, padT);
  ctx.lineTo(p.x, padT + plotH);
  ctx.stroke();

  // surbrillance des points
  ctx.fillStyle = '#e53e3e';
  ctx.beginPath();
  ctx.arc(p.x, p.yT, 5, 0, Math.PI * 2);
  ctx.fill();

  ctx.fillStyle = '#3182ce';
  ctx.beginPath();
  ctx.arc(p.x, p.yRH, 5, 0, Math.PI * 2);
  ctx.fill();

  // contenu tooltip
  const line1 = formatClimateTime(p.ts);
  const line2 = `T: ${p.t.toFixed(1)} °C`;
  const line3 = `RH: ${p.rh.toFixed(0)} %`;

  ctx.font = '12px sans-serif';
  const textW = Math.max(
    ctx.measureText(line1).width,
    ctx.measureText(line2).width,
    ctx.measureText(line3).width
  );

  const boxW = textW + 16;
  const boxH = 54;

  let boxX = p.x + 12;
  let boxY = padT + 10;

  if (boxX + boxW > W - 4) boxX = p.x - boxW - 12;
  if (boxY + boxH > H - padB) boxY = padT + 4;

  // fond tooltip
  ctx.fillStyle = 'rgba(255,255,255,0.95)';
  ctx.strokeStyle = '#bbb';
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.rect(boxX, boxY, boxW, boxH);
  ctx.fill();
  ctx.stroke();

  // texte tooltip
  ctx.fillStyle = '#222';
  ctx.fillText(line1, boxX + 8, boxY + 14);

  ctx.fillStyle = '#e53e3e';
  ctx.fillText(line2, boxX + 8, boxY + 30);

  ctx.fillStyle = '#3182ce';
  ctx.fillText(line3, boxX + 8, boxY + 46);
}

}

function sanitizeClimateData(data){
  if (!Array.isArray(data)) return [];

  const cleanedInput = data
    .map(p => ({
      ts: p.ts,
      epoch: Number(p.epoch),
      t: Number(p.t),
      rh: Number(p.rh)
    }))
    .filter(p =>
      Number.isFinite(p.epoch) &&
      Number.isFinite(p.t) &&
      Number.isFinite(p.rh) &&
      p.rh >= 0 && p.rh <= 100 &&
      p.t > -20 && p.t < 80
    );

  cleanedInput.sort((a, b) => a.epoch - b.epoch);

  if (cleanedInput.length <= 1) return cleanedInput;

  const result = [cleanedInput[0]];

  for (let i = 1; i < cleanedInput.length; i++) {
    const prev = result[result.length - 1];
    const cur = cleanedInput[i];
    const dt = cur.epoch - prev.epoch;

    // On attend environ 300 s entre deux points.
    // On accepte une marge large : de 60 s à 900 s.
    if (dt >= 60 && dt <= 900) {
      result.push(cur);
    } else {
      console.warn('Point climat ignoré (écart temps incohérent):', cur);
    }
  }

  return result;
}
async function loadClimate(){
  try{
    const raw = await (await fetch('/api/climate?_=' + Date.now())).json();
    lastClimate = sanitizeClimateData(raw);
    drawClimateChart(lastClimate);
  }catch(e){
    console.log(e);
  }
}

buildMistRows();
cfgLoad.onclick=loadConfigUI;
cfgSave.onclick=saveConfigUI;
modeBtn.onclick=setMode;
fanOn.onclick =()=>relay('fan','on');
fanOff.onclick=()=>relay('fan','off');
extOn.onclick =()=>relay('extract','on');
extOff.onclick=()=>relay('extract','off');
mistOn.onclick=()=>relay('mister','on');
mistOff.onclick=()=>relay('mister','off');
lightSet.onclick=()=>setLight(document.getElementById('lightSlider').value);
lightOff.onclick=()=>setLight(0);
logRefresh.onclick=getLogs;
logClear.onclick=clearLogs;

const climateCanvas = document.getElementById('climateChart');

climateCanvas.addEventListener('mousemove', (e) => {
  if (!lastClimate || !lastClimate.length) return;

  const rect = climateCanvas.getBoundingClientRect();
  const scaleX = climateCanvas.width / rect.width;
  const mouseX = (e.clientX - rect.left) * scaleX;

  const padL = 50;
  const padR = 20;
  const plotW = climateCanvas.width - padL - padR;

  const firstEpoch = Number(lastClimate[0]?.epoch || 0);
  const lastEpoch  = Number(lastClimate[lastClimate.length - 1]?.epoch || 0);
  const spanEpoch = Math.max(1, lastEpoch - firstEpoch);

  let bestIdx = -1;
  let bestDist = Infinity;

  lastClimate.forEach((p, i) => {
    const epoch = Number(p.epoch || 0);
    const x = padL + ((epoch - firstEpoch) / spanEpoch) * plotW;
    const d = Math.abs(mouseX - x);
    if (d < bestDist) {
      bestDist = d;
      bestIdx = i;
    }
  });

  hoveredClimateIndex = bestIdx;
  drawClimateChart(lastClimate);
});

climateCanvas.addEventListener('mouseleave', () => {
  hoveredClimateIndex = -1;
  drawClimateChart(lastClimate);
});

loadConfigUI();
getStatus();
getLogs();
loadClimate();

setInterval(async ()=>{
  await getStatus();
  await getLogs();
}, 3000);

setInterval(async ()=>{
  await loadClimate();
}, 30000);
</script></body></html>
)HTML";

void sendIndexPage(WiFiClient &client) {
  const char* data = INDEX_HTML;
  size_t len = strlen(data);

  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Content-Type: text/html\r\n"));
  client.print(F("Connection: close\r\n"));
  client.print(F("Cache-Control: no-cache\r\n"));
  client.print(F("Content-Length: "));
  client.print(len);
  client.print(F("\r\n\r\n"));

  const size_t CHUNK = 1024;
  size_t sent = 0;
  while (sent < len) {
    size_t n = min(CHUNK, len - sent);
    client.write((const uint8_t*)(data + sent), n);
    sent += n;
    delay(1);
  }
}

/* ===== JSON ===== */
String jsonClimateHistory() {
  String j;
  j.reserve(14000);
  j += "[";

  for (int i = 0; i < climateCount; i++) {
    int idx = (climateHead - climateCount + i + CLIMATE_CAP) % CLIMATE_CAP;
    j += "{\"ts\":\"" + tsToStr(climateBuf[idx].ts) + "\",\"epoch\":" + String(climateBuf[idx].ts) + ",\"t\":" + String(climateBuf[idx].t, 1) + ",\"rh\":" + String(climateBuf[idx].rh, 0) + "}";
    if (i < climateCount - 1) j += ",";
  }

  j += "]";
  return j;
}

String jsonStatus() {
  bool daytime = isDaytime();
  ClimateStats cs = computeClimateStats24h();

  String j;
  j.reserve(900);
  j += "{";
  j += "\"version\":\"" FW_VERSION "\",";
  j += "\"t\":" + String(isnan(curT) ? 0.0f : curT, 1) + ",";
  j += "\"rh\":" + String(isnan(curRH) ? 0.0f : curRH, 0) + ",";
  j += "\"sensor_ok\":" + String((millis()-lastAmbOkMs)<=SENSOR_TIMEOUT_MS ? "true" : "false") + ",";
  j += "\"daytime\":" + String(daytime ? "true" : "false") + ",";
  j += "\"time_ok\":" + String(timeIsValid() ? "true" : "false") + ",";
  j += "\"time\":\"" + tsToStr(nowEpoch()) + "\",";

  j += "\"mode\":\"" + String(globalMode==AUTO_MODE ? "auto" : "manual") + "\",";
  j += "\"fan\":{\"on\":" + String(fan.on ? "true":"false") + ",\"locked\":" + String(fan.lockedManual ? "true":"false") + "},";
  j += "\"extract\":{\"on\":" + String(extract.on ? "true":"false") + ",\"locked\":" + String(extract.lockedManual ? "true":"false") + "},";
  j += "\"mister\":{\"on\":" + String(mister.on ? "true":"false") + ",\"locked\":" + String(mister.lockedManual ? "true":"false") + "},";

  j += "\"light\":{\"level\":" + String(light.levelPct,0) + ",\"locked\":" + String(light.lockedManual ? "true":"false") + "},";

  j += "\"climate24h\":{";
  j += "\"valid\":" + String(cs.valid ? "true" : "false") + ",";
  j += "\"minT\":" + String(cs.valid ? cs.minT : 0.0f, 1) + ",";
  j += "\"maxT\":" + String(cs.valid ? cs.maxT : 0.0f, 1) + ",";
  j += "\"minRH\":" + String(cs.valid ? cs.minRH : 0.0f, 0) + ",";
  j += "\"maxRH\":" + String(cs.valid ? cs.maxRH : 0.0f, 0) + ",";
  j += "\"trendT\":" + String(cs.valid ? cs.trendT : 0.0f, 1) + ",";
  j += "\"trendRH\":" + String(cs.valid ? cs.trendRH : 0.0f, 0);
  j += "},";

  j += "\"day\":{"
       "\"minT\":" + String(dayRange.minT,0) + ","
       "\"maxT\":" + String(dayRange.maxT,0) + ","
       "\"minRH\":" + String(dayRange.minRH,0) + ","
       "\"maxRH\":" + String(dayRange.maxRH,0) + "},";

  j += "\"night\":{"
       "\"minT\":" + String(nightRange.minT,0) + ","
       "\"maxT\":" + String(nightRange.maxT,0) + ","
       "\"minRH\":" + String(nightRange.minRH,0) + ","
       "\"maxRH\":" + String(nightRange.maxRH,0) + "}";

  j += "}";
  return j;
}

String jsonConfig() {
  String j;
  j.reserve(700);
  j += "{";
  j += "\"dMinT\":" + String(dayRange.minT,1) + ",";
  j += "\"dMaxT\":" + String(dayRange.maxT,1) + ",";
  j += "\"dMinRH\":" + String(dayRange.minRH,0) + ",";
  j += "\"dMaxRH\":" + String(dayRange.maxRH,0) + ",";
  j += "\"nMinT\":" + String(nightRange.minT,1) + ",";
  j += "\"nMaxT\":" + String(nightRange.maxT,1) + ",";
  j += "\"nMinRH\":" + String(nightRange.minRH,0) + ",";
  j += "\"nMaxRH\":" + String(nightRange.maxRH,0) + ",";

  j += "\"dayBegH\":" + String(DAY_BEGIN_H) + ",";
  j += "\"dayEndH\":" + String(DAY_END_H) + ",";
  j += "\"sunriseMin\":" + String(SUNRISE_MIN) + ",";
  j += "\"sunsetMin\":" + String(SUNSET_MIN) + ",";
  j += "\"lightDayPct\":" + String(LIGHT_DAY_PCT,0) + ",";
  j += "\"lightNightPct\":" + String(LIGHT_NIGHT_PCT,0) + ",";

  for (int i = 0; i < 6; i++) {
    j += "\"msEn" + String(i) + "\":" + String(mist[i].enabled ? "true":"false") + ",";
    j += "\"msH" + String(i) + "\":" + String(mist[i].hour) + ",";
    j += "\"msM" + String(i) + "\":" + String(mist[i].minute) + ",";
    j += "\"msD" + String(i) + "\":" + String(mist[i].durationS);
    if (i < 5) j += ",";
  }

  j += "}";
  return j;
}

/* ===== HTTP HELPERS ===== */
String reqMethod, reqPath, reqQuery;

bool parseFirstLine(const String& line) {
  int sp1 = line.indexOf(' ');
  int sp2 = line.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) return false;

  reqMethod = line.substring(0, sp1);
  String url = line.substring(sp1 + 1, sp2);

  int q = url.indexOf('?');
  if (q >= 0) {
    reqPath = url.substring(0, q);
    reqQuery = url.substring(q + 1);
  } else {
    reqPath = url;
    reqQuery = "";
  }
  return true;
}

String qparam(const String& name) {
  int p = 0;
  while (p < (int)reqQuery.length()) {
    int amp = reqQuery.indexOf('&', p);
    if (amp < 0) amp = reqQuery.length();
    int eq = reqQuery.indexOf('=', p);
    if (eq > p && eq < amp) {
      String k = reqQuery.substring(p, eq);
      String v = reqQuery.substring(eq + 1, amp);
      if (k == name) return v;
    }
    p = amp + 1;
  }
  return "";
}

void sendResponse(WiFiClient &client, int code, const char* ctype, const String& body) {
  client.print("HTTP/1.1 ");
  client.print(code);
  client.print(" OK\r\n");
  client.print("Connection: close\r\n");
  client.print("Access-Control-Allow-Origin: *\r\n");
  client.print("Content-Type: ");
  client.print(ctype);
  client.print("\r\n");
  client.print("Cache-Control: no-cache\r\n");
  client.print("Content-Length: ");
  client.print(body.length());
  client.print("\r\n\r\n");

  const size_t CHUNK = 512;
  const char* data = body.c_str();
  size_t len = body.length();
  size_t sent = 0;
  while (sent < len) {
    size_t n = min(CHUNK, len - sent);
    client.write((const uint8_t*)(data + sent), n);
    sent += n;
    delay(1);
  }
}

void sendJsonClimateHistory(WiFiClient &client) {
  client.print(F("HTTP/1.1 200 OK\r\n"));
  client.print(F("Connection: close\r\n"));
  client.print(F("Access-Control-Allow-Origin: *\r\n"));
  client.print(F("Content-Type: application/json\r\n"));
  client.print(F("Cache-Control: no-cache\r\n\r\n"));

  client.print("[");
  for (int i = 0; i < climateCount; i++) {
    int idx = (climateHead - climateCount + i + CLIMATE_CAP) % CLIMATE_CAP;
    client.print("{\"ts\":\"");
    client.print(tsToStr(climateBuf[idx].ts));
    client.print("\",\"epoch\":");
    client.print(climateBuf[idx].ts);
    client.print(",\"t\":");
    client.print(String(climateBuf[idx].t, 1));
    client.print(",\"rh\":");
    client.print(String(climateBuf[idx].rh, 0));
    client.print("}");
    if (i < climateCount - 1) client.print(",");
  }
  client.print("]");
}


/* ===== Handlers ===== */
void handleToggleMode() {
  globalMode = (globalMode == AUTO_MODE) ? MANUAL_MODE : AUTO_MODE;
  addLog(String("Mode -> ") + (globalMode==AUTO_MODE ? "AUTO" : "MANUEL"));

  if (globalMode == AUTO_MODE) {
    fan.lockedManual = false;
    extract.lockedManual = false;
    mister.lockedManual = false;
    light.lockedManual = false;
    addLog("Verrous manuels liberes");
    lastModeChangeMs = millis();
    lastFanPulseStart = millis();
    fanPulseActive = false;
  }
}

void manualRelayControl(const String &which, bool turnOn) {
  if (globalMode == AUTO_MODE) {
    if      (which=="fan")     fan.lockedManual = true;
    else if (which=="extract") extract.lockedManual = true;
    else if (which=="mister")  mister.lockedManual = true;
  }

  if (which=="fan") {
    if (turnOn && mister.on) {
      addLog("Refus brassage pendant brumisation");
      return;
    }
    setRelay(PIN_RELAY_FAN, turnOn, fan, fanOnSince, fanOffSince, "Brassage");
  }
  else if (which=="extract") {
    if (turnOn && mister.on) {
      addLog("Refus extraction pendant brumisation");
      return;
    }
    setRelay(PIN_RELAY_EXTRACT, turnOn, extract, extractOnSince, extractOffSince, "Extraction");
    if (turnOn) extractOnSince = millis();
    else        extractOffSince = millis();

    if (turnOn && !fan.lockedManual) {
      unsigned long fsince = fan.on ? fanOnSince : fanOffSince;
      unsigned long fdwell = fan.on ? FAN_MIN_ON_MS : FAN_MIN_OFF_MS;
      if (!fan.on && (millis() - fsince >= fdwell)) {
        setRelay(PIN_RELAY_FAN, true, fan, "Brassage couple");
        fanOnSince = millis();
      }
    }
  }
  else if (which=="mister") {
    setRelay(PIN_RELAY_MISTER, turnOn, mister, misterOnSince, misterOffSince, "Brumisateur");

    if (turnOn) {
      if (!fan.lockedManual && fan.on) {
        setRelay(PIN_RELAY_FAN, false, fan, "Brassage OFF brumisation");
        fanOffSince = millis();
      }
      if (!extract.lockedManual && extract.on) {
        setRelay(PIN_RELAY_EXTRACT, false, extract, "Extraction OFF brumisation");
        extractOffSince = millis();
      }
    } else {
      if (misterScheduledActive) {
        misterScheduledActive = false;
        addLog("Brumisation programmee stop manuel");
      }
      postMistBlockActive = true;
      postMistBlockStartMs = millis();
      addLog("Blocage ventilation post-brumisation");
    }
  }
  else if (which=="light") {
    setLightLevelPct(turnOn ? 100.0f : 0.0f);
  }
}

/* ===== SETUP ===== */
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("[BOOT] Setup starting...");
  Serial.print("[BOOT] Free memory: ");
  Serial.println(freeMemory());

  pinMode(PIN_RELAY_FAN, OUTPUT);
  pinMode(PIN_RELAY_EXTRACT, OUTPUT);
  pinMode(PIN_RELAY_MISTER, OUTPUT);

  digitalWrite(PIN_RELAY_FAN, relayLevel(false));
  digitalWrite(PIN_RELAY_EXTRACT, relayLevel(false));
  digitalWrite(PIN_RELAY_MISTER, relayLevel(false));

  fanOffSince = millis();
  extractOffSince = millis();
  misterOffSince = millis();

#if defined(analogWriteResolution)
  analogWriteResolution(PWM_RESBITS);
#else
  PWM_RESBITS = 8;
#endif
  PWM_FULLSCALE = (1 << PWM_RESBITS) - 1;
  pinMode(PIN_PWM_LIGHT, OUTPUT);
  analogWrite(PIN_PWM_LIGHT, 0);

  Wire.begin();
  Wire.setClock(100000);

  if (!sht.begin(SHT85_ADDR)) {
    addLog("SHT85 introuvable 0x44");
  } else {
    sht.heater(false);
    addLog("SHT85 OK");
  }

  loadPersist();

  Serial.print("WiFi... SSID=");
  Serial.println(WIFI_SSID);
  if (!connectWiFiBlocking(WIFI_SSID, WIFI_PASSWORD)) {
    Serial.println("Pas d IP");
  }

  IPAddress ip;
  if (WiFi.hostByName("pool.ntp.org", ip)) {
    NTP_SERVER_IP = ip;
    addLog(String("DNS NTP -> ") + NTP_SERVER_IP.toString());
  } else {
    addLog("DNS NTP KO");
  }

  ntpUDP.begin(NTP_LOCAL_PORT);
  if (ntpSyncOnce()) addLog("NTP OK");
  else               addLog("NTP KO plus tard");

  dayRaw = dayEffective = isDaytime();

  server.begin();
  lastHttpActivity = millis();
  addLog("Serveur HTTP pret");
}

/* ===== LOOP ===== */
void loop() {
  static unsigned long lastMemCheck = 0;
  if (millis() - lastMemCheck > 60000) { // Check every minute
    lastMemCheck = millis();
    int mem = freeMemory();
    if (mem < 1000) { // Low memory warning
      addLog(String("Memoire faible: ") + String(mem));
    }
  }

  wifiWatchdog();
  httpWatchdog();

  tm lt;
  time_t nowLoc = nowEpoch() + (DST_enabled ? TZ_OFFSET_DST : TZ_OFFSET_STD);
  gmtime_r(&nowLoc, &lt);
  static int lastResetDay = -1;
  if (lt.tm_hour == 2 && lt.tm_min == 0 && lt.tm_yday != lastResetDay) {
    lastResetDay = lt.tm_yday;
    addLog("Reboot planifie 03:00");
    doRestart();
  }

  if (WiFi.status() == WL_CONNECTED && (millis() - lastNtpSync > NTP_INTERVAL_MS)) {
    if (ntpSyncOnce()) addLog("NTP resynchronise");
    lastNtpSync = millis();
  }

  if (millis() - lastShtMs >= SHT_PERIOD_MS) {
    lastShtMs = millis();
    float t = sht.readTemperature();
    float h = sht.readHumidity();
    if (!isnan(t) && !isnan(h) && h >= 0 && h <= 100) {
      curT = t;
      curRH = h;
      lastAmbOkMs = millis();
      shtBadReads = 0;
    } else {
      Serial.println("SHT85 lecture invalide...");
      if (++shtBadReads >= SHT_BAD_READ_MAX) recoverSHT();
    }
  }

  maybeStoreClimateSample();
  dayModeUpdate();

  if (globalMode == AUTO_MODE) autoControl();

  WiFiClient client = server.available();
  if (client) {
    client.setTimeout(HTTP_CLIENT_TIMEOUT_MS);
    lastHttpActivity = millis();

    String line = client.readStringUntil('\n');
    line.trim();
        if (line.length() > 220) {
      sendResponse(client,414,"text/plain","URI too long");
      client.stop();
      return;
    }
    if (!parseFirstLine(line)) {
      client.stop();
      return;
    }

    int contentLength = 0;
    while (client.connected()) {
      String h = client.readStringUntil('\n');
      if (h == "\r" || h.length() == 0) break;
      h.trim();
      if (h.startsWith("Content-Length:")) contentLength = h.substring(15).toInt();
    }

    if (contentLength < 0) contentLength = 0;
    if (contentLength > 4096) {
      sendResponse(client,413,"text/plain","Payload too large");
      client.stop();
      return;
    }

    if (reqMethod=="GET" && reqPath=="/") {
      sendIndexPage(client);
    }
    else if (reqMethod=="GET" && reqPath=="/api/status") {
      sendResponse(client,200,"application/json",jsonStatus());
    }
    else if (reqMethod=="GET" && reqPath=="/api/climate") {
      sendJsonClimateHistory(client);
    }
    else if (reqMethod=="POST" && reqPath=="/api/mode/toggle") {
      handleToggleMode();
      sendResponse(client,200,"application/json","{\"ok\":true}");
    }
    else if (reqMethod=="POST" && reqPath=="/api/light/set") {
      float lvl = 0;
      if (reqQuery.length()) {
        String lv = qparam("level");
        if (lv.length()) lvl = lv.toFloat();
      } else if (contentLength > 0) {
        FormKV P = parseFormUrlEncoded(client, contentLength);
        if (formHas(P,"level")) lvl = formGet(P,"level").toFloat();
      }
      if (globalMode==AUTO_MODE) light.lockedManual = true;
      setLightLevelPct(lvl);
      sendResponse(client,200,"application/json","{\"ok\":true}");
    }
    else if (reqMethod=="POST" && reqPath.startsWith("/api/relay/")) {
      String s = reqPath;
      int p1 = s.indexOf('/', 10);
      int p2 = s.indexOf('/', p1 + 1);
      String which = (p1 > 0 && p2 > p1) ? s.substring(p1 + 1, p2) : "";
      String state = (p2 > 0) ? s.substring(p2 + 1) : "";
      bool turnOn = (state == "on");
      if (which.length() > 0) {
        manualRelayControl(which, turnOn);
        sendResponse(client,200,"application/json","{\"ok\":true}");
      } else {
        sendResponse(client,400,"application/json","{\"error\":\"bad relay\"}");
      }
    }
    else if (reqMethod=="GET" && reqPath=="/api/config") {
      sendResponse(client,200,"application/json",jsonConfig());
    }
    else if (reqMethod=="POST" && reqPath=="/api/config/update") {
      FormKV P = parseFormUrlEncoded(client, contentLength);
      bool changed = false;
      applyConfigFromParams(P, changed);
      if (changed) savePersist();
      sendResponse(client,200,"application/json","{\"ok\":true}");
    }
    else if (reqMethod=="GET" && reqPath=="/api/logs") {
      String j = "[";
      for (int i = 0; i < logCount; i++) {
        int idx = (logHead - logCount + i + LOG_CAP) % LOG_CAP;
        j += "{\"ts\":\"" + tsToStr(logBuf[idx].ts) + "\",\"msg\":\"";
        String m(logBuf[idx].msg);
        m.replace("\\","\\\\");
        m.replace("\"","\\\"");
        j += m + "\"}";
        if (i < logCount - 1) j += ",";
      }
      j += "]";
      sendResponse(client,200,"application/json",j);
    }
    else if (reqMethod=="POST" && reqPath=="/api/logs/clear") {
      logHead = 0;
      logCount = 0;
      addLog("Journal vide");
      sendResponse(client,200,"application/json","{\"ok\":true}");
    }
    else if (reqMethod=="GET" && reqPath=="/ping") {
      sendResponse(client,200,"text/plain","pong");
    }
    else {
      sendResponse(client,404,"text/plain","Not found");
    }

    unsigned long t0 = millis();
    while (client.available() && millis() - t0 < 50) client.read();
    delay(1);
    client.stop();
    httpReqsSinceBoot++;
    lastHttpActivity = millis();
  }
}