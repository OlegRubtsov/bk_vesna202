/*
 * BK Tape Player для ESP32-C3.
 *
 * Прошивка читает BIN-файлы в формате ленты БК, формирует импульсный сигнал
 * на AUDIO_PIN аппаратным таймером и параллельно обслуживает веб-плеер,
 * OLED-дисплей, LittleFS и Wi-Fi. Долгие операции загрузки выполняются
 * порциями в основном цикле; обработчик таймера не работает с файлами
 * и не выделяет память.
 *
 * Проекты и материалы, использованные при разработке
 *
 * raydac/bkbin2wav — конвертер BIN в WAV, справочный пример формирования ленты.
 *   https://github.com/raydac/bkbin2wav
 *   https://www.igormaznitsa.com/bkbin2wav/index.html
 * bk-utils — конвертер WAV из подборки reference/links.txt.
 *   https://zakirov-net.github.io/bk-utils/wav-converter.html
 * Manwe и lzakirov — загрузчик BK0010, разбор пилотона и опыт ускоренной загрузки.
 *   https://zx-pk.ru/threads/30298-zagruzka-s-magnitofona-na-bk-0011%28m%29.html
 *   https://thesands.ru/bk0010/wav-converter/
 * bkread — описание синхроимпульсов, полей заголовка и контрольной суммы.
 *   https://bk0010.narod.ru/readme.html
 * Эмулятор GID — справка по загрузке с ленты на разных моделях БК.
 *   https://gid.pdp-11.net/emu_docv4.html
 * Ю. А. Зальцман — описание кодировки символов БК для имени на ленте.
 *   https://gid.pdp-11.net/books/Zaltsman.html
 *
 * Онлайн-каталог kalininskiy — источник карточек программ, BIN и ZIP.
 *   https://kalininskiy.github.io/bk-catalog/
 *   https://github.com/kalininskiy/bk-catalog
 *
 * U8g2 — вывод на OLED. Arduino-ESP32 — таймер, Wi-Fi, HTTP, DNS, NVS и LittleFS.
 *   https://github.com/olikraus/u8g2
 *   https://github.com/espressif/arduino-esp32
 * PlatformIO Espressif 32 — сборка и загрузка прошивки на ESP32-C3.
 *   https://github.com/platformio/platform-espressif32
 */
#include <Arduino.h>
#include <GyverButton.h>  // https://github.com/GyverLibs/GyverButton
#include <U8g2lib.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <esp32-hal-timer.h>
#include <algorithm>
#include <vector>

// Подключение платы: OLED по I²C на GPIO 5/6, ленточный аудиовыход GPIO 8.
// Confirmed wiring: large OLED on GPIO 5/6, audio/tape signal on GPIO 8.
constexpr uint8_t OLED_SDA = 5;
constexpr uint8_t OLED_SCL = 6;
extern "C" const uint8_t bk0010_start[], bk0010_end[];
constexpr int SYSTEM_BIN_ID = 65536; // Вне диапазона индексов файлов LittleFS.
constexpr char SYSTEM_BIN_NAME[] = "bk0010.bin";
constexpr char SYSTEM_BIN_MESSAGE[] = "This is a system file and cannot be deleted.";
size_t systemBinSize() {
  return reinterpret_cast<uintptr_t>(bk0010_end) - reinterpret_cast<uintptr_t>(bk0010_start);
}

constexpr uint8_t AUDIO_PIN = 8;
// Все четыре кнопки замыкают вход на GND.
GButton buttons[] = {GButton(0), GButton(1), GButton(2), GButton(3)};
struct LocalBin {
  String name;
  int id;
};
std::vector<LocalBin> localBins;
bool localBinsDirty = true;
size_t selectedBin = 0;
float savedSpeed = 1.0f;
bool replaceTapeName = false, upperTapeName = false;
String replacementName = "V";
enum class MenuItem { Speed, Level, Replace, Upper };
MenuItem menuItem = MenuItem::Speed;
bool menuOpen = false, menuEditing = false;
int menuDraft = 0;
uint32_t menuRevision = 0;
bool buttonPlayFailed = false;
// Длительность элементарного сигнала БК при скорости 1x; ускорение делит это значение.
constexpr uint32_t BASE_UNIT_US = 417; // Standard BK tape bit timing, about 1200 baud.
constexpr uint8_t MAX_FILENAME = 16;
// BIN состоит из 4-байтового заголовка и payload длиной до 65535 байт.
constexpr size_t MAX_BIN_FILE_SIZE = 65539; // 4-byte header plus 16-bit payload length.

U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, OLED_SCL, OLED_SDA, U8X8_PIN_NONE);
WebServer server(80);
DNSServer dnsServer;
Preferences settings;
hw_timer_t *tapeTimer = nullptr;
String wifiSsid, wifiPassword, lastSelectedFile, lastSelectedCatalog;
bool wifiApActive = false, wifiSetupRequired = false, wifiConnectPending = false;
uint32_t wifiConnectStarted = 0;


// Состояния поточного кодировщика. Поля, которыми пользуется таймерное
// прерывание, объявлены volatile; составные операции защищаются от ISR.
enum class TapeStage : uint8_t { Idle, Calibration, SyncCal, Marker, SyncMarker, Header, Gap, SyncHeader, Data, EndPulse, Tail, Done };
// Состояние воспроизведения и счётчики используются ISR и главным циклом.
volatile TapeStage stage = TapeStage::Idle;
volatile bool isHighPhase = false;
volatile bool stopRequested = false;
volatile bool isPlaying = false;
volatile bool playbackEnded = false;
volatile uint32_t currentPulseUs = BASE_UNIT_US;
volatile uint32_t currentHighUs = BASE_UNIT_US / 2;
volatile uint8_t audioLevel = 100;
volatile uint32_t activeUnitUs = BASE_UNIT_US;
float playbackSpeed = 1.0f;
volatile uint16_t pulseCount = 0;
volatile uint8_t syncIndex = 0;
volatile uint8_t bitIndex = 0;
volatile uint16_t byteIndex = 0;
volatile uint8_t streamPart = 0; // 0=header, 1=payload, 2=checksum
volatile uint16_t checksum = 0;
volatile uint16_t payloadLength = 0;
volatile uint32_t playedPayloadBits = 0;
volatile uint32_t scheduledSignalUs = 0;
volatile uint32_t totalSignalUs = 0;
volatile bool secondPulseOfBit = false;
volatile const uint8_t *activeData = nullptr;
volatile bool activeDataInFlash = true;
volatile uint32_t audioEdgeCount = 0;
uint8_t tapeHeader[20] = {};
String lengthWarning;
char currentName[64] = "Ready";
String wifiStatus = "Connecting";
uint32_t lastScreenUpdate = 0;
bool filesystemReady = false;
uint8_t *playbackBuffer = nullptr;
File uploadFile;
File linksUploadFile;
String uploadPath;
String uploadTempPath = "/upload.tmp";
String linksUploadLine;
size_t uploadBytes = 0;
size_t linksUploadBytes = 0;
size_t linksUploadCount = 0;
size_t linksUploadLineNumber = 0;
bool linksUploadLimitExceeded = false;
bool uploadAccepted = false;
bool linksUploadAccepted = false;
int uploadResultCode = 200;
String uploadResultMessage = "Upload complete";
int linksUploadResultCode = 200;
String linksUploadResultMessage = "Links saved";
constexpr char CATALOG_LINKS_PATH[] = "/catalog_links.txt";
constexpr char CATALOG_LINKS_TEMP[] = "/catalog_links.tmp";
constexpr size_t MAX_CATALOG_LINKS_TEXT = 1024 * 1024;
constexpr size_t MAX_CATALOG_LINKS = 3000;
constexpr char INITIAL_CATALOG_LINKS[] =
  "https://kalininskiy.github.io/bk-catalog/#game-738\n";
#ifdef PLAYER_AUTOTEST
uint32_t autotestStartedAt = 0;
uint32_t autotestLastLogAt = 0;
bool autotestEndLogged = false;
#endif

// Источник потока для ISR: синтезируемый заголовок БК, затем payload BIN,
// затем два байта контрольной суммы.
uint8_t IRAM_ATTR readActiveByte(uint16_t index) {
  if (streamPart == 0) return tapeHeader[index];
  if (streamPart == 1) {
    return activeDataInFlash ? pgm_read_byte(activeData + 4 + index) : activeData[4 + index];
  }
  return static_cast<uint8_t>(checksum >> (index * 8));
}

void IRAM_ATTR startSync(TapeStage purpose) {
  stage = purpose;
  syncIndex = 0;
}

// Возвращает длительность очередного уровня сигнала в микросекундах и
// переводит конечный автомат между калибровкой, синхронизацией, заголовком,
// данными и завершающими импульсами. Функция должна оставаться в IRAM.
uint32_t IRAM_ATTR nextPulseUs() {
  for (;;) {
    switch (stage) {
      case TapeStage::Calibration:
        if (pulseCount) { --pulseCount; return activeUnitUs; }
        startSync(TapeStage::SyncCal);
        streamPart = 0;
        break;
      case TapeStage::SyncCal:
      case TapeStage::SyncMarker:
      case TapeStage::SyncHeader:
        if (syncIndex < 3) {
          const uint8_t widths[] = {3, 2, 1};
          return activeUnitUs * widths[syncIndex++];
        }
        if (stage == TapeStage::SyncCal) { stage = TapeStage::Marker; pulseCount = 8; }
        else if (stage == TapeStage::SyncMarker) { stage = TapeStage::Header; streamPart = 0; byteIndex = 0; bitIndex = 0; }
        else { stage = TapeStage::Data; streamPart = 1; byteIndex = 0; bitIndex = 0; }
        break;
      case TapeStage::Marker:
        if (pulseCount) { --pulseCount; return activeUnitUs; }
        startSync(TapeStage::SyncMarker);
        break;
      case TapeStage::Header:
      case TapeStage::Data: {
        const uint16_t length = streamPart == 0 ? sizeof(tapeHeader) :
                                streamPart == 1 ? payloadLength : 2;
        if (byteIndex >= length) {
          if (streamPart == 0) { stage = TapeStage::Gap; pulseCount = 8; }
          else if (streamPart == 1) { streamPart = 2; byteIndex = 0; bitIndex = 0; stage = TapeStage::Data; }
          else { stage = TapeStage::EndPulse; }
          break;
        }
        const uint8_t value = readActiveByte(byteIndex);
        if (!secondPulseOfBit) {
          const bool one = (value >> bitIndex) & 1;
          // Each data bit is a long/short pulse followed by a short sync pulse.
          currentPulseUs = one ? activeUnitUs * 2 : activeUnitUs;
          secondPulseOfBit = true;
          return currentPulseUs;
        }
        secondPulseOfBit = false;
        if (streamPart == 1) ++playedPayloadBits;
        if (++bitIndex == 8) { bitIndex = 0; ++byteIndex; }
        return activeUnitUs;
      }
      case TapeStage::Gap:
        if (pulseCount) { --pulseCount; return activeUnitUs; }
        startSync(TapeStage::SyncHeader);
        break;
      case TapeStage::EndPulse:
        stage = TapeStage::Tail;
        pulseCount = 256;
        return activeUnitUs * 6;
      case TapeStage::Tail:
        if (pulseCount) { --pulseCount; return activeUnitUs; }
        stage = TapeStage::Done;
        return 0;
      case TapeStage::Done:
      case TapeStage::Idle:
        return 0;
    }
  }
}

// ISR аппаратного таймера: переключает аудиовыход и назначает следующий
// интервал. Здесь запрещены сетевые запросы, файловая система и выделение RAM.
void IRAM_ATTR tapeTimerIsr() {
  // Отложенное прерывание после останова не должно запускать GPIO снова.
  if (!isPlaying || stage == TapeStage::Idle) {
    GPIO.out_w1tc.val = (1UL << AUDIO_PIN);
    return;
  }
  if (!isHighPhase && !stopRequested) currentPulseUs = nextPulseUs();
  ++audioEdgeCount;
  if (stopRequested || stage == TapeStage::Done) {
    timerStop(tapeTimer);
    timerAlarmDisable(tapeTimer);
    GPIO.out_w1tc.val = (1UL << AUDIO_PIN);
    isHighPhase = false;
    isPlaying = false;
    if (stopRequested) stage = TapeStage::Idle;
    else playbackEnded = true;
    stopRequested = false;
    return;
  }
  if (isHighPhase) {
    GPIO.out_w1tc.val = (1UL << AUDIO_PIN);
    isHighPhase = false;
    const uint32_t lowUs = currentPulseUs - currentHighUs;
    timerAlarmWrite(tapeTimer, lowUs ? lowUs : 1, true);
  } else {
    if (audioLevel) GPIO.out_w1ts.val = (1UL << AUDIO_PIN);
    isHighPhase = true;
    scheduledSignalUs += currentPulseUs;
    currentHighUs = audioLevel ? (currentPulseUs * audioLevel) / 200 : 1;
    if (!currentHighUs) currentHighUs = 1;
    timerAlarmWrite(tapeTimer, currentHighUs, true);
  }
  timerAlarmEnable(tapeTimer);
}

// Экранирует строки для JSON, чтобы кавычки и управляющие символы в имени
// файла или SSID не повредили структуру ответа API.
String jsonEscape(const char *s) {
  String out;
  for (; *s; ++s) { if (*s == '"' || *s == '\\') out += '\\'; out += *s; }
  return out;
}

uint8_t countSetBits(uint8_t value) {
  uint8_t count = 0;
  while (value) { count += value & 1; value >>= 1; }
  return count;
}

uint8_t readBinByte(const uint8_t *data, bool inFlash, size_t offset) {
  return inFlash ? pgm_read_byte(data + offset) : data[offset];
}

// Точная длительность подготовленного потока: учитываются заголовок,
// данные, checksum и выбранный интервал базового бита.
uint32_t estimateSignalDurationUs(const uint8_t *data, bool inFlash, uint16_t length, const uint8_t *header, uint32_t unitUs) {
  // Sum of all pulse-cycle widths, in units of the selected playback speed.
  uint32_t units = 4096 + 6 + 8 + 6 + 8 + 6 + 6 + 256;
  uint32_t oneBits = 0;
  for (size_t i = 0; i < 20; ++i) oneBits += countSetBits(header[i]);
  for (uint32_t i = 0; i < uint32_t(length) + 2; ++i) {
    const uint8_t value = i < length ? readBinByte(data, inFlash, 4 + i) :
      static_cast<uint8_t>(checksum >> ((i - length) * 8));
    oneBits += countSetBits(value);
  }
  units += (20UL * 8 * 2) + oneBits;
  units += ((uint32_t(length) + 2) * 8 * 2);
  return units * unitUs;
}

uint8_t playbackPercent() {
  const uint32_t totalBits = uint32_t(payloadLength) * 8;
  if (!totalBits) return 0;
  const uint32_t percent = (playedPayloadBits * 100UL) / totalBits;
  return percent > 100 ? 100 : percent;
}

uint32_t remainingSeconds() {
  const uint32_t total = totalSignalUs;
  const uint32_t elapsed = scheduledSignalUs;
  if (!isPlaying || elapsed >= total) return 0;
  return (total - elapsed + 999999UL) / 1000000UL;
}

void drawProgressLine() {
  const uint32_t seconds = remainingSeconds();
  char line[24];
  snprintf(line, sizeof(line), "DATA %3u%% ETA %02lu:%02lu", playbackPercent(),
           static_cast<unsigned long>(seconds / 60), static_cast<unsigned long>(seconds % 60));
  display.drawStr(6, 43, line);
  display.drawFrame(6, 46, 116, 6);
  display.drawBox(7, 47, 114 * playbackPercent() / 100, 4);
}

// Отдаёт состояние плеера веб-интерфейсу. Пароль Wi-Fi в JSON не включается.
void sendStatus() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  const String ip = connected ? WiFi.localIP().toString() : wifiApActive ? WiFi.softAPIP().toString() : "-";
  const String currentSsid = connected ? WiFi.SSID() : "";
  const String currentNetwork = connected ? WiFi.SSID() : wifiApActive ? "BK_PLAYER" : "";
  String json = "{\"lengthWarning\":\"" + lengthWarning + "\",\"playing\":" + String(isPlaying ? "true" : "false") +
    ",\"file\":\"" + jsonEscape(currentName) + "\",\"wifi\":\"" + wifiStatus +
    "\",\"ip\":\"" + ip + "\",\"ap_ip\":\"" + (wifiApActive ? WiFi.softAPIP().toString() : "") +
    "\",\"ssid\":\"" + jsonEscape(currentSsid.c_str()) + "\",\"current_network\":\"" + jsonEscape(currentNetwork.c_str()) + "\",\"wifi_connected\":" + String(connected ? "true" : "false") +
    ",\"wifi_setup_required\":" + String(wifiSetupRequired ? "true" : "false") +
    ",\"menu_revision\":" + String(menuRevision) +
    ",\"speed\":" + String(playbackSpeed, 1) + ",\"percent\":" + String(playbackPercent()) +
    ",\"remaining_seconds\":" + String(remainingSeconds()) + "}";
  server.send(200, "application/json; charset=utf-8", json);
}

// Веб-страница хранится во flash программы. Favicon встроен в HTML из
// reference/cassete.png и не требует отдельной загрузки LittleFS.
const char INDEX_HTML[] PROGMEM = R"HTML(<!doctype html><html lang="ru"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Плеер БК0010...11М</title><link rel="icon" type="image/png" href="data:image/png;base64,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"><style>
body{font:16px system-ui;background:#101722;color:#eef4ff;max-width:560px;margin:32px auto;padding:0 18px}main{background:#1b2838;border-radius:16px;padding:24px}h1{text-align:center;font-size:24px;margin-top:0}select,input,button,textarea{font:inherit;padding:10px;border:0;border-radius:9px;box-sizing:border-box}
textarea{display:block;background:#eef4ff;color:#101722;line-height:1.35;min-height:42px;max-height:220px;overflow:hidden;resize:none;white-space:pre-wrap;overflow-wrap:anywhere;word-break:break-word}select{width:100%;margin:8px 0 14px;background:#eef4ff}button{height:42px;box-sizing:border-box;cursor:pointer;margin:0}#play,#upload,#remotePlay,#uploadLinks,#bin::file-selector-button,#linksText::file-selector-button{background:#45d483;color:#102219}#bin::file-selector-button,#linksText::file-selector-button{height:42px;box-sizing:border-box;font:inherit;padding:10px;border:0;border-radius:9px;cursor:pointer;margin-right:8px}#stop{background:#f07878}.status{line-height:1.7;margin-top:18px;color:#becde0}.muted{color:#93a6bd}.upload{border-top:1px solid #35475c;margin-top:20px;padding-top:16px}.player-row{display:flex;align-items:center;gap:8px;margin:6px 0 14px}.player-row label{white-space:nowrap}.file-row .file-select-wrap{position:relative;flex:1;min-width:0}.file-picker-button{display:flex;align-items:center;gap:12px;width:100%;height:42px;margin:0;padding:0 12px;background:#eef4ff;color:#101722;text-align:left}.file-display{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.file-meta{flex:none;white-space:nowrap;font-size:12px;color:#58677b;font-variant-numeric:tabular-nums}.file-pin,.file-option-pin{font-family:"Segoe UI Symbol",sans-serif;font-size:17px;line-height:1;color:#526278;transform:rotate(-50deg)}.file-pin[hidden]{display:none}.file-chevron{font-size:14px;color:#526278}.file-options{position:absolute;z-index:30;top:46px;left:0;right:0;max-height:260px;overflow:auto;border-radius:9px;background:#eef4ff;color:#101722;box-shadow:0 8px 24px #0008}.file-options[hidden]{display:none}.file-option{display:flex;align-items:center;justify-content:space-between;width:100%;height:38px;margin:0;padding:0 14px;border-radius:0;background:#eef4ff;color:#101722;text-align:left}.file-option:hover,.file-option[aria-selected=true]{background:#dbe5f4}.file-option-name{flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}.file-option-meta{flex:none;margin-left:10px;text-align:right;white-space:nowrap;font-size:12px;color:#58677b;font-variant-numeric:tabular-nums}.file-option-pin{flex:none;margin-left:12px}.controls-row{gap:6px;flex-wrap:wrap}.controls-row #play,.controls-row #stop,.controls-row #delete{width:72px;flex:0 0 72px}.controls-row button{font-size:14px;white-space:nowrap;padding:8px 5px;margin:0}.controls-row label{font-size:14px;margin-left:auto}#speed{width:78px;height:42px;flex:0 0 78px;padding:8px 5px;margin:0}.catalog-web-link{display:block;margin:8px 0}.upload-file-btn{display:inline-flex;align-items:center;justify-content:center;width:72px;flex:0 0 72px;height:42px;background:#e4bd55;color:#30270f;border-radius:9px;box-sizing:border-box;cursor:pointer;font-size:14px}.controls-row #bin{display:none}.protect-row{display:flex;align-items:center;gap:8px;margin:8px 0;color:#93a6bd}.protect-row input{width:auto} .audio-level-row{display:flex;align-items:center;gap:8px;flex-wrap:nowrap;margin:8px 0;color:#93a6bd}.audio-level-row span{margin-left:auto;color:#eef4ff;font-variant-numeric:tabular-nums}.audio-level-row input[type=range]{flex:1 1 0;min-width:40px;width:auto;margin:0;accent-color:#45d483} .link-button{display:inline-flex;align-items:center;justify-content:center;height:42px;padding:8px 12px;border-radius:9px;background:#45d483;color:#102219;text-decoration:none;box-sizing:border-box;margin:0;cursor:pointer} .file-actions,.links-actions{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin:8px 0} .file-actions input[type=file],.links-actions input[type=file]{display:none}.catalog-picker-wrap{position:relative;margin:8px 0 14px}.catalog-picker-button{display:flex;align-items:center;justify-content:space-between;width:100%;height:42px;margin:0;padding:0 12px;background:#eef4ff;color:#101722;text-align:left}.catalog-display{display:flex;min-width:0;gap:8px;overflow:hidden;white-space:nowrap}.catalog-display-number,.catalog-option-number{color:#7c8999;flex:none}.catalog-display-name,.catalog-option-name{font-weight:600}.catalog-option{display:flex;align-items:flex-start;justify-content:flex-start;gap:8px}.catalog-option-number{width:48px;text-align:right}.wifi-settings-button{margin-top:12px}dialog{border:0;border-radius:14px;padding:22px;max-width:min(420px,calc(100vw - 36px));background:#1b2838;color:#eef4ff;box-shadow:0 12px 48px #0009}dialog::backdrop{background:#0009}dialog label{display:block;margin-top:12px}dialog input{width:100%;margin-top:4px}dialog button{margin-top:12px}.dialog-actions{display:flex;justify-content:space-between;gap:12px;margin-top:12px}.dialog-actions button{width:calc((100% - 12px)/2);margin:0}.name-options{display:grid;grid-template-columns:minmax(0,auto) minmax(4ch,1fr) auto;align-items:center;gap:6px}
.name-options #tapeName{width:100%;min-width:0;padding-inline:5px}
@media(max-width:420px){.name-options{font-size:13px;gap:4px}.name-options .protect-row{gap:3px}}
#filePickerButton{background-image:linear-gradient(to right,#c4e9cb var(--played,0%),transparent var(--played,0%))}
.catalog-sort{display:grid;grid-template-columns:auto minmax(0,1fr) auto minmax(0,auto);align-items:center;gap:6px;margin:8px 0}
.catalog-sort input,.catalog-sort select{width:100%;min-width:0;height:38px;margin:0;padding:4px 6px}
@media(max-width:420px){.catalog-sort{font-size:13px;gap:4px}.catalog-sort select{max-width:104px}}@media(max-width:420px){body{margin:16px auto;padding:0 10px}main{padding:16px}.player-row{gap:4px}.controls-row{gap:4px}.controls-row #play,.controls-row #stop,.controls-row #delete,.upload-file-btn{width:58px;flex-basis:58px}.controls-row button{font-size:12px;padding:7px 3px}.controls-row label{font-size:12px}#speed{width:68px;flex-basis:68px}.upload-file-btn{font-size:12px}}
.file-details{display:flex;align-items:center;gap:6px;min-width:0}
#filePickerButton.playing-selection{display:grid;grid-template-columns:minmax(0,1fr) auto minmax(0,1fr);gap:5px}
#fileEta{color:#45d483;white-space:nowrap;font-weight:700;font-variant-numeric:tabular-nums}
.playing-selection .file-details{justify-content:flex-end}
.playing-selection .file-meta{flex:0 1 auto;min-width:0;white-space:normal;text-align:right}
</style><main><h1>Плеер БК0010...11М</h1><div class="player-row file-row"><label for="file">Файл BIN</label><div class="file-select-wrap"><select id="file" hidden></select><button id="filePickerButton" type="button" class="file-picker-button"><span id="fileDisplay" class="file-display"></span><span id="fileEta" hidden></span><span class="file-details"><span id="fileMeta" class="file-meta"></span><span id="filePin" class="file-pin" aria-hidden="true" hidden>&#128206;&#65038;</span><span class="file-chevron">&#9662;</span></span></button><div id="fileOptions" class="file-options" hidden></div></div></div><div class="player-row controls-row"><button id="play" onclick="playSelected()"><span aria-hidden="true">▶</span> Пуск</button><button id="stop" onclick="cmd('/stop')"><span aria-hidden="true">■</span> Стоп</button><input id="bin" type="file" accept=".bin"><label class="upload-file-btn" for="bin">Загрузить</label><button id="delete" onclick="deleteSelected()">Удалить</button><label for="speed">Скорость</label><select id="speed"><option value="1.0">1,0×</option><option value="1.5">1,5×</option><option value="2.0">2,0×</option><option value="2.5">2,5×</option><option value="3.0">3,0×</option></select></div><div class="name-options"><label class="protect-row"><input type="checkbox" id="overrideTapeName" onchange="document.getElementById('tapeName').disabled=!this.checked"> Заменить имя в заголовке ленты</label><input type="text" id="tapeName" maxlength="16" value="V" aria-label="Имя на ленте" disabled><label class="protect-row"><input type="checkbox" id="upperTapeName"> ЗАГЛАВНЫМИ</label></div><label class="protect-row"><input id="protectFile" type="checkbox" onchange="setProtected()"> Не удалять автоматически при загрузке новых BIN &#128206;&#65038;</label><label class="audio-level-row" for="audioLevel">Уровень сигнала <span id="audioLevelValue">100%</span><input id="audioLevel" type="range" min="0" max="100" value="100" oninput="showAudioLevel(this.value)" onchange="saveAudioLevel(this.value)"></label><span id="uploadResult"></span><div class="upload"><strong>Сохранённые ссылки</strong><div class="catalog-sort"><label class="catalog-search-label" for="catalogSearch">Поиск</label><input id="catalogSearch" type="search" placeholder="Имя файла или ссылка" oninput="renderCatalogLinks()"><label for="catalogSort">Сортировка</label><select id="catalogSort" onchange="renderCatalogLinks()"><option value="file">Как в файле</option><option value="alpha">По алфавиту</option></select></div><div class="catalog-picker-wrap"><select id="catalogFiles" hidden></select><button id="catalogPickerButton" type="button" class="catalog-picker-button"><span id="catalogDisplay"></span><span class="file-chevron">&#9662;</span></button><div id="catalogOptions" class="file-options" hidden></div></div><label for="remoteUrl">Ссылка на файл</label><textarea id="remoteUrl" rows="1" placeholder="Вставьте ссылку на BIN или ZIP" oninput="onManualUrlInput()" style="width:100%;margin:8px 0"></textarea><button id="remotePlay" onclick="playRemote()">Скачать и запустить</button><span id="remoteResult"></span><div id="zipPicker" hidden><label for="zipMember">Файл BIN в ZIP-архиве</label><select id="zipMember"></select><button id="extractZip" onclick="extractSelectedZip()">Распаковать и запустить</button><button onclick="cancelZip()" style="margin-left:18px">Отмена</button></div><div class="links-actions"><a class="link-button" href="/catalog_links.txt" download="catalog_links.txt">Скачать список ссылок</a><input id="linksText" type="file" onchange="uploadLinks()"><label class="link-button" for="linksText">Загрузить список ссылок</label></div><span id="linksResult"></span><p class="muted">Добавленные вручную ссылки сохранятся в списке. ZIP-архив распаковывается в браузере. Если BIN-файлов несколько, выберите нужный.</p></div><p id="lengthWarning" role="status" style="color:#e4bd55" hidden></p><div class="status" id="status">Подключение...</div><p class="muted" id="storage">Память: проверка...</p><button type="button" class="wifi-settings-button" onclick="openWifiDialog()">Настройки Wi-Fi</button><dialog id="wifiDialog"><h2>Подключение к Wi-Fi</h2><p id="currentWifi" class="muted">Current network: checking...</p><label for="wifiSsid">Имя сети (SSID)</label><input id="wifiSsid" autocomplete="off"><label for="wifiPassword">Пароль</label><input id="wifiPassword" type="password" autocomplete="new-password"><p id="wifiMessage" class="muted"></p><div class="dialog-actions"><button type="button" onclick="connectWifi()">Подключиться</button><button type="button" onclick="closeWifiDialog()">Закрыть</button></div></dialog></main><script>
// Связывает элементы интерфейса со списками BIN и синхронизирует выбор с API устройства.
const file=document.querySelector('#file'),speed=document.querySelector('#speed'),status=document.querySelector('#status'),fileWrap=document.querySelector('.file-select-wrap'),fileOptions=document.querySelector('#fileOptions'),filePin=document.querySelector('#filePin'),fileDisplay=document.querySelector('#fileDisplay'),fileMeta=document.querySelector('#fileMeta'),protectBox=document.querySelector('#protectFile');document.querySelector('#bin').onchange=uploadBin;speed.onchange=async()=>{await saveControl('speed',speed.value);await loadFiles(file.selectedOptions[0]?.textContent||'')};function showAudioLevel(value){document.querySelector('#audioLevelValue').textContent=value+'%'}async function saveAudioLevel(value){showAudioLevel(value);try{const response=await fetch('/audio-level?value='+encodeURIComponent(value),{method:'POST'});if(!response.ok)throw Error(await response.text())}catch(e){document.querySelector('#uploadResult').textContent='Could not save audio level: '+e.message}}async function loadAudioLevel(){try{const response=await fetch('/api/audio-level');if(!response.ok)return;const value=Number(await response.text());document.querySelector('#audioLevel').value=value;showAudioLevel(value)}catch(e){}}let filePinState=[],fileNames=[],fileInfo=[];const paperclip=String.fromCodePoint(0x1F4CE)+String.fromCharCode(0xFE0E);function cleanFileLabel(s){return s.trim()}let activeBinName='', activeBinPercent=0, activeRemainingSeconds=0;
function updateFileProgress(){
  const selectedName=file.selectedOptions[0]?.textContent;
  const playing=Boolean(activeBinName && selectedName===activeBinName);
  const percent=playing ? activeBinPercent : 0;
  const eta=document.querySelector('#fileEta');
  eta.hidden=!playing;
  eta.textContent=playing ? percent+'% · '+fmt(activeRemainingSeconds) : '';
  document.querySelector('#filePickerButton').classList.toggle('playing-selection',playing);
  document.querySelector('#filePickerButton').style.setProperty('--played',percent+'%');
}
function updatePinIcon(){updateFileProgress();const option=file.selectedOptions[0],index=option?Number(option.value):-1;fileDisplay.textContent=option?option.textContent+(fileInfo[index]?.system?' *':''):'';const info=fileInfo[index]||{};fileMeta.textContent=option?Number(info.size||0).toLocaleString()+' B · ~'+formatFileTime(Number(info.duration_seconds||0)):'';filePin.hidden=index<0||!filePinState[index]}function updateDelete(){document.querySelector('#delete').disabled=!file.selectedOptions[0];updatePinIcon()}function formatFileTime(seconds){return Math.floor(seconds/60)+':'+String(seconds%60).padStart(2,'0')}function renderFilePicker(){fileOptions.replaceChildren();fileNames.forEach((name,i)=>{const row=document.createElement('button');row.type='button';row.className='file-option';row.setAttribute('aria-selected',String(file.value===String(i)));const label=document.createElement('span');label.className='file-option-name';label.textContent=name+(fileInfo[i]?.system?' *':'');row.append(label);const meta=document.createElement('span');meta.className='file-option-meta';const info=fileInfo[i]||{};meta.textContent=Number(info.size||0).toLocaleString()+' B · ~'+formatFileTime(Number(info.duration_seconds||0));row.append(meta);if(filePinState[i]){const icon=document.createElement('span');icon.className='file-option-pin';icon.textContent=paperclip;icon.setAttribute('aria-label','Protected from automatic deletion');row.append(icon)}row.onclick=()=>{file.value=String(i);fileOptions.hidden=true;file.dispatchEvent(new Event('change'))};fileOptions.append(row)})}document.querySelector('#filePickerButton').onclick=()=>{fileOptions.hidden=!fileOptions.hidden};document.addEventListener('click',e=>{if(!fileWrap.contains(e.target))fileOptions.hidden=true});async function loadFiles(selectName=''){let selectedName=file.selectedOptions[0]?.textContent||'';file.replaceChildren();const entries=(await(await fetch('/api/files?speed='+encodeURIComponent(speed.value))).json()).map((entry,index)=>({...entry,id:entry.id??index})).sort((a,b)=>a.name.localeCompare(b.name,'ru',{sensitivity:'base',numeric:true}));fileInfo=entries;fileNames=entries.map(x=>x.name);filePinState=entries.map(x=>x.protected);entries.forEach((x,i)=>file.add(new Option(x.name,i)));if(selectName){const found=entries.findIndex(x=>x.name.toLowerCase()===selectName.toLowerCase());if(found>=0)file.value=String(found)}else if(selectedName){const found=entries.findIndex(x=>x.name.toLowerCase()===selectedName.toLowerCase());if(found>=0)file.value=String(found)}renderFilePicker();updateDelete();refreshProtection()}file.onchange=()=>{renderFilePicker();updateDelete();refreshProtection();persistSelections()};function selectedFileServerId(){const entry=fileInfo[Number(file.value)];return entry?entry.id:-1}let menuRevisionSeen = -1;
let pendingSetting = Promise.resolve();
function saveControl(key, value) {
  pendingSetting = pendingSetting.catch(()=>{}).then(async()=>{
    const result = await fetch('/selection?' + new URLSearchParams({[key]:value}));
    if (!result.ok) throw Error(await result.text());
  });
  pendingSetting.catch(error=>document.querySelector('#uploadResult').textContent=error.message);
  return pendingSetting;
}
const replaceBox = document.querySelector('#overrideTapeName');
const tapeInput = document.querySelector('#tapeName');
const upperBox = document.querySelector('#upperTapeName');
replaceBox.onchange = ()=>{tapeInput.disabled=!replaceBox.checked;saveControl('replace', Number(replaceBox.checked))};
tapeInput.onchange = ()=>saveControl('tape_name', tapeInput.value);
upperBox.onchange = ()=>saveControl('upper', Number(upperBox.checked));
function applySavedControls(data) {
  speed.value=Number(data.speed).toFixed(1);
  replaceBox.checked=data.replace;
  tapeInput.disabled=!data.replace;
  if(document.activeElement!==tapeInput) tapeInput.value=data.name;
  upperBox.checked=data.upper;
  menuRevisionSeen=data.menu_revision;
}
async function playbackUrl(id){await pendingSetting;return '/play?id='+encodeURIComponent(id)}async function playSelected(){const id=selectedFileServerId();if(id>=0)cmd(await playbackUrl(id))}async function refreshProtection(){const option=file.selectedOptions[0];protectBox.disabled=!option;if(!option){protectBox.checked=false;updatePinIcon();return}try{const info=await(await fetch('/api/protect?id='+selectedFileServerId())).json();protectBox.checked=info.protected;filePinState[Number(file.value)]=info.protected}catch(e){protectBox.checked=false}renderFilePicker();updatePinIcon()}async function setProtected(){if(fileInfo[Number(file.value)]?.system){protectBox.checked=true;alert("This is a system file and cannot be deleted.");return}const index=Number(file.value),serverId=selectedFileServerId(),keep=protectBox.checked,response=await fetch('/protect?id='+serverId+'&keep='+(keep?'1':'0'));if(response.ok){filePinState[index]=keep;renderFilePicker();updatePinIcon()}else await refreshProtection()}async function deleteSelected(){if(!confirm('Delete selected BIN file?'))return;let r=await fetch('/delete?id='+selectedFileServerId());document.querySelector('#uploadResult').textContent=await r.text();if(r.ok){await loadFiles();await persistSelections();refreshStorage()}}let catalogLinkData=[],remoteBusy=false,pendingZip=null;let manualRemoteUrl=false,manualLinkName="",manualLinkUrl="";function renderCatalogPicker(){const select=document.querySelector('#catalogFiles'),options=document.querySelector('#catalogOptions'),display=document.querySelector('#catalogDisplay');options.replaceChildren();for(const opt of select.options){const isManual=opt.value==='manual',entry=isManual?{line:0,name:manualLinkName,url:manualLinkUrl}:catalogLinkData[Number(opt.value)];if(!entry)continue;const row=document.createElement('button');row.type='button';row.className='file-option catalog-option';row.setAttribute('aria-selected',String(select.value===opt.value));if(entry.line){const line=document.createElement('span');line.className='catalog-option-number';line.textContent=entry.line+': ';row.append(line)}const name=document.createElement('span');name.className='catalog-option-name';name.textContent=entry.name;row.append(name);row.onclick=()=>{select.value=opt.value;options.hidden=true;if(isManual){document.querySelector('#remoteUrl').value=manualLinkUrl;resizeRemoteUrl();manualRemoteUrl=true;renderCatalogPicker()}else selectCatalogLink(true)};options.append(row)}const selected=select.value==='manual'?{line:0,name:manualLinkName}:catalogLinkData[Number(select.value)];display.replaceChildren();if(selected){if(selected.line){const line=document.createElement('span');line.className='catalog-display-number';line.textContent=selected.line+': ';display.append(line)}const name=document.createElement('span');name.className='catalog-display-name';name.textContent=selected.name;display.append(name)}}document.querySelector('#catalogPickerButton').onclick=()=>{const options=document.querySelector('#catalogOptions');options.hidden=!options.hidden};document.addEventListener('click',e=>{if(!document.querySelector('.catalog-picker-wrap').contains(e.target))document.querySelector('#catalogOptions').hidden=true});function renderCatalogLinks(preferredUrl=''){const select=document.querySelector('#catalogFiles'),old=select.value,query=(document.querySelector('#catalogSearch')?.value||'').trim().toLocaleLowerCase(),order=catalogLinkData.map((_,i)=>i).filter(i=>!query||catalogLinkData[i].name.toLocaleLowerCase().includes(query)||catalogLinkData[i].url.toLocaleLowerCase().includes(query));if(document.querySelector('#catalogSort').value==='alpha')order.sort((a,b)=>catalogLinkData[a].name.localeCompare(catalogLinkData[b].name,undefined,{sensitivity:'base'}));select.replaceChildren();order.forEach(i=>select.add(new Option(catalogLinkData[i].name,String(i))));const preferred=preferredUrl?catalogLinkData.findIndex(x=>x.url===preferredUrl):-1;if(preferred>=0&&order.includes(preferred))select.value=String(preferred);else if(old&&order.includes(Number(old)))select.value=old;else if(order.length)select.value=String(order[0]);renderCatalogPicker();if(order.length)selectCatalogLink(false)}async function loadCatalogLinks(preferredUrl=''){try{catalogLinkData=await(await fetch('/api/catalog-links')).json();renderCatalogLinks(preferredUrl)}catch(e){document.querySelector('#linksResult').textContent='Saved catalog links are unavailable'}}async function persistSelections(){const name=file.selectedOptions[0]?.textContent||'',item=catalogLinkData[Number(document.querySelector('#catalogFiles').value)],url=item?item.url:'';await fetch('/selection?file='+encodeURIComponent(name)+'&catalog='+encodeURIComponent(url))}function selectCatalogLink(persist=true){const item=catalogLinkData[Number(document.querySelector('#catalogFiles').value)];if(item){document.querySelector('#remoteUrl').value=item.url;onManualUrlInput(false);manualRemoteUrl=false;renderCatalogPicker();if(persist)persistSelections()}}async function uploadLinks(){const input=document.querySelector('#linksText'),message=document.querySelector('#linksResult'),selected=input.files[0];if(!selected)return;if(!selected.name.toLowerCase().endsWith('.txt')){message.textContent='Choose a TXT file';return}try{const body=new FormData();body.append('file',selected,selected.name);const response=await fetch('/catalog-links',{method:'POST',body});message.textContent=await response.text();if(response.ok){input.value='';await loadCatalogLinks()}}catch(e){message.textContent='Could not upload catalog_links.txt'}}function unwrapCatalogInput(input){const value=input.trim(),match=value.match(/^\[[^\]]+\]\((https?:\/\/[^)]+)\)$/i);return match?match[1]:value}function catalogRawUrl(input){let url=unwrapCatalogInput(input).replace(/[.,;!?]+$/,'');const pages='https://kalininskiy.github.io/bk-catalog/';if(url.startsWith(pages))url='https://raw.githubusercontent.com/kalininskiy/bk-catalog/main/'+url.substring(pages.length);else if(url.startsWith('https://github.com/kalininskiy/bk-catalog/blob/main/'))url='https://raw.githubusercontent.com/kalininskiy/bk-catalog/main/'+url.substring('https://github.com/kalininskiy/bk-catalog/blob/main/'.length);return url}// Разбирает якорь карточки каталога вида #software-ID и выбирает JSON-набор нужного раздела.
function isBinZipUrl(input){try{const parsed=new URL(input);return (parsed.protocol==='http:'||parsed.protocol==='https:')&&/\.(bin|zip)$/i.test(parsed.pathname)}catch(e){return false}}function parseCatalogPageUrl(input){try{const page=new URL(unwrapCatalogInput(input));if(page.protocol!=='http:'&&page.protocol!=='https:')return null;if(isBinZipUrl(page.href))return null;const isBkCatalog=page.origin==='https://kalininskiy.github.io'&&page.pathname.replace(/\/+$/,'')==='/bk-catalog';const match=isBkCatalog?page.hash.match(/^#(game|games|software|demoscene)-(\d+)$/i):null;if(match){const section=match[1].toLowerCase();return{category:section==='game'?'games':section,id:Number(match[2]),url:page.href}}return{url:page.href}}catch(e){return null}}// Находит запись по ID, извлекает поля «Имя файла N» и оставляет только BIN/ZIP каталога.
async function getBkCatalogPageFiles(page){const source='https://raw.githubusercontent.com/kalininskiy/bk-catalog/main/content/'+page.category+'.json',response=await fetch(source);if(!response.ok)throw Error('Could not load catalog data: HTTP '+response.status);const rows=await response.json(),record=rows.find(item=>Number(item.ID)===page.id);if(!record)throw Error('Catalog entry '+page.id+' was not found');const files=[],seen=new Set();for(let i=1;i<=5;i++){const value=record['Имя файла '+i];if(typeof value!=='string'||!value.trim())continue;const url=catalogRawUrl(value.trim()),path=url.toLowerCase().split('?')[0];if(!allowedCatalogDownloadUrl(url)||seen.has(url))continue;let name=record['Описание файла '+i];if(typeof name!=='string'||!name.trim()||name.includes('/')||name.includes('\\'))name=decodeURIComponent(path.split('/').pop());else name=name.trim();if(name.length>180)name=decodeURIComponent(path.split('/').pop());files.push({url,name});seen.add(url)}if(!files.length)throw Error('This catalog entry has no downloadable BIN/ZIP files');return files}async function getGenericPageFiles(page){const response=await fetch(page.url,{cache:'no-store'});if(!response.ok)throw Error('Could not load page: HTTP '+response.status);const html=await response.text(),doc=new DOMParser().parseFromString(html,'text/html'),files=[],seen=new Set(),candidates=[];for(const anchor of doc.querySelectorAll('a[href]'))candidates.push({value:anchor.getAttribute('href'),label:(anchor.textContent||'').trim()});for(const match of html.matchAll(/https?:\/\/[^\s"'<>]+?\.(?:bin|zip)(?:[?#][^\s"'<>]*)?/gi))candidates.push({value:match[0],label:''});for(const candidate of candidates){let url;try{url=new URL(candidate.value.replaceAll('\\/','/'),page.url).href}catch(e){continue}url=catalogRawUrl(url);if(!allowedCatalogDownloadUrl(url)||seen.has(url))continue;let name=candidate.label;if(!name||name.length>180||name.includes('/')||name.includes('\\')){try{name=decodeURIComponent(new URL(url).pathname.split('/').pop())}catch(e){name=new URL(url).pathname.split('/').pop()}}files.push({url,name});seen.add(url)}if(!files.length)throw Error('No BIN or ZIP links found on this page');return files}function allowedCatalogDownloadUrl(input){return isBinZipUrl(input)&&/^https?:\/\//i.test(input)}async function getCatalogPageFiles(page){return page.category?getBkCatalogPageFiles(page):getGenericPageFiles(page)}// Сохраняет найденные файлы в локальный список; один файл сразу запускает, несколько предлагает выбрать.
async function importCatalogPage(page){const message=document.querySelector('#remoteResult'),button=document.querySelector('#remotePlay');remoteBusy=true;button.disabled=true;message.textContent='Reading files from the catalog entry...';try{const files=await getCatalogPageFiles(page);let saved=0;for(const item of files){const response=await fetch('/catalog-link/add',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({url:item.url,name:item.name})});if(response.ok)saved++;else throw Error(await response.text())}await loadCatalogLinks(files[0].url);if(files.length===1){document.querySelector('#remoteUrl').value=files[0].url;onManualUrlInput(false);manualRemoteUrl=false;message.textContent='Catalog file found. Downloading...';await playRemote()}else message.textContent='Found '+files.length+' files ('+saved+' saved). Choose one in Saved catalog links, then press Download and run.'}catch(e){message.textContent='Could not read files from catalog page: '+e.message}finally{remoteBusy=false;button.disabled=false}}async function uploadAndPlay(blob,name){if(blob.size<5||blob.size>65539)throw Error('BIN file size must be 5 to 65539 bytes');if(!name.toLowerCase().endsWith('.bin'))name+='.bin';const data=new FormData();data.append('file',new File([blob],name,{type:'application/octet-stream'}));const uploaded=await fetch('/upload',{method:'POST',body:data}),uploadMessage=await uploaded.text();if(!uploaded.ok)throw Error(uploadMessage);await loadFiles(name);await persistSelections();await refreshStorage();const names=await(await fetch('/api/files')).json(),id=names.findIndex(n=>n.name.toLowerCase()===name.toLowerCase());if(id<0)throw Error('Uploaded file was not found in the list');await loadFiles(name);const selectedId=selectedFileServerId();const started=await fetch(await playbackUrl(selectedId));if(!started.ok)throw Error(await started.text());messageFor('Downloaded and playing: '+name);refresh()}function resizeRemoteUrl(){const input=document.querySelector('#remoteUrl');input.style.height='auto';input.style.height=Math.min(220,Math.max(42,input.scrollHeight))+'px'}function onManualUrlInput(manual=true){resizeRemoteUrl();manualRemoteUrl=manual;const original=document.querySelector('#remoteUrl').value.trim(),page=parseCatalogPageUrl(original);let value=unwrapCatalogInput(original).split(/[?#]/)[0];while(value.endsWith('/'))value=value.slice(0,-1);let name=page?(page.id?'Catalog item '+page.id:'Web page'):value.slice(value.lastIndexOf('/')+1);try{if(!page)name=decodeURIComponent(name)}catch(e){}manualLinkName=name;manualLinkUrl=original;const select=document.querySelector('#catalogFiles'),old=select.querySelector('option[value="manual"]');if(manual&&name){if(!old)select.add(new Option(name,'manual'));else old.textContent=name;select.value='manual'}else if(old){old.remove();if(select.value==='manual'&&select.options.length)select.value=select.options[0].value}renderCatalogPicker()}function messageFor(s){document.querySelector('#remoteResult').textContent=s}function zipEndOffset(bytes){const view=new DataView(bytes),start=Math.max(0,bytes.byteLength-65557);for(let i=bytes.byteLength-22;i>=start;i--)if(view.getUint32(i,true)===0x06054b50)return i;return -1}// Decodes a ZIP member name using the UTF-8 flag, Unicode Path extra field, or legacy Russian code pages.
function decodeZipName(bytes,offset,length,flags,extraOffset,extraLength,view){const raw=new Uint8Array(bytes,offset,length),utf8=new TextDecoder('utf-8');if(flags&0x800)return utf8.decode(raw);let end=extraOffset+extraLength,at=extraOffset;while(at+4<=end){const id=view.getUint16(at,true),size=view.getUint16(at+2,true);at+=4;if(at+size>end)break;if(id===0x7075&&size>=5&&view.getUint8(at)===1){const nameBytes=new Uint8Array(bytes,at+5,size-5);try{return utf8.decode(nameBytes)}catch(e){}}at+=size}try{return new TextDecoder('utf-8',{fatal:true}).decode(raw)}catch(e){}
  // В старых ZIP кодировка не указана. Сравниваем всё имя:
  // байт E8, например, встречается и в DOS-866, и в Windows-1251.
  const dos = new TextDecoder('ibm866').decode(raw);
  const windows = new TextDecoder('windows-1251').decode(raw);
  const russianLetters = name => (name.match(/[А-Яа-яЁё]/g) || []).length;
  const difference = russianLetters(dos) - russianLetters(windows);
  if (difference !== 0) return difference > 0 ? dos : windows;
  return raw.some(byte => byte >= 0xc0) ? windows : dos;
}
// Читает центральный каталог ZIP и выбирает поддерживаемые BIN-записи.
function parseZip(bytes){const view=new DataView(bytes),eocd=zipEndOffset(bytes);if(eocd<0)throw Error('Could not read ZIP archive');if(view.getUint16(eocd+4,true)!==0||view.getUint16(eocd+6,true)!==0)throw Error('Multi-volume ZIP archives are not supported');const count=view.getUint16(eocd+10,true),centralSize=view.getUint32(eocd+12,true),centralOffset=view.getUint32(eocd+16,true);if(centralOffset+centralSize>eocd)throw Error('Corrupt ZIP archive');const entries=[];let hasNestedZip=false,at=centralOffset;for(let i=0;i<count;i++){if(at+46>bytes.byteLength||view.getUint32(at,true)!==0x02014b50)throw Error('Corrupt ZIP directory entry');const flags=view.getUint16(at+8,true),method=view.getUint16(at+10,true),compressed=view.getUint32(at+20,true),size=view.getUint32(at+24,true),nameLen=view.getUint16(at+28,true),extraLen=view.getUint16(at+30,true),commentLen=view.getUint16(at+32,true),local=view.getUint32(at+42,true),name=decodeZipName(bytes,at+46,nameLen,flags,at+46+nameLen,extraLen,view);const lower=name.toLowerCase();if(!name.endsWith('/')&&lower.endsWith('.zip'))hasNestedZip=true;if(!name.endsWith('/')&&lower.endsWith('.bin')){if(flags&1)throw Error('Encrypted BIN files in ZIP archives are not supported');if(method!==0&&method!==8)throw Error('Unsupported ZIP compression method: '+method);if(size>65539)throw Error(name+' exceeds the 64 KiB BIN limit');entries.push({name,method,compressed,size,local})}at+=46+nameLen+extraLen+commentLen}if(!entries.length)throw Error(hasNestedZip?'No BIN files found. Nested ZIP archives are not supported; this player can play BIN tape files only.':'No BIN files found in ZIP archive. This player can play BIN tape files only.');return entries}// Распаковывает ZIP-запись и проверяет размер и заголовок BIN до запуска.
async function extractZipEntry(bytes,entry){const view=new DataView(bytes),at=entry.local;if(at+30>bytes.byteLength||view.getUint32(at,true)!==0x04034b50)throw Error('Corrupt local ZIP entry');const start=at+30+view.getUint16(at+26,true)+view.getUint16(at+28,true),end=start+entry.compressed;if(end>bytes.byteLength)throw Error('ZIP data is truncated');let result;if(entry.method===0)result=bytes.slice(start,end);else{if(!('DecompressionStream'in window))throw Error('This browser cannot decompress ZIP Deflate archives');const stream=new Blob([bytes.slice(start,end)]).stream().pipeThrough(new DecompressionStream('deflate-raw'));result=await new Response(stream).arrayBuffer()}if(result.byteLength!==entry.size)throw Error('Unpacked file size does not match the ZIP entry');const b=new Uint8Array(result),declared=b.length>=4?b[2]|(b[3]<<8):0;if(b.length<5||declared===0)throw Error('File does not contain a valid BIN payload');return new Blob([result],{type:'application/octet-stream'})}function cancelZip(){pendingZip=null;document.querySelector('#zipPicker').hidden=true}async function extractSelectedZip(){if(!pendingZip)return;const entry=pendingZip.entries[Number(document.querySelector('#zipMember').value)];if(!entry)return;try{document.querySelector('#extractZip').disabled=true;document.querySelector('#remoteResult').textContent='Extracting '+entry.name+'...';const bin=await extractZipEntry(pendingZip.bytes,entry);cancelZip();await uploadAndPlay(bin,entry.name.split('/').pop())}catch(e){document.querySelector('#remoteResult').textContent='Could not extract BIN: '+e.message}finally{document.querySelector('#extractZip').disabled=false}}async function saveManualCatalogLink(url){const item=catalogLinkData.find(x=>catalogRawUrl(x.url)===url);if(item)return;let name=url.split('?')[0].split('/').pop();try{name=decodeURIComponent(name)}catch(e){}try{const response=await fetch('/catalog-link/add',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({url,name})});if(response.ok)await loadCatalogLinks(url)}catch(e){}}async function playRemote(){const message=document.querySelector('#remoteResult'),inputUrl=unwrapCatalogInput(document.querySelector('#remoteUrl').value),catalogPage=parseCatalogPageUrl(inputUrl);if(catalogPage){await importCatalogPage(catalogPage);return}let url=catalogRawUrl(inputUrl);if(!/^https?:\/\//i.test(url)){message.textContent='Enter a valid HTTP/HTTPS link';return}const path=url.toLowerCase().split(/[?#]/)[0];if(!path.endsWith('.bin')&&!path.endsWith('.zip')){message.textContent='This link is not a page or a BIN/ZIP file';return}remoteBusy=true;pendingZip=null;document.querySelector('#zipPicker').hidden=true;document.querySelector('#remotePlay').disabled=true;message.textContent='Downloading from catalog...';try{const response=await fetch(url);if(!response.ok)throw Error('HTTP '+response.status);const blob=await response.blob();await saveManualCatalogLink(url);if(path.endsWith('.bin')){const item=catalogLinkData[Number(document.querySelector('#catalogFiles').value)],linkedUrl=item?catalogRawUrl(item.url):'';let name=!manualRemoteUrl&&item&&linkedUrl===url&&item.name.toLowerCase().endsWith('.bin')?item.name:decodeURIComponent(url.split('?')[0].split('/').pop());await uploadAndPlay(blob,name)}else{if(blob.size>8*1024*1024)throw Error('ZIP archive exceeds 8 MiB browser limit');const bytes=await blob.arrayBuffer(),entries=parseZip(bytes),select=document.querySelector('#zipMember');if(entries.length===1){const bin=await extractZipEntry(bytes,entries[0]);await uploadAndPlay(bin,entries[0].name.split('/').pop())}else{pendingZip={bytes,entries};select.replaceChildren();entries.forEach((x,i)=>select.add(new Option(x.name,i)));document.querySelector('#zipPicker').hidden=false;message.textContent='Archive contains '+entries.length+' BIN files. Choose one to play.'}}}catch(e){message.textContent='Could not download or extract the file: '+e.message}finally{remoteBusy=false;document.querySelector('#remotePlay').disabled=false}}function cmd(u){fetch(u).then(refresh).catch(refresh)}function fmt(s){return String(Math.floor(s/60)).padStart(2,'0')+':'+String(s%60).padStart(2,'0')}// Проверяет локальный файл и показывает результат его загрузки во flash.
async function uploadBin(){let f=document.querySelector('#bin').files[0];if(!f)return;let m=document.querySelector('#uploadResult');if(!f.name.toLowerCase().endsWith('.bin')){m.textContent='File must have .bin extension';document.querySelector('#bin').value='';return}if(f.size<5||f.size>65539){m.textContent='Invalid BIN file size';document.querySelector('#bin').value='';return}m.textContent='Uploading...';let d=new FormData();d.append('file',f);try{let r=await fetch('/upload',{method:'POST',body:d}),t=await r.text();m.textContent=t;if(r.ok){await loadFiles(f.name);await persistSelections();document.querySelector('#bin').value='';refreshStorage()}}catch(e){m.textContent='BIN upload failed'}}async function refreshStorage(){try{let s=await(await fetch('/api/storage',{cache:'no-store'})).json();document.querySelector('#storage').textContent='Flash storage: '+(s.free/1024).toFixed(0)+' kB free of '+(s.total/1024).toFixed(0)+' kB'}catch(e){document.querySelector('#storage').textContent='Flash storage unavailable'}}let wifiPromptShown=false;function openWifiDialog(){const dialog=document.querySelector('#wifiDialog');if(!dialog.open)dialog.showModal()}function closeWifiDialog(){document.querySelector('#wifiDialog').close()}async function connectWifi(){const message=document.querySelector('#wifiMessage'),ssid=document.querySelector('#wifiSsid').value.trim(),passwordField=document.querySelector('#wifiPassword'),password=passwordField.value;if(!ssid){message.textContent='Enter the network name (SSID).';return}message.textContent='Connecting...';try{const response=await fetch('/wifi',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:new URLSearchParams({ssid,password})});message.textContent=await response.text()}catch(e){message.textContent='Could not send Wi-Fi settings.'}finally{passwordField.value=''}}async function skipWifiSetup(){await fetch('/wifi/skip',{method:'POST'});closeWifiDialog()}// Обновляет статус плеера и SSID; диалог настроек закрывает только пользователь.
async function refresh(){try{let s=await(await fetch('/api/status',{cache:'no-store'})).json();if(menuRevisionSeen>=0 && s.menu_revision!==menuRevisionSeen){await pendingSetting;applySavedControls(await(await fetch('/api/selection')).json());await loadAudioLevel();await loadFiles(file.selectedOptions[0]?.textContent||'')}const warning=document.querySelector('#lengthWarning');
warning.textContent=s.lengthWarning;
warning.hidden=!s.lengthWarning;
document.querySelector('#currentWifi').textContent='Current network: '+(s.current_network||'Not connected');if(s.wifi_connected&&!document.querySelector('#wifiSsid').value)document.querySelector('#wifiSsid').value=s.ssid;activeBinName=s.playing ? s.file : '';
activeRemainingSeconds=s.remaining_seconds;
activeBinPercent=Math.min(100,Math.max(0,Number(s.percent)||0));
updateFileProgress();
status.textContent='Wi-Fi: '+s.wifi+' | '+s.ip+' | Speed: '+Number(s.speed).toFixed(1).replace('.',',')+'×';
document.querySelector('#play').disabled=s.playing;document.querySelector('#stop').disabled=!s.playing;document.querySelector('#bin').disabled=s.playing;document.querySelector('#remotePlay').disabled=s.playing||remoteBusy;document.querySelector('#linksText').disabled=s.playing;document.querySelector('#delete').disabled=s.playing||!file.selectedOptions[0];if(s.wifi_setup_required&&!wifiPromptShown){wifiPromptShown=true;openWifiDialog()}}catch(e){status.textContent='Player is unreachable'}}async function initializePlayer(){let saved={file:'',catalog:''};try{saved=await(await fetch('/api/selection')).json()}catch(e){}if(saved.speed)applySavedControls(saved);await loadFiles(saved.file||'');await loadCatalogLinks(saved.catalog||'');refreshStorage()}initializePlayer();loadAudioLevel();setInterval(refresh,1000);setInterval(refreshStorage,10000);refresh();
</script></html>)HTML";

int menuValue(MenuItem item) {
  switch (item) {
    case MenuItem::Speed: return int(savedSpeed * 2);
    case MenuItem::Level: return audioLevel;
    case MenuItem::Replace: return replaceTapeName;
    case MenuItem::Upper: return upperTapeName;
  }
  return 0;
}

void drawMenu() {
  display.drawStr(5, 10, menuEditing ? "2 Save     4 Exit" : "2 Edit     4 Exit");
  display.drawHLine(4, 13, 120);
  for (int i = 0; i < 4; ++i) {
    const MenuItem item = static_cast<MenuItem>(i);
    const bool selected = item == menuItem;
    const int value = selected && menuEditing ? menuDraft : menuValue(item);
    const int y = 25 + i * 12;
    const char *labels[] = {"Speed", "Level", "Replace name", "UPPERCASE"};
    display.drawStr(4, y, selected ? (menuEditing ? "*" : ">") : " ");
    display.drawStr(12, y, labels[i]);
    String text;
    if (item == MenuItem::Speed) text = String(value * 0.5f, 1) + "x";
    else if (item == MenuItem::Level) text = String(value) + "%";
    else text = value ? "ON" : "OFF";
    display.drawStr(121 - display.getStrWidth(text.c_str()), y, text.c_str());
  }
}

// Обновляет OLED: режим воспроизведения, имя BIN, скорость, сеть/IP и прогресс.
void drawScreen() {
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  display.setFont(u8g2_font_5x8_tf);
  if (menuOpen && !isPlaying) {
    drawMenu();
    display.sendBuffer();
    return;
  }
  char heading[32];
  snprintf(heading, sizeof(heading), "%s  %u/%u", isPlaying ? "PLAY" : "FILE",
           unsigned(localBins.empty() ? 0 : selectedBin + 1), unsigned(localBins.size()));
  display.drawStr(6, 11, heading);
  display.drawHLine(5, 14, 118);
  display.setFont(u8g2_font_6x13_t_cyrillic);
  display.setClipWindow(5, 15, 123, 32);
  display.drawUTF8(6, 28, isPlaying ? currentName : localBins.empty() ? "..." : localBins[selectedBin].name.c_str());
  display.setMaxClipWindow();
  display.setFont(u8g2_font_5x8_tf);
  if (isPlaying) drawProgressLine();
  else {
    char line[32];
    snprintf(line, sizeof(line), "%.1fx   Level %u%%", savedSpeed, unsigned(audioLevel));
    display.drawStr(6, 43, buttonPlayFailed ? "Cannot play file" : line);
  }
  const bool connected = WiFi.status() == WL_CONNECTED;
  String networkLine = connected ? "WIFI " + WiFi.localIP().toString() : wifiApActive ? "AP 192.168.4.1" : "WIFI ...";
  display.drawStr(6, 61, networkLine.c_str());
  display.sendBuffer();
}

// Проверяет заголовок BIN, считает checksum, готовит 20-байтовый заголовок
// ленты и запускает конечный автомат таймера. При ошибке формат не играет.
bool beginPlayback(const char *name, const uint8_t *data, size_t size, bool inFlash, float speed, const char *overrideName, bool uppercase) {
  if (isPlaying || size < 5 || size > MAX_BIN_FILE_SIZE || speed < 1 || speed > 3) return false;
  const uint16_t start = readBinByte(data, inFlash, 0) | (uint16_t(readBinByte(data, inFlash, 1)) << 8);
  const uint16_t headerLength = readBinByte(data, inFlash, 2) | (uint16_t(readBinByte(data, inFlash, 3)) << 8);
  const uint16_t actualLength = size - 4;
  const uint16_t declared = min(headerLength, actualLength);
  if (!declared) return false;
  menuOpen = menuEditing = false;
  lengthWarning = "";
  if (actualLength != headerLength) {
    lengthWarning = String("Предупреждение: в заголовке BIN ") + headerLength +
      " байт, фактически " + actualLength + ". Передаётся " + declared + " байт.";
  }

  activeData = data;
  activeDataInFlash = inFlash;
  playbackBuffer = inFlash ? nullptr : const_cast<uint8_t *>(data);
  playbackSpeed = speed;
  activeUnitUs = max<uint32_t>(1, BASE_UNIT_US / speed);
  payloadLength = declared;
  playedPayloadBits = 0;
  scheduledSignalUs = activeUnitUs;
  audioEdgeCount = 0;
  checksum = 0;
  for (uint16_t i = 0; i < declared; ++i) {
    const uint16_t sum = checksum + readBinByte(data, inFlash, 4 + i);
    checksum = sum + (sum < checksum ? 1 : 0);
  }
  tapeHeader[0] = start & 0xff; tapeHeader[1] = start >> 8;
  tapeHeader[2] = declared & 0xff; tapeHeader[3] = declared >> 8;
  memset(tapeHeader + 4, ' ', MAX_FILENAME);
  // БК хранит имя в однобайтовой KOI-8-совместимой таблице; расширение .BIN
  // в имя на ленте не входит. Конвертируем UTF-8-имя файла в коды БК.
  const char *tapeName = *overrideName ? overrideName : name;
  size_t nameLength = strnlen(tapeName, 256);
  if (!*overrideName && nameLength >= 4 && tapeName[nameLength - 4] == '.' &&
      (tapeName[nameLength - 3] == 'b' || tapeName[nameLength - 3] == 'B') &&
      (tapeName[nameLength - 2] == 'i' || tapeName[nameLength - 2] == 'I') &&
      (tapeName[nameLength - 1] == 'n' || tapeName[nameLength - 1] == 'N')) nameLength -= 4;
  uint8_t encodedName[MAX_FILENAME];
  size_t encodedLength = 0;
  bool nameEncodable = true;
  for (size_t i = 0; i < nameLength && encodedLength < MAX_FILENAME;) {
    const uint8_t first = static_cast<uint8_t>(tapeName[i++]);
    uint32_t codepoint = first;
    if ((first & 0xe0) == 0xc0 && i < nameLength) {
      codepoint = (first & 0x1f) << 6;
      codepoint |= static_cast<uint8_t>(tapeName[i++]) & 0x3f;
    } else if ((first & 0xf0) == 0xe0 && i + 1 < nameLength) {
      codepoint = (first & 0x0f) << 12;
      codepoint |= (static_cast<uint8_t>(tapeName[i++]) & 0x3f) << 6;
      codepoint |= static_cast<uint8_t>(tapeName[i++]) & 0x3f;
    } else if (first >= 0x80) { nameEncodable = false; break; }
    if (uppercase) {
      if ((codepoint >= 'a' && codepoint <= 'z') ||
          (codepoint >= 0x0430 && codepoint <= 0x044f)) codepoint -= 0x20;
      else if (codepoint == 0x0451) codepoint = 0x0401; // ё → Ё
    }
    uint8_t encoded = 0;
    if (codepoint >= 0x20 && codepoint <= 0x7e) encoded = static_cast<uint8_t>(codepoint);
    else {
      switch (codepoint) {
        case 0x0410: encoded=0xe1; break; case 0x0411: encoded=0xe2; break;
        case 0x0412: encoded=0xf7; break; case 0x0413: encoded=0xe7; break;
        case 0x0414: encoded=0xe4; break; case 0x0415: encoded=0xe5; break;
        case 0x0401: encoded=0xa3; break; case 0x0416: encoded=0xf6; break;
        case 0x0417: encoded=0xfa; break; case 0x0418: encoded=0xe9; break;
        case 0x0419: encoded=0xea; break; case 0x041a: encoded=0xeb; break;
        case 0x041b: encoded=0xec; break; case 0x041c: encoded=0xed; break;
        case 0x041d: encoded=0xee; break; case 0x041e: encoded=0xef; break;
        case 0x041f: encoded=0xf0; break; case 0x0420: encoded=0xf2; break;
        case 0x0421: encoded=0xf3; break; case 0x0422: encoded=0xf4; break;
        case 0x0423: encoded=0xf5; break; case 0x0424: encoded=0xe6; break;
        case 0x0425: encoded=0xe8; break; case 0x0426: encoded=0xe3; break;
        case 0x0427: encoded=0xfe; break; case 0x0428: encoded=0xfb; break;
        case 0x0429: encoded=0xfd; break; case 0x042a: encoded=0xff; break;
        case 0x042b: encoded=0xf9; break; case 0x042c: encoded=0xf8; break;
        case 0x042d: encoded=0xfc; break; case 0x042e: encoded=0xe0; break;
        case 0x042f: encoded=0xf1; break;
        case 0x0430: encoded=0xc1; break; case 0x0431: encoded=0xc2; break;
        case 0x0432: encoded=0xd7; break; case 0x0433: encoded=0xc7; break;
        case 0x0434: encoded=0xc4; break; case 0x0435: encoded=0xc5; break;
        case 0x0451: encoded=0xb3; break; case 0x0436: encoded=0xd6; break;
        case 0x0437: encoded=0xda; break; case 0x0438: encoded=0xc9; break;
        case 0x0439: encoded=0xca; break; case 0x043a: encoded=0xcb; break;
        case 0x043b: encoded=0xcc; break; case 0x043c: encoded=0xcd; break;
        case 0x043d: encoded=0xce; break; case 0x043e: encoded=0xcf; break;
        case 0x043f: encoded=0xd0; break; case 0x0440: encoded=0xd2; break;
        case 0x0441: encoded=0xd3; break; case 0x0442: encoded=0xd4; break;
        case 0x0443: encoded=0xd5; break; case 0x0444: encoded=0xc6; break;
        case 0x0445: encoded=0xc8; break; case 0x0446: encoded=0xc3; break;
        case 0x0447: encoded=0xde; break; case 0x0448: encoded=0xdb; break;
        case 0x0449: encoded=0xdd; break; case 0x044a: encoded=0xdf; break;
        case 0x044b: encoded=0xd9; break; case 0x044c: encoded=0xd8; break;
        case 0x044d: encoded=0xdc; break; case 0x044e: encoded=0xc0; break;
        case 0x044f: encoded=0xd1; break;
        default: nameEncodable = false; break;
      }
    }
    if (!nameEncodable) break;
    encodedName[encodedLength++] = encoded;
  }
  if (nameEncodable && encodedLength) memcpy(tapeHeader + 4, encodedName, encodedLength);
  else {
    char shortName[8];
    snprintf(shortName, sizeof(shortName), "BIN%02u", static_cast<unsigned>(data[0]));
    memcpy(tapeHeader + 4, shortName, strlen(shortName));
  }
  totalSignalUs = estimateSignalDurationUs(data, inFlash, declared, tapeHeader, activeUnitUs);
  snprintf(currentName, sizeof(currentName), "%s", name);

  noInterrupts();
  stage = TapeStage::Calibration; pulseCount = 4095; syncIndex = 0; // Первый период запускаем здесь.
  byteIndex = bitIndex = 0; streamPart = 0; secondPulseOfBit = false; stopRequested = false;
  isHighPhase = true; currentPulseUs = activeUnitUs;
  currentHighUs = audioLevel ? (activeUnitUs * audioLevel) / 200 : 1;
  if (!currentHighUs) currentHighUs = 1;
  isPlaying = true;
  playbackEnded = false;
  interrupts();
  if (audioLevel) GPIO.out_w1ts.val = (1UL << AUDIO_PIN);
  else GPIO.out_w1tc.val = (1UL << AUDIO_PIN);
  timerWrite(tapeTimer, 0);
  timerAlarmWrite(tapeTimer, currentHighUs, true);
  timerAlarmEnable(tapeTimer);
  timerStart(tapeTimer);
  drawScreen();
  return true;
}

// Возвращает имя без родительских директорий для показа в интерфейсе.
String basenameOf(String path) {
  int slash = max(path.lastIndexOf('/'), path.lastIndexOf('\\'));
  if (slash >= 0) path = path.substring(slash + 1);
  return path;
}

// Проверяет расширение без учёта регистра, не открывая файл.
bool hasBinExtension(const String &path) {
  String name = basenameOf(path);
  name.toLowerCase();
  return name.endsWith(".bin");
}

// Находит файл по индексу API-списка. Служебные файлы и маркеры .keep
// в список не входят.
bool uploadedPathAt(size_t wanted, String &path, String &name) {
  if (!filesystemReady) return false;
  File root = LittleFS.open("/");
  if (!root) return false;
  size_t current = 0;
  File item = root.openNextFile();
  while (item) {
    String itemPath = item.name();
    if (!item.isDirectory() && hasBinExtension(itemPath)) {
      if (current++ == wanted) {
        path = itemPath.startsWith("/") ? itemPath : "/" + itemPath;
        name = basenameOf(itemPath);
        item.close(); root.close();
        return true;
      }
    }
    item.close();
    item = root.openNextFile();
  }
  root.close();
  return false;
}

// Возвращает количество доступных пользовательских BIN для API и автоочистки.
size_t uploadedBinCount() {
  if (!filesystemReady) return 0;
  File root = LittleFS.open("/");
  if (!root) return 0;
  size_t count = 0;
  File item = root.openNextFile();
  while (item) {
    if (!item.isDirectory() && hasBinExtension(item.name())) ++count;
    item.close(); item = root.openNextFile();
  }
  root.close();
  return count;
}

// Загружает пользовательский BIN из LittleFS в RAM и передаёт буфер
// beginPlayback(); буфер освобождается при завершении или остановке.
bool startSelectedFile(int index, float speed, const char *overrideName = "", bool uppercase = false) {
  if (index < 0 || isPlaying) return false;
  if (index == SYSTEM_BIN_ID)
    return beginPlayback(SYSTEM_BIN_NAME, bk0010_start, systemBinSize(), true, speed, overrideName, uppercase);
  String path, name;
  if (!uploadedPathAt(static_cast<size_t>(index), path, name)) return false;
  File input = LittleFS.open(path, "r");
  if (!input) return false;
  const size_t size = input.size();
  if (size < 5 || size > MAX_BIN_FILE_SIZE) { input.close(); return false; }
  uint8_t *buffer = static_cast<uint8_t *>(malloc(size));
  if (!buffer) { input.close(); return false; }
  const size_t received = input.read(buffer, size);
  input.close();
  if (received != size || !beginPlayback(name.c_str(), buffer, size, false, speed, overrideName, uppercase)) {
    free(buffer);
    return false;
  }
  return true;
}

// Сохраняем исходный индекс LittleFS до сортировки: именно его ожидает
// startSelectedFile(). Обход каталога нужен только после изменений.
void refreshLocalBins() {
  if (!localBinsDirty) return;
  localBins.clear();
  localBins.push_back({SYSTEM_BIN_NAME, SYSTEM_BIN_ID});
  if (filesystemReady) {
    File root = LittleFS.open("/");
    int id = 0;
    for (File item = root.openNextFile(); item; item = root.openNextFile()) {
      if (!item.isDirectory() && hasBinExtension(item.name()))
        localBins.push_back({basenameOf(item.name()), id++});
      item.close();
    }
  }
  std::sort(localBins.begin(), localBins.end(), [](const LocalBin &a, const LocalBin &b) {
    return strcasecmp(a.name.c_str(), b.name.c_str()) < 0;
  });
  selectedBin = 0;
  for (size_t i = 0; i < localBins.size(); ++i)
    if (localBins[i].name == lastSelectedFile) { selectedBin = i; break; }
  localBinsDirty = false;
  buttonPlayFailed = false;
}

void rememberSpeed(float speed) {
  if (speed == savedSpeed) return;
  savedSpeed = speed;
  settings.putUChar("speed_half", uint8_t(speed * 2));
}

// Проверяет параметры HTTP-запроса и начинает передачу выбранного файла.
// Принимаем пять значений скорости, включая целые из ранее открытых страниц.
float requestedPlaybackSpeed(float defaultSpeed) {
  if (!server.hasArg("speed")) return defaultSpeed;
  String value = server.arg("speed");
  value.replace(',', '.');
  for (int halfSteps = 2; halfSteps <= 6; ++halfSteps) {
    const float speed = halfSteps * 0.5f;
    if (value == String(speed, 1) || (!(halfSteps % 2) && value == String(halfSteps / 2))) return speed;
  }
  return 0;
}

void handlePlay() {
  if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
  const int id = server.arg("id").toInt();
  const float speed = requestedPlaybackSpeed(savedSpeed);
  if (speed < 1 || speed > 3) { server.send(400, "text/plain", "speed must be 1.0..3.0 in steps of 0.5"); return; }
  String overrideName = server.hasArg("name") ? server.arg("name") : (replaceTapeName ? replacementName : String());
  overrideName.trim();
  const bool uppercase = server.hasArg("upper") ? server.arg("upper") == "1" : upperTapeName;
  if (!startSelectedFile(id, speed, overrideName.c_str(), uppercase)) { server.send(409, "text/plain", "cannot start playback"); return; }
  rememberSpeed(speed);
  if (lastSelectedFile != currentName) {
    lastSelectedFile = currentName;
    settings.putString("last_file", lastSelectedFile);
    localBinsDirty = true;
  }
  server.send(200, "text/plain", "playing");
}

// Запрашивает немедленную остановку. Освобождение RAM-буфера отложено
// до основного цикла, чтобы не выполнять free() из ISR.
void stopPlaybackImmediately() {
  noInterrupts();
  if (tapeTimer) {
    timerStop(tapeTimer);
    timerAlarmDisable(tapeTimer);
  }
  stage = TapeStage::Idle;
  stopRequested = false;
  isPlaying = false;
  isHighPhase = false;
  GPIO.out_w1tc.val = (1UL << AUDIO_PIN);
  interrupts();
  if (playbackBuffer) { free(playbackBuffer); playbackBuffer = nullptr; }
  activeData = nullptr;
  activeDataInFlash = true;
  drawScreen();
}

// Черновик живёт до подтверждения кнопкой 2. Выход кнопкой 4 отменяет его.
void handleMenuButton(size_t button) {
  if (button == 1) {
    if (menuEditing) {
      switch (menuItem) {
        case MenuItem::Speed: rememberSpeed(menuDraft * 0.5f); break;
        case MenuItem::Level:
          audioLevel = menuDraft;
          settings.putUChar("audio_level", audioLevel);
          break;
        case MenuItem::Replace:
          replaceTapeName = menuDraft;
          settings.putBool("replace_name", replaceTapeName);
          break;
        case MenuItem::Upper:
          upperTapeName = menuDraft;
          settings.putBool("upper_name", upperTapeName);
          break;
      }
      ++menuRevision;
    } else menuDraft = menuValue(menuItem);
    menuEditing = !menuEditing;
    return;
  }
  const int direction = button == 0 ? -1 : 1;
  if (!menuEditing) menuItem = static_cast<MenuItem>((int(menuItem) + direction + 4) % 4);
  else if (menuItem == MenuItem::Speed) menuDraft = constrain(menuDraft + direction, 2, 6);
  else if (menuItem == MenuItem::Level) menuDraft = constrain(menuDraft + direction * 5, 0, 100);
  else menuDraft = direction > 0;
}

// Первое нажатие листает сразу. При удержании повторяем только навигацию.
void pollButtons() {
  for (size_t i = 0; i < 4; ++i) {
    GButton &b = buttons[i];
    b.tick();
    const bool pressed = b.isPress();
    const bool repeat = (i == 0 || i == 2) && (b.isHolded() || b.isStep(b.getHoldClicks()));
    if (!pressed && !repeat) continue;
    if (i == 3) {
      if (!isPlaying) {
        menuOpen = !menuOpen;
        menuEditing = false;
        menuItem = MenuItem::Speed;
        drawScreen();
      }
      continue;
    }
    if (menuOpen) { handleMenuButton(i); drawScreen(); continue; }
    if (i == 1 && isPlaying) { stopPlaybackImmediately(); continue; }
    if (isPlaying || localBins.empty()) continue;
    if (i == 1) {
      buttonPlayFailed = !startSelectedFile(localBins[selectedBin].id, savedSpeed,
                                            replaceTapeName ? replacementName.c_str() : "", upperTapeName);
    } else {
      selectedBin = (selectedBin + (i == 0 ? localBins.size() - 1 : 1)) % localBins.size();
      buttonPlayFailed = false;
      if (lastSelectedFile != localBins[selectedBin].name) {
        lastSelectedFile = localBins[selectedBin].name;
      }
    }
    drawScreen();
  }
  // При прокрутке сохраняем конечный выбор после отпускания, чтобы каждый
  // повтор не приводил к записи во флеш-память.
  if ((buttons[0].isRelease() || buttons[2].isRelease()) &&
      settings.getString("last_file", "") != lastSelectedFile)
    settings.putString("last_file", lastSelectedFile);
}

// Маркер .keep означает, что файл нельзя удалять ради освобождения места.
bool isAutoDeleteProtected(const String &path) {
  return LittleFS.exists(path + ".keep");
}

// Удаляет самый старый незакреплённый BIN при нехватке места; текущая
// цель загрузки исключается из кандидатов на удаление.
bool removeOldestUnprotected(const String &exceptPath) {
  File root = LittleFS.open("/");
  if (!root) return false;
  String oldestPath;
  time_t oldestTime = 0;
  File item = root.openNextFile();
  while (item) {
    String path = item.name();
    if (!item.isDirectory() && hasBinExtension(path) && path != exceptPath &&
        !isAutoDeleteProtected(path)) {
      const time_t changed = item.getLastWrite();
      if (oldestPath.isEmpty() || changed < oldestTime) {
        oldestPath = path.startsWith("/") ? path : "/" + path;
        oldestTime = changed;
      }
    }
    item.close();
    item = root.openNextFile();
  }
  root.close();
  if (oldestPath.isEmpty()) return false;
  if (!LittleFS.remove(oldestPath)) return false;
  localBinsDirty = true;
  LittleFS.remove(oldestPath + ".keep");
  return true;
}

// Удаляет старые незакреплённые BIN, пока не появится место под новую загрузку.
bool makeUploadRoom(size_t required, const String &exceptPath) {
  const size_t reserve = 2048;
  while (LittleFS.totalBytes() - LittleFS.usedBytes() < required + reserve) {
    if (!removeOldestUnprotected(exceptPath)) return false;
  }
  return true;
}

// Потоковая multipart-загрузка BIN: проверка расширения, размера и заголовка,
// запись во временный файл и замена целевого файла только после проверки.
void handleUploadData() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    uploadAccepted = false;
    uploadResultCode = 200;
    uploadResultMessage = "Upload complete";
    uploadBytes = 0;
    if (!filesystemReady) { uploadResultCode = 503; uploadResultMessage = "Flash storage unavailable"; return; }
    if (isPlaying) { uploadResultCode = 409; uploadResultMessage = "Stop playback before uploading"; return; }
    String name = basenameOf(upload.filename);
    if (name.isEmpty() || name.length() > 56 || !hasBinExtension(name)) {
      uploadResultCode = 400; uploadResultMessage = "Choose a .bin file (filename limit: 56 characters)"; return;
    }
    for (size_t i = 0; i < name.length(); ++i) {
      const uint8_t c = static_cast<uint8_t>(name[i]);
      if (c < 0x20 || c == 0x7f) { uploadResultCode = 400; uploadResultMessage = "Invalid filename"; return; }
    }
    if (name.equalsIgnoreCase(SYSTEM_BIN_NAME)) { uploadResultCode = 403; uploadResultMessage = "Reserved system filename"; return; }
    uploadPath = "/" + name;
    if (LittleFS.exists(uploadTempPath)) LittleFS.remove(uploadTempPath);
    uploadFile = LittleFS.open(uploadTempPath, "w");
    if (!uploadFile) { uploadResultCode = 507; uploadResultMessage = "Cannot create upload file"; return; }
    uploadAccepted = true;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!uploadAccepted) return;
    if (uploadBytes + upload.currentSize > MAX_BIN_FILE_SIZE) {
      uploadAccepted = false; uploadResultCode = 413; uploadResultMessage = "BIN exceeds 64 KiB limit";
      uploadFile.close(); LittleFS.remove(uploadTempPath); return;
    }
    if (!makeUploadRoom(upload.currentSize, uploadPath)) { uploadAccepted = false; uploadResultCode = 507; uploadResultMessage = "No space: all old files are protected"; uploadFile.close(); LittleFS.remove(uploadTempPath); return; }
    const size_t written = uploadFile.write(upload.buf, upload.currentSize);
    uploadBytes += written;
    if (written != upload.currentSize) {
      uploadAccepted = false; uploadResultCode = 507; uploadResultMessage = "Flash storage is full";
      uploadFile.close(); LittleFS.remove(uploadTempPath);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!uploadAccepted) return;
    uploadFile.close();
    File verify = LittleFS.open(uploadTempPath, "r");
    uint8_t header[4] = {};
    const size_t actualSize = verify ? verify.size() : 0;
    const bool headerRead = verify && verify.read(header, sizeof(header)) == sizeof(header);
    verify.close();
    const uint16_t declared = uint16_t(header[2]) | (uint16_t(header[3]) << 8);
    if (!headerRead || actualSize < 5 || declared == 0) {
      uploadAccepted = false; uploadResultCode = 422; uploadResultMessage = "Invalid BIN header or truncated payload";
      LittleFS.remove(uploadTempPath); return;
    }
    localBinsDirty = true;
    if (LittleFS.exists(uploadPath)) LittleFS.remove(uploadPath);
    if (!LittleFS.rename(uploadTempPath, uploadPath)) {
      uploadAccepted = false; uploadResultCode = 507; uploadResultMessage = "Could not save BIN to flash";
      LittleFS.remove(uploadTempPath); return;
    }
    uploadAccepted = false;
    uploadResultMessage = "BIN saved to flash";
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    uploadFile.close();
    LittleFS.remove(uploadTempPath);
    uploadAccepted = false; uploadResultCode = 400; uploadResultMessage = "Upload cancelled";
  }
}

// Разрешает только BIN/ZIP из BK-каталога; произвольные хосты из txt
// отклоняются до сохранения или скачивания.
bool allowedCatalogUrl(const String &url) {
  if (url.length() > 1024 ||
      (!url.startsWith("https://") && !url.startsWith("http://"))) return false;
  const int authorityStart = url.indexOf("://") + 3;
  int authorityEnd = url.indexOf('/', authorityStart);
  const int query = url.indexOf('?', authorityStart);
  const int fragment = url.indexOf('#', authorityStart);
  if (authorityEnd < 0) authorityEnd = url.length();
  if (query >= 0 && query < authorityEnd) authorityEnd = query;
  if (fragment >= 0 && fragment < authorityEnd) authorityEnd = fragment;
  if (authorityEnd <= authorityStart) return false;
  const String authority = url.substring(authorityStart, authorityEnd);
  if (authority.indexOf('@') >= 0) return false;
  for (size_t i = 0; i < url.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(url[i]);
    if (c <= 0x20 || c == 0x7f) return false;
  }
  return true;
}

bool hasCatalogBinOrZipExtension(const String &path) {
  String name = basenameOf(path);
  name.toLowerCase();
  return name.endsWith(".bin") || name.endsWith(".zip");
}

// Поддерживает URL;имя, URL без имени и Markdown-ссылки. Имя может быть
// русским и без расширения; тип BIN/ZIP проверяется по расширению URL.
bool parseCatalogLink(String line, String &name, String &url) {
  line.trim();
  if (line.isEmpty() || line.startsWith("#")) return false;
  if (line.startsWith("[") && line.endsWith(")")) {
    const int markdownSeparator = line.indexOf("](");
    if (markdownSeparator > 1) {
      String label = line.substring(1, markdownSeparator);
      String target = line.substring(markdownSeparator + 2, line.length() - 1);
      const int labelSeparator = label.indexOf(';');
      if (labelSeparator >= 0) {
        url = label.substring(0, labelSeparator); url.trim();
        name = label.substring(labelSeparator + 1); name.trim();
      } else {
        url = target; url.trim();
        name = label; name.trim();
      }
    } else url = line;
  } else {
    const int separator = line.indexOf(';');
    if (separator >= 0) {
      url = line.substring(0, separator); url.trim();
      name = line.substring(separator + 1); name.trim();
    } else url = line;
  }
  if (name.isEmpty()) {
    String path = url;
    const int query = path.indexOf('?');
    if (query >= 0) path.remove(query);
    const int slash = path.lastIndexOf('/');
    name = slash >= 0 ? path.substring(slash + 1) : path;
  }
  if (name.isEmpty() || name.length() > 255 || basenameOf(name) != name ||
      !allowedCatalogUrl(url)) return false;
  for (size_t i = 0; i < name.length(); ++i) {
    const uint8_t c = static_cast<uint8_t>(name[i]);
    if (c < 0x20 || c == 0x7f || c == '"' || c == '\\' || c == '|') return false;
  }
  return true;
}

// Предварительно проверяет весь текстовый список и лимит до его сохранения.
bool validateCatalogLinksText(const String &contents) {
  if (contents.length() > MAX_CATALOG_LINKS_TEXT) return false;
  size_t count = 0;
  int start = 0;
  while (start < contents.length()) {
    int end = contents.indexOf('\n', start);
    if (end < 0) end = contents.length();
    String line = contents.substring(start, end);
    line.trim();
    if (!line.isEmpty() && !line.startsWith("#")) {
      String name, url;
      if (!parseCatalogLink(line, name, url) || ++count > MAX_CATALOG_LINKS) return false;
    }
    start = end + 1;
  }
  return true;
}

// Проверяет одну строку загрузки и увеличивает счётчик только для валидной записи.
bool acceptCatalogLinkLine(String line) {
  line.trim();
  if (line.isEmpty() || line.startsWith("#")) return true;
  String name, url;
  if (!parseCatalogLink(line, name, url)) return false;
  if (++linksUploadCount > MAX_CATALOG_LINKS) { linksUploadLimitExceeded = true; return false; }
  return true;
}

// Создаёт стартовый каталог при первом запуске с пустой файловой системой.
void ensureCatalogLinksFile() {
  if (!filesystemReady || LittleFS.exists(CATALOG_LINKS_PATH)) return;
  File seed = LittleFS.open(CATALOG_LINKS_PATH, "w");
  if (seed) { seed.print(INITIAL_CATALOG_LINKS); seed.close(); }
}

// Отдаёт каталог ссылок как JSON; line — физический номер строки исходного
// txt, включая пустые строки и комментарии.
void sendCatalogLinks() {
  if (!filesystemReady) { server.send(503, "application/json", "[]"); return; }
  File input = LittleFS.open(CATALOG_LINKS_PATH, "r");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json; charset=utf-8", "");
  String chunk = "[";
  bool first = true;
  size_t lineNumber = 0;
  if (input) {
    while (input.available()) {
      ++lineNumber;
      String line = input.readStringUntil('\n');
      String name, url;
      if (!parseCatalogLink(line, name, url)) continue;
      String entry = (first ? "" : ",") + String("{\"line\":") + String(lineNumber) + ",\"name\":\"" + jsonEscape(name.c_str()) + "\",\"url\":\"" + jsonEscape(url.c_str()) + "\"}";
      if (chunk.length() + entry.length() > 2048) {
        server.sendContent(chunk);
        chunk = "";
      }
      chunk += entry;
      first = false;
    }
    input.close();
  }
  chunk += ']';
  server.sendContent(chunk);
  server.sendContent(""); // terminate the chunked response
}


// Добавляет вручную скачанный URL в конец файла каталога, если точного URL
// там ещё нет; действует общий лимит записей.
void addCatalogLink() {
  if (!filesystemReady) { server.send(503, "text/plain; charset=utf-8", "storage unavailable"); return; }
  if (!server.hasArg("url") || !server.hasArg("name")) { server.send(400, "text/plain; charset=utf-8", "missing URL or name"); return; }
  String url = server.arg("url"), name = server.arg("name");
  url.trim(); name.trim();
  if (!allowedCatalogUrl(url) || name.isEmpty() ||
      name.length() > 180 || name.indexOf(';') >= 0 || name.indexOf('\n') >= 0 || name.indexOf('\r') >= 0) {
    server.send(422, "text/plain; charset=utf-8", "invalid catalog link"); return;
  }
  File input = LittleFS.open(CATALOG_LINKS_PATH, "r");
  size_t count = 0; bool found = false;
  while (input && input.available()) {
    String line = input.readStringUntil('\n'), oldName, oldUrl;
    if (parseCatalogLink(line, oldName, oldUrl)) { ++count; if (oldUrl == url) found = true; }
  }
  if (input) input.close();
  if (found) { server.send(200, "text/plain; charset=utf-8", "already saved"); return; }
  if (count >= MAX_CATALOG_LINKS) { server.send(413, "text/plain; charset=utf-8", "catalog link limit reached"); return; }
  File output = LittleFS.open(CATALOG_LINKS_PATH, "a");
  if (!output) { server.send(507, "text/plain; charset=utf-8", "cannot open catalog links"); return; }
  File check = LittleFS.open(CATALOG_LINKS_PATH, "r");
  if (check && check.size()) { check.seek(check.size() - 1, SeekSet); if (check.read() != '\n') output.println(); }
  if (!output.println(url + ";" + name)) { output.close(); server.send(507, "text/plain; charset=utf-8", "cannot save catalog link"); return; }
  output.close(); server.send(200, "text/plain; charset=utf-8", "link saved");
}

// Выдаёт нормализованный текст URL;имя с именем загрузки catalog_links.txt.
void sendCatalogLinksText() {
  if (!filesystemReady) { server.send(503, "text/plain", "storage unavailable"); return; }
  ensureCatalogLinksFile();
  File input = LittleFS.open(CATALOG_LINKS_PATH, "r");
  if (!input) { server.send(500, "text/plain", "cannot open links file"); return; }
  server.sendHeader("Content-Disposition", "attachment; filename=\"catalog_links.txt\"");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/plain; charset=utf-8", "");
  while (input.available()) {
    String line = input.readStringUntil('\n');
    String name, url;
    if (parseCatalogLink(line, name, url)) server.sendContent(url + ";" + name + "\n");
  }
  server.sendContent("");
  input.close();
}

// Принимает txt во временный файл, проверяет все строки и лишь затем заменяет
// рабочий каталог, чтобы ошибка не портила ранее сохранённый список.
void handleCatalogLinksUpload() {
  HTTPUpload &upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    linksUploadAccepted = false;
    linksUploadResultCode = 200;
    linksUploadResultMessage = "Links saved";
    linksUploadBytes = linksUploadCount = 0;
    linksUploadLineNumber = 0;
    linksUploadLimitExceeded = false;
    linksUploadLine = "";
    linksUploadLine.reserve(800);
    if (!filesystemReady) { linksUploadResultCode = 503; linksUploadResultMessage = "storage unavailable"; return; }
    if (isPlaying) { linksUploadResultCode = 409; linksUploadResultMessage = "stop playback before uploading links"; return; }
    if (LittleFS.exists(CATALOG_LINKS_TEMP)) LittleFS.remove(CATALOG_LINKS_TEMP);
    linksUploadFile = LittleFS.open(CATALOG_LINKS_TEMP, "w");
    if (!linksUploadFile) { linksUploadResultCode = 507; linksUploadResultMessage = "cannot create temporary links file"; return; }
    linksUploadAccepted = true;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!linksUploadAccepted) return;
    if (linksUploadBytes + upload.currentSize > MAX_CATALOG_LINKS_TEXT) {
      linksUploadAccepted = false; linksUploadResultCode = 413; linksUploadResultMessage = "catalog_links.txt exceeds 1 MiB";
      linksUploadFile.close(); LittleFS.remove(CATALOG_LINKS_TEMP); return;
    }
    if (linksUploadFile.write(upload.buf, upload.currentSize) != upload.currentSize) {
      linksUploadAccepted = false; linksUploadResultCode = 507; linksUploadResultMessage = "flash storage is full";
      linksUploadFile.close(); LittleFS.remove(CATALOG_LINKS_TEMP); return;
    }
    linksUploadBytes += upload.currentSize;
    for (size_t i = 0; i < upload.currentSize; ++i) {
      const char c = static_cast<char>(upload.buf[i]);
      if (c == '\n') {
        ++linksUploadLineNumber;
        if (!acceptCatalogLinkLine(linksUploadLine)) {
          linksUploadAccepted = false; linksUploadResultCode = 422;
          linksUploadResultMessage = linksUploadLimitExceeded ? "Catalog link limit is 3000 entries" : "Invalid catalog link at line " + String(linksUploadLineNumber);
          linksUploadFile.close(); LittleFS.remove(CATALOG_LINKS_TEMP); return;
        }
        linksUploadLine = "";
      } else {
        if (linksUploadLine.length() >= 800) {
          linksUploadAccepted = false; linksUploadResultCode = 422; linksUploadResultMessage = "link line is too long";
          linksUploadFile.close(); LittleFS.remove(CATALOG_LINKS_TEMP); return;
        }
        linksUploadLine += c;
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!linksUploadAccepted) return;
    if (!linksUploadLine.isEmpty()) {
      ++linksUploadLineNumber;
      if (!acceptCatalogLinkLine(linksUploadLine)) {
        linksUploadAccepted = false; linksUploadResultCode = 422;
        linksUploadResultMessage = linksUploadLimitExceeded ? "Catalog link limit is 3000 entries" : "Invalid catalog link at line " + String(linksUploadLineNumber);
        linksUploadFile.close(); LittleFS.remove(CATALOG_LINKS_TEMP); return;
      }
    }
    linksUploadFile.close();
    if (linksUploadCount == 0) {
      linksUploadAccepted = false; linksUploadResultCode = 422; linksUploadResultMessage = "links.txt contains no valid BIN/ZIP links";
      LittleFS.remove(CATALOG_LINKS_TEMP); return;
    }
    if (LittleFS.exists(CATALOG_LINKS_PATH)) LittleFS.remove(CATALOG_LINKS_PATH);
    if (!LittleFS.rename(CATALOG_LINKS_TEMP, CATALOG_LINKS_PATH)) {
      linksUploadAccepted = false; linksUploadResultCode = 507; linksUploadResultMessage = "could not replace links file";
      LittleFS.remove(CATALOG_LINKS_TEMP); return;
    }
    linksUploadAccepted = false;
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    linksUploadFile.close(); LittleFS.remove(CATALOG_LINKS_TEMP);
    linksUploadAccepted = false; linksUploadResultCode = 400; linksUploadResultMessage = "upload cancelled";
  }
}

// Отправляет итоговый HTTP-код после завершения потоковой загрузки каталога.
void finishCatalogLinksUpload() {
  server.send(linksUploadResultCode, "text/plain; charset=utf-8", linksUploadResultMessage);
}

// Оценивает длительность по размеру BIN и выбранной скорости, используя
// среднюю плотность единичных битов. UI помечает результат символом ~.
uint32_t estimateFileDurationSeconds(size_t fileSize, float speed) {
  const uint32_t payloadLength = fileSize > 4 ? static_cast<uint32_t>(fileSize - 4) : 0;
  const uint32_t units = 4792UL + 20UL * (payloadLength + 2UL);
  const uint32_t unitUs = max<uint32_t>(1, BASE_UNIT_US / speed);
  const uint64_t durationUs = static_cast<uint64_t>(units) * unitUs;
  return static_cast<uint32_t>((durationUs + 500000ULL) / 1000000ULL);
}

// Возвращает BIN LittleFS с размером, флагом защиты и оценкой времени
// для выбранной скорости.
void sendFileList() {
  const float requested = requestedPlaybackSpeed(playbackSpeed);
  const float estimateSpeed = requested > 0 ? requested : playbackSpeed;
  String body = "[";
  bool first = true;
  size_t fileId = 0;
  if (filesystemReady) {
    File root = LittleFS.open("/");
    File item = root ? root.openNextFile() : File();
    while (item) {
      const String itemPath = item.name();
      if (!item.isDirectory() && hasBinExtension(itemPath)) {
        const String path = itemPath.startsWith("/") ? itemPath : "/" + itemPath;
        const String name = basenameOf(itemPath);
        if (!first) body += ',';
        const size_t fileSize = item.size();
        body += "{\"id\":" + String(static_cast<unsigned long>(fileId++)) + ",\"name\":\"" + jsonEscape(name.c_str()) +
                "\",\"size\":" + String(static_cast<unsigned long>(fileSize)) +
                ",\"duration_seconds\":" + String(estimateFileDurationSeconds(fileSize, estimateSpeed)) +
                ",\"protected\":" + String(isAutoDeleteProtected(path) ? "true}" : "false}");
        first = false;
      }
      item.close(); item = root.openNextFile();
    }
    root.close();
  }
  if (!first) body += ',';
  body += "{\"system\":true,\"protected\":true,\"id\":" + String(SYSTEM_BIN_ID) +
          ",\"name\":\"" + SYSTEM_BIN_NAME + "\",\"size\":" + String(systemBinSize()) +
          ",\"duration_seconds\":" + String(estimateFileDurationSeconds(systemBinSize(), estimateSpeed)) + "}";
  body += ']';
  server.send(200, "application/json; charset=utf-8", body);
}

// Возвращает браузеру общий, занятый и свободный объём LittleFS в байтах.
void sendStorageInfo() {
  if (!filesystemReady) { server.send(503, "application/json", "{\"available\":false}"); return; }
  const size_t total = LittleFS.totalBytes();
  const size_t used = LittleFS.usedBytes();
  String json = "{\"available\":true,\"total\":" + String(total) +
                ",\"used\":" + String(used) + ",\"free\":" + String(total - used) + "}";
  server.send(200, "application/json", json);
}

// Удаляет пользовательский BIN и его .keep-маркер; встроенные файлы
// не используются и не попадают в этот обработчик.
void handleDelete() {
  if (!server.hasArg("id")) { server.send(400, "text/plain", "missing id"); return; }
  if (server.arg("id").toInt() == SYSTEM_BIN_ID) { server.send(403, "text/plain", SYSTEM_BIN_MESSAGE); return; }
  if (isPlaying) { server.send(409, "text/plain", "stop playback before deleting"); return; }
  const int id = server.arg("id").toInt();
  if (id < 0) { server.send(400, "text/plain", "invalid file id"); return; }
  String path, name;
  if (!uploadedPathAt(static_cast<size_t>(id), path, name)) {
    server.send(404, "text/plain", "file not found"); return;
  }
  if (!LittleFS.remove(path)) { server.send(500, "text/plain", "delete failed"); return; }
  LittleFS.remove(path + ".keep");
  localBinsDirty = true;
  server.send(200, "text/plain", "BIN deleted");
}

// Читает/меняет .keep-маркер; закреплённые файлы пропускаются автоочисткой.
void handleProtect() {
  if (server.arg("id").toInt() == SYSTEM_BIN_ID) {
    if (server.uri() == "/api/protect") server.send(200, "application/json", "{\"protected\":true}");
    else if (server.arg("keep") == "1") server.send(200, "text/plain", "Protected");
    else server.send(403, "text/plain", SYSTEM_BIN_MESSAGE);
    return;
  }
  if (!server.hasArg("id") || isPlaying) { server.send(409, "text/plain", "Cannot change protection now"); return; }
  const int id = server.arg("id").toInt();
  if (id < 0) { server.send(400, "text/plain", "invalid file id"); return; }
  String path, name;
  if (!uploadedPathAt(static_cast<size_t>(id), path, name)) { server.send(404, "text/plain", "File not found"); return; }
  if (server.uri() == "/api/protect") {
    server.send(200, "application/json", String("{\"protected\":") + (isAutoDeleteProtected(path) ? "true}" : "false}"));
    return;
  }
  const String marker = path + ".keep";
  const bool keep = server.hasArg("keep") && server.arg("keep") == "1";
  if (keep) {
    File flag = LittleFS.open(marker, "w");
    if (!flag) { server.send(507, "text/plain", "Cannot save protection flag"); return; }
    flag.close();
  } else if (LittleFS.exists(marker)) LittleFS.remove(marker);
  server.send(200, "text/plain", keep ? "Protected from automatic deletion" : "Automatic deletion enabled");
}

// Отдаёт сохранённые в NVS выборы BIN-файла и ссылки каталога.
void sendSavedSelection() {
  String body = "{\"speed\":" + String(savedSpeed, 1) +
                ",\"replace\":" + (replaceTapeName ? "true" : "false") +
                ",\"upper\":" + (upperTapeName ? "true" : "false") +
                ",\"name\":\"" + jsonEscape(replacementName.c_str()) +
                "\",\"menu_revision\":" + String(menuRevision) +
                ",\"file\":\"" + jsonEscape(lastSelectedFile.c_str()) +
                "\",\"catalog\":\"" + jsonEscape(lastSelectedCatalog.c_str()) + "\"}";
  server.send(200, "application/json; charset=utf-8", body);
}

// Сохраняет выбранный BIN и выбранную ссылку каталога в NVS для восстановления
// интерфейса после перезапуска платы.
void saveSelection() {
  if (server.hasArg("tape_name")) {
    String name = server.arg("tape_name");
    name.trim();
    if (name.length() > 64) { server.send(400, "text/plain", "Tape name too long"); return; }
    for (size_t i = 0; i < name.length(); ++i)
      if (uint8_t(name[i]) < 32) { server.send(400, "text/plain", "Invalid tape name"); return; }
    if (name != replacementName) {
      replacementName = name;
      settings.putString("tape_name", replacementName);
    }
  }
  if (server.hasArg("replace")) {
    replaceTapeName = server.arg("replace") == "1";
    settings.putBool("replace_name", replaceTapeName);
  }
  if (server.hasArg("upper")) {
    upperTapeName = server.arg("upper") == "1";
    settings.putBool("upper_name", upperTapeName);
  }
  if (server.hasArg("speed")) {
    float speed = requestedPlaybackSpeed(savedSpeed);
    if (!speed) { server.send(400, "text/plain", "invalid speed"); return; }
    rememberSpeed(speed);
  }
  if (server.hasArg("file") && server.arg("file") != lastSelectedFile) {
    localBinsDirty = true;
    lastSelectedFile = server.arg("file");
    settings.putString("last_file", lastSelectedFile);
  }
  if (server.hasArg("catalog")) {
    lastSelectedCatalog = server.arg("catalog");
    settings.putString("last_catalog", lastSelectedCatalog);
  }
  server.send(200, "text/plain", "saved");
}

// Сохраняет SSID/пароль в NVS и начинает подключение к роутеру. Точка
// BK_PLAYER остаётся доступной; пароль не попадает в ответы API.
void handleWifiConnect() {
  if (!server.hasArg("ssid") || server.arg("ssid").isEmpty()) {
    server.send(400, "text/plain; charset=utf-8", "SSID is required");
    return;
  }
  wifiSsid = server.arg("ssid");
  wifiPassword = server.arg("password");
  settings.putString("wifi_ssid", wifiSsid);
  settings.putString("wifi_password", wifiPassword);
  settings.putBool("wifi_skip", false);
  wifiSetupRequired = true;
  WiFi.persistent(false);
  WiFi.mode(wifiApActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.disconnect(false, false);
  WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
  wifiConnectPending = true;
  wifiConnectStarted = millis();
  wifiStatus = "Connecting";
  server.send(200, "text/plain; charset=utf-8", "Connecting to the selected Wi-Fi network...");
}

// Запоминает отказ от начальной настройки Wi-Fi; точка BK_PLAYER остаётся активной.
void handleWifiSkip() {
  settings.putBool("wifi_skip", true);
  wifiSetupRequired = false;
  server.send(200, "text/plain; charset=utf-8", "Wi-Fi setup skipped");
}

// Регистрирует веб-страницу и API. DNS wildcard вместе с redirect обеспечивает
// captive portal при подключении клиента к BK_PLAYER.
void setupWeb() {
  server.on("/", HTTP_GET, [] {
    server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    server.sendHeader("Pragma", "no-cache");
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
  });
  server.on("/api/files", HTTP_GET, sendFileList);
  server.on("/api/catalog-links", HTTP_GET, sendCatalogLinks);
  server.on("/catalog_links.txt", HTTP_GET, sendCatalogLinksText);
  server.on("/catalog-link/add", HTTP_POST, addCatalogLink);
  server.on("/catalog-links", HTTP_POST, finishCatalogLinksUpload, handleCatalogLinksUpload);
  server.on("/api/storage", HTTP_GET, sendStorageInfo);
  server.on("/api/audio-level", HTTP_GET, [] { server.send(200, "text/plain", String(audioLevel)); });
  server.on("/audio-level", HTTP_POST, [] {
    if (!server.hasArg("value")) { server.send(400, "text/plain", "missing value"); return; }
    const int value = server.arg("value").toInt();
    if (value < 0 || value > 100) { server.send(400, "text/plain", "value must be 0..100"); return; }
    audioLevel = static_cast<uint8_t>(value);
    settings.putUChar("audio_level", audioLevel);
    server.send(200, "text/plain", String(audioLevel));
  });
  server.on("/api/status", HTTP_GET, sendStatus);
  server.on("/api/selection", HTTP_GET, sendSavedSelection);
  server.on("/selection", HTTP_GET, saveSelection);
  server.on("/wifi", HTTP_POST, handleWifiConnect);
  server.on("/wifi/skip", HTTP_POST, handleWifiSkip);
  server.on("/play", HTTP_GET, handlePlay);
  server.on("/delete", HTTP_GET, handleDelete);
  server.on("/protect", HTTP_GET, handleProtect);
  server.on("/api/protect", HTTP_GET, handleProtect);
  server.on("/stop", HTTP_GET, [] { stopPlaybackImmediately(); server.send(200, "text/plain", "stopped"); });
  server.on("/upload", HTTP_POST,
    [] { server.send(uploadResultCode, "text/plain; charset=utf-8", uploadResultMessage); },
    handleUploadData);
  server.onNotFound([] {
    if (wifiApActive) {
      server.sendHeader("Location", "http://192.168.4.1/", true);
      server.send(302, "text/plain", "");
    } else server.send(404, "text/plain", "not found");
  });
  server.begin();
}

// Инициализирует GPIO, OLED, LittleFS, таймер, Wi-Fi/AP и HTTP-сервер.
void setup() {
  Serial.begin(115200);
  for (GButton &b : buttons) {
    b.setDebounce(30);
    b.setTimeout(500);
    b.setStepTimeout(180);
  }
  pinMode(AUDIO_PIN, OUTPUT);
  digitalWrite(AUDIO_PIN, LOW);
  display.setI2CAddress(0x3C << 1);
  display.begin();
  display.setContrast(180);
  drawScreen();

  filesystemReady = LittleFS.begin(true);
  ensureCatalogLinksFile();
  Serial.printf("BIN storage: %s, %u/%u bytes free\n",
                filesystemReady ? "ready" : "unavailable",
                filesystemReady ? static_cast<unsigned>(LittleFS.totalBytes() - LittleFS.usedBytes()) : 0,
                filesystemReady ? static_cast<unsigned>(LittleFS.totalBytes()) : 0);

  tapeTimer = timerBegin(0, 80, true); // 1 us timer ticks on 80 MHz APB.
  timerAttachInterrupt(tapeTimer, &tapeTimerIsr, true);
  timerAlarmWrite(tapeTimer, BASE_UNIT_US / 2, false);
  timerAlarmDisable(tapeTimer);
  timerStop(tapeTimer);

  settings.begin("bkplayer", false);
  wifiSsid = settings.getString("wifi_ssid", "");
  wifiPassword = settings.getString("wifi_password", "");
  lastSelectedFile = settings.getString("last_file", "");
  lastSelectedCatalog = settings.getString("last_catalog", "");
  audioLevel = settings.getUChar("audio_level", 100);
  const uint8_t halfSteps = settings.getUChar("speed_half", 2);
  savedSpeed = halfSteps >= 2 && halfSteps <= 6 ? halfSteps * 0.5f : 1.0f;
  replacementName = settings.getString("tape_name", "V");
  replaceTapeName = settings.getBool("replace_name", false);
  upperTapeName = settings.getBool("upper_name", false);
  refreshLocalBins();
  const bool wifiSkipped = settings.getBool("wifi_skip", false);
  WiFi.persistent(false);
  esp_wifi_restore();
  WiFi.mode(WIFI_AP_STA);
  wifiApActive = WiFi.softAP("BK_PLAYER");
  if (wifiApActive) dnsServer.start(53, "*", WiFi.softAPIP());
  if (!wifiSsid.isEmpty()) {
    WiFi.begin(wifiSsid.c_str(), wifiPassword.c_str());
    const uint32_t connectStart = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - connectStart < 15000) delay(100);
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiStatus = "Connected";
    wifiSetupRequired = false;
  } else {
    wifiStatus = wifiApActive ? "AP: BK_PLAYER" : "AP start failed";
    wifiSetupRequired = !wifiSkipped;
  }
  setupWeb();
  Serial.printf("BK tape player ready; AP=%s IP=%s STA=%s\n",
                wifiApActive ? "BK_PLAYER" : "off",
                wifiApActive ? WiFi.softAPIP().toString().c_str() : "-",
                WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-");
#ifdef PLAYER_AUTOTEST
  Serial.println("AUTOTEST waiting for COM monitor (up to 8 seconds)");
  const uint32_t waitStartedAt = millis();
  while (!Serial && millis() - waitStartedAt < 8000) delay(10);
  if (uploadedBinCount() > 0 && startSelectedFile(0, 3.0f)) {
    autotestStartedAt = millis();
    Serial.printf("AUTOTEST speed=%.1fx expected duration=%lu ms\\n", playbackSpeed,
                  static_cast<unsigned long>(totalSignalUs / 1000));
  }
#endif
}

// Обслуживает DNS и HTTP, отслеживает отложенное Wi-Fi-подключение и
// освобождает буфер после воспроизведения. Импульсы создаёт ISR таймера.
void loop() {
  if (playbackEnded) {
    noInterrupts();
    timerAlarmDisable(tapeTimer);
    GPIO.out_w1tc.val = (1UL << AUDIO_PIN);
    stage = TapeStage::Idle;
    playbackEnded = false;
    interrupts();
  }
  if (!isPlaying && playbackBuffer) {
    free(playbackBuffer);
    playbackBuffer = nullptr;
    activeData = nullptr;
    activeDataInFlash = true;
  }
  if (wifiApActive) dnsServer.processNextRequest();
  server.handleClient();
  refreshLocalBins();
  pollButtons();
  const wl_status_t wifi = WiFi.status();
  if (wifiConnectPending) {
    if (wifi == WL_CONNECTED) {
      wifiConnectPending = false;
      wifiSetupRequired = false;
      settings.putBool("wifi_skip", false);
      wifiStatus = "Connected";
    } else if (millis() - wifiConnectStarted >= 15000) {
      wifiConnectPending = false;
      wifiSetupRequired = true;
      wifiStatus = "Connection failed";
    }
  } else if (wifi == WL_CONNECTED) wifiStatus = "Connected";
  else if (wifiApActive) wifiStatus = "AP: BK_PLAYER";
  else wifiStatus = "Connecting";
  if (millis() - lastScreenUpdate >= 1000) {
    lastScreenUpdate = millis();
    drawScreen();
  }
#ifdef PLAYER_AUTOTEST
  if (autotestStartedAt && millis() - autotestLastLogAt >= 1000) {
    autotestLastLogAt = millis();
    Serial.printf("AUTOTEST t=%lu ms expected=%lu ms stage=%u sync=%u high=%u pulse=%lu count=%u progress=%u%% bits=%lu/%lu edges=%lu timer=%llu/%llu running=%u alarm=%u playing=%u heap=%u\n",
      static_cast<unsigned long>(millis() - autotestStartedAt),
      static_cast<unsigned long>(totalSignalUs / 1000), static_cast<unsigned>(stage), syncIndex,
      static_cast<unsigned>(isHighPhase), static_cast<unsigned long>(currentPulseUs), pulseCount, playbackPercent(),
      static_cast<unsigned long>(playedPayloadBits), static_cast<unsigned long>(payloadLength) * 8,
      static_cast<unsigned long>(audioEdgeCount),
      static_cast<unsigned long long>(timerRead(tapeTimer)),
      static_cast<unsigned long long>(timerAlarmRead(tapeTimer)), static_cast<unsigned>(timerStarted(tapeTimer)),
      static_cast<unsigned>(timerAlarmEnabled(tapeTimer)), static_cast<unsigned>(isPlaying),
      static_cast<unsigned>(ESP.getFreeHeap()));
    if (!isPlaying && !autotestEndLogged) {
      autotestEndLogged = true;
      Serial.printf("AUTOTEST END actual=%lu ms expected=%lu ms progress=%u%% stage=%u edges=%lu\n",
        static_cast<unsigned long>(millis() - autotestStartedAt),
        static_cast<unsigned long>(totalSignalUs / 1000), playbackPercent(),
        static_cast<unsigned>(stage), static_cast<unsigned long>(audioEdgeCount));
    }
  }
#endif
  delay(2);
}
