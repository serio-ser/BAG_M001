#include <WiFi.h>
#include <PubSubClient.h>
#include "ilife_logo.h"
#include <WiFiManager.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <esp_wifi.h>
#include <esp_task_wdt.h>
#include <deque>

#define RELAY1_PIN 32
#define RS485_TX 17
#define RS485_RX 16
#define RS485_EN 19

const int btnReboot = 36;
const int btnMenu = 39;
const int buzzer = 12;

#define TFT_SDA 23
#define TFT_SCL 18
#define TFT_RES 5
#define TFT_DC  4
#define TFT_CS  25

#define FIRMWARE_VERSION "v2026.09.16-69"
#define SN_Number "SN1024"

enum BirdTransportMode : uint8_t {
  BirdTransportTTL = 0,
  BirdTransportRS485 = 1
};

enum BirdUartPinMode : uint8_t {
  BirdPinsRx16Tx17 = 0,
  BirdPinsRx17Tx16 = 1,
  BirdPinsRx26Tx27 = 2,  // Direct TTL via Female Pin Header (bypass MAX485)
  BirdPinsRx27Tx26 = 3   // Direct TTL swapped
};

static const BirdTransportMode DEFAULT_BIRD_TRANSPORT_MODE = BirdTransportRS485;
static const BirdUartPinMode DEFAULT_BIRD_UART_PIN_MODE = BirdPinsRx16Tx17;
static const uint32_t DEFAULT_BIRD_BAUD_RATE = 115200;
static const uint8_t BIRD_MASTER_ADDRESS = 0x00;      // แอดเดรสของ ESP32 (Master) บนบัส RS485
static const uint8_t BIRD_BOARD_ADDRESS = 0x01;       // แอดเดรสเริ่มต้นของบอร์ด B-BIRD ตัวแรก
static const uint8_t BIRD_HEADER = 0x48;              // ไบต์ header ที่ต้องมีทุกแพ็กเก็ต (0x48 = 'H')
static const uint8_t BIRD_ACK = 0x06;                 // รหัสที่บอร์ดตอบกลับเมื่อ "รับคำสั่งได้"
static const uint8_t BIRD_NAK = 0x0F;                 // รหัสที่บอร์ดตอบกลับเมื่อ "ปฏิเสธคำสั่ง"
static const uint8_t BIRD_EOT = 0x04;                 // ไบต์ปิดท้ายแพ็กเก็ต (End Of Transmission)
static const uint8_t BIRD_COMM_TEST_COMMAND = 0x01;   // คำสั่งทดสอบการเชื่อมต่อ (ping) ไม่เกี่ยวกับการหมุน
static const size_t BIRD_PACKET_SIZE = 40;            // ขนาดแพ็กเก็ตทั้งหมด 40 ไบต์ตามสเปก BIRD
static const size_t BIRD_DATA_SIZE = 32;              // ขนาดส่วนข้อมูลภายในแพ็กเก็ต
static const uint8_t BIRD_MAX_RETRY = 3;              // จำนวนครั้งที่ลองส่งซ้ำถ้าไม่ได้ ACK ก่อนถือว่าล้มเหลว
static const uint32_t BIRD_ACK_TIMEOUT_MS = 600;      // เวลารอ ACK หลังส่งแพ็กเก็ต (600ms) — ไม่ใช่เวลาหมุน
static const uint32_t BIRD_REPLY_TIMEOUT_MS = 5000;   // เวลารอการตอบกลับทั่วไปสูงสุด
static const uint32_t BIRD_DISPENSE_TIMEOUT_MS = 15000; // เวลารอสูงสุดทั้งกระบวนการจ่ายสินค้า 1 ครั้ง

// ★ 2 บรรทัดหลักที่แก้บั๊ก "มอเตอร์หมุนไม่ครบตาม qty": เพิ่มช่วงห่างระหว่างคำสั่งหมุน 2 รอบ
// ให้นานพอที่บอร์ด B-BIRD จะพร้อมรับคำสั่งถัดไป (ทดสอบแล้วใช้งานได้จริงกับ qty=5 และ qty=10)
static const uint32_t DEFAULT_BIRD_SPIN_TIME_MS = 4000; // เวลาที่ให้มอเตอร์ "หมุน" ต่อ 1 รอบ (เดิม 3600 → ใหม่ 4000ms)
static const uint32_t DEFAULT_BIRD_REST_TIME_MS = 3800; // เวลา "พัก" หลังหมุนก่อนหมุนรอบถัดไป (เดิม 1200 → ใหม่ 3800ms)
// รวม Spin+Rest = 7.8 วินาที/รอบ คือช่วงห่างต่ำสุดที่ปลอดภัยระหว่างคำสั่งหมุนแต่ละครั้ง

static const bool DEFAULT_BACKEND_DELAY_ENABLED = false;
static const uint8_t DEFAULT_BACKEND_DELAY_SECONDS = 0;
static const uint8_t MAX_BACKEND_DELAY_SECONDS = 5;
static const uint32_t MIN_BIRD_SPIN_TIME_MS = 1000;   // ค่าต่ำสุดที่ยอมให้ตั้งผ่านหน้าเว็บได้ (กันตั้งเร็วเกินไปจนพัง)
static const uint32_t MAX_BIRD_SPIN_TIME_MS = 15000;  // ค่าสูงสุดที่ยอมให้ตั้งผ่านหน้าเว็บได้
static const uint32_t MIN_BIRD_REST_TIME_MS = 0;      // ค่าต่ำสุดของ Rest Time ที่ตั้งได้
static const uint32_t MAX_BIRD_REST_TIME_MS = 15000;  // ค่าสูงสุดของ Rest Time ที่ตั้งได้ (4000/3800 อยู่ในช่วงนี้)
static const uint32_t QUEUE_DISPLAY_HOLD_MS = 500;    // เวลาค้างแสดงผลคิวบนจอ LCD ต่อ 1 สถานะ ไม่เกี่ยวกับมอเตอร์
static const size_t MAX_QUEUE_SIZE = 20;              // จำนวนงาน BAG สูงสุดที่คิวรับพร้อมกันได้
static const char BAG_JOURNAL_NAMESPACE[] = "bagjournal";
static const uint8_t BAG_JOURNAL_VERSION = 1;
static const char WIFI_SETUP_AP_NAME[] = "iLife-Wifi-Setup"; // ชื่อ WiFi AP ตอนตั้งค่าเครื่องครั้งแรก
static const uint8_t SPIRAL_MOTOR_COUNT = 12;         // จำนวนมอเตอร์ทั้งหมดในเครื่อง (12 ตัว)
static const uint8_t SPIRAL_MOTOR_CODES[SPIRAL_MOTOR_COUNT] = {
  0x00, 0x01,
  0x0A, 0x0B,
  0x14, 0x15,
  0x1E, 0x1F,
  0x28, 0x29,
  0x32, 0x33
};

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RES);
WebServer server(80);
Preferences prefs;

char mqtt_server[40] = "111.223.38.34";
char mqtt_port[6] = "1883";
char mqtt_user[20] = "wuuser";
char mqtt_pass[20] = "wu1234";
char client_id[20] = "BG1002";

WiFiClient espClient;
PubSubClient client(espClient);

struct BirdDispenseResult {
  bool transportOk;
  bool success;
  uint8_t productId;
  uint8_t mode;
  uint8_t resultCode;
  uint16_t useTimeMs;
  String message;
};

struct PendingDispenseJob {
  uint8_t productId;
  uint8_t mode;
  String source;
};

struct PendingBagJob {
  String orderId;
  uint8_t location;
  uint8_t qty;
  String source;
  uint8_t sentQty;
  uint8_t confirmedQty;
  bool allOk;
};

enum BirdControlResult {
  BirdControlTimeout,
  BirdControlAck,
  BirdControlNak,
  BirdControlPacketStart
};

unsigned long lastDisplayUpdate = 0;
unsigned long lastHeartbeat = 0;
unsigned long lastMqttReconnect = 0;
unsigned long lastBirdProbe = 0;
unsigned long menuPressStarted = 0;
unsigned long rebootPressStarted = 0;
unsigned long birdDispenseCooldownUntil = 0;
unsigned long queueDisplayUntil = 0;

bool birdLinkOk = false;
bool dispenseBusy = false;
bool pendingBirdTest = false;
bool pendingWiFiSetup = false;
bool pendingWiFiChange = false;
bool webRoutesConfigured = false;
bool webServerStarted = false;
bool menuButtonEnabled = true;
bool rebootButtonEnabled = true;
BirdTransportMode birdTransportMode = DEFAULT_BIRD_TRANSPORT_MODE;
BirdUartPinMode birdUartPinMode = DEFAULT_BIRD_UART_PIN_MODE;
uint32_t birdBaudRate = DEFAULT_BIRD_BAUD_RATE;
uint32_t birdSpinTimeMs = DEFAULT_BIRD_SPIN_TIME_MS;
uint32_t birdRestTimeMs = DEFAULT_BIRD_REST_TIME_MS;
bool backendDelayEnabled = DEFAULT_BACKEND_DELAY_ENABLED;
uint8_t backendDelaySeconds = DEFAULT_BACKEND_DELAY_SECONDS;
bool birdDeReInvert = false;

String lastBirdMessage = "BOOT";
String pendingOtaUrl = "";
String wifiSetupHint = "";
String birdDebugLog = "";
String pendingWiFiSSID = "";
String pendingWiFiPassword = "";
bool wifiConfigResultPending = false;
String wifiConfigResultStatus = "";
String wifiConfigResultMessage = "";
String wifiConfigResultSSID = "";
static int _otaLastPct = -1;

BirdDispenseResult lastDispense = {false, false, 0, 0, 0, 0, "BOOT"};
std::deque<PendingDispenseJob> dispenseQueue;
std::deque<PendingBagJob> bagQueue;
bool bagRecoveryRequired = false;
bool bagSpinInFlight = false;
String bagRecoveryReason = "";
String lastBagOrderId = "";
uint8_t lastBagOrderLocation = 0;
uint8_t lastBagOrderQty = 0;
bool lastBagOrderOk = false;

int totalDispenseCount = 0;  // cumulative dispense counter (persisted in Preferences)

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: sans-serif; margin: 20px; background: #10131a; color: #ecf3ff; }
    .card { max-width: 480px; margin: 0 auto; padding: 20px; background: #18202b; border-radius: 16px; }
    h2 { margin-top: 0; }
    label { display: block; margin: 12px 0 6px; }
    input, select, button { width: 100%; padding: 12px; font-size: 16px; border-radius: 10px; border: 0; box-sizing: border-box; }
    input, select { background: #edf2f7; color: #111827; }
    button { margin-top: 12px; background: #1d8f6d; color: white; font-weight: 700; }
    button.secondary { background: #3559b8; }
    pre { margin-top: 16px; padding: 12px; background: #0f1723; border-radius: 10px; white-space: pre-wrap; }
    .split { display: flex; gap: 12px; }
    .split button { flex: 1; }
  </style>
</head>
<body>
  <div class="card">
    <h2>B-BIRD Vending Control</h2>
    <label for="id">Product ID / Spiral Motor</label>
    <input type="number" id="id" min="1" max="12" value="1">

    <label for="mode">Dispense Mode</label>
    <select id="mode" onchange="updateProductRange()">
      <option value="0">0 - Solenoid</option>
      <option value="1">1 - Belt</option>
      <option value="2" selected>2 - Spiral Motor</option>
      <option value="3">3 - Solenoid Hard Open</option>
      <option value="4">4 - FMT Mama</option>
      <option value="5">5 - Spiral + Drop Sensor</option>
    </select>

    <label for="addr">Board Address</label>
    <input type="number" id="addr" min="0" max="255" value="1">

    <label for="spinTimeMs">Spin Time (milliseconds)</label>
    <input type="number" id="spinTimeMs" min="1000" max="15000" step="100" value="3600">

    <label for="restTimeMs">Rest Time (milliseconds)</label>
    <input type="number" id="restTimeMs" min="0" max="15000" step="100" value="1200">
    <button class="secondary" onclick="saveMotorTiming()">SAVE MOTOR TIMING</button>

    <button onclick="dispense()">DISPENSE</button>
    <button class="secondary" onclick="wifiSetup()">WIFI SETUP</button>
    <button class="secondary" onclick="window.location='/bag-test'">BAG TEST</button>
    <button class="secondary" onclick="window.location='/motor-delay'">BACKEND MOTOR DELAY</button>

    <pre id="result">ready</pre>
  </div>

  <script>
    async function call(url) {
      const response = await fetch(url);
      const text = await response.text();
      try {
        return JSON.stringify(JSON.parse(text), null, 2);
      } catch (error) {
        return text;
      }
    }

    async function dispense() {
      const id = document.getElementById('id').value;
      const mode = document.getElementById('mode').value;
      const addr = document.getElementById('addr').value;
      document.getElementById('result').textContent = 'working...';
      document.getElementById('result').textContent = await call('/dispense?id=' + encodeURIComponent(id) + '&mode=' + encodeURIComponent(mode) + '&addr=' + encodeURIComponent(addr));
    }

    function updateProductRange() {
      const mode = document.getElementById('mode').value;
      const isSpiral = mode === '2' || mode === '5';
      const input = document.getElementById('id');
      input.min = isSpiral ? '1' : '0';
      input.max = isSpiral ? '12' : '99';
      const value = Number(input.value);
      if (value < Number(input.min) || value > Number(input.max)) input.value = input.min;
    }

    async function saveMotorTiming() {
      const spin = Number(document.getElementById('spinTimeMs').value);
      const rest = Number(document.getElementById('restTimeMs').value);
      if (!Number.isInteger(spin) || !Number.isInteger(rest) || spin < 1000 || spin > 15000 || rest < 0 || rest > 15000) {
        document.getElementById('result').textContent = 'Spin 1000-15000 ms, Rest 0-15000 ms';
        return;
      }
      document.getElementById('result').textContent = await call('/motor-time?spin=' + spin + '&rest=' + rest);
    }

    updateProductRange();
    fetch('/status').then(response => response.json()).then(data => {
      if (data.spinTimeMs !== undefined) document.getElementById('spinTimeMs').value = data.spinTimeMs;
      if (data.restTimeMs !== undefined) document.getElementById('restTimeMs').value = data.restTimeMs;
    });

    async function wifiSetup() {
      document.getElementById('result').textContent = 'switching to wifi setup...';
      document.getElementById('result').textContent = await call('/wifi-setup');
    }

  </script>
</body>
</html>)rawliteral";

const char motor_delay_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Motor Delay</title>
<style>
body{font-family:sans-serif;background:#10131a;color:#ecf3ff;margin:20px}
main{max-width:480px;margin:auto;padding:20px;background:#18202b;border-radius:12px}
h2{margin-top:0}label{display:block;margin:14px 0 6px}
select,button{width:100%;padding:12px;font-size:16px;box-sizing:border-box;border:0;border-radius:8px}
select{background:#edf2f7}button{margin-top:14px;background:#1d8f6d;color:white;font-weight:bold}
#out{margin-top:14px;padding:12px;background:#0f1723;border-radius:8px}
</style>
<main><a href="/">&#8592; Control Panel</a><h2>Backend Motor Delay</h2>
<label><input id="on" type="checkbox"> Enable for backend commands</label>
<label for="sec">Delay before motor starts</label>
<select id="sec"><option value="0">0 seconds</option><option value="1">1 second</option><option value="2">2 seconds</option><option value="3">3 seconds</option><option value="4">4 seconds</option><option value="5">5 seconds</option></select>
<button onclick="save()">SAVE MOTOR DELAY</button><div id="out">Loading...</div></main>
<script>
const on=document.getElementById('on'),sec=document.getElementById('sec'),out=document.getElementById('out');
function sync(){sec.disabled=!on.checked} on.onchange=sync;
async function load(){const d=await (await fetch('/status')).json();on.checked=d.backendDelayEnabled;sec.value=d.backendDelaySeconds||0;sync();out.textContent=on.checked?'Delay '+sec.value+' second(s)':'Delay disabled'}
async function save(){out.textContent='Saving...';const d=await (await fetch('/motor-delay-config?enabled='+(on.checked?1:0)+'&seconds='+sec.value)).json();out.textContent=d.ok?(d.enabled?'Saved: '+d.seconds+' second(s)':'Saved: disabled'):(d.error||'Save failed')}
load();
</script>)rawliteral";

#if 0
const char dash_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Dashboard</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:sans-serif;background:#0f1218;color:#e2e8f0;padding:14px}
    h1{font-size:18px;color:#60a5fa;margin-bottom:14px;text-align:center}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:560px;margin:0 auto}
    .card{background:#1a2130;border-radius:12px;padding:12px}
    .card.full{grid-column:1/-1}
    .card h3{font-size:11px;color:#64748b;text-transform:uppercase;letter-spacing:1px;margin-bottom:7px}
    .val{font-size:20px;font-weight:700}
    .val.sm{font-size:14px;word-break:break-all}
    .badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:13px;font-weight:600}
    .ok{color:#34d399}.err{color:#f87171}.warn{color:#fbbf24}
    .bg-ok{background:#065f46;color:#34d399}.bg-err{background:#7f1d1d;color:#fca5a5}.bg-warn{background:#78350f;color:#fcd34d}
    .row{display:flex;justify-content:space-between;align-items:center;padding:3px 0}
    .lbl{font-size:12px;color:#64748b}
    .mono{font-family:monospace;font-size:12px;color:#93c5fd;background:#0d1622;padding:3px 7px;border-radius:5px;word-break:break-all}
    .ts{font-size:11px;color:#334155;text-align:center;margin-top:10px}
    a.back{display:block;text-align:center;color:#3b82f6;font-size:13px;text-decoration:none;margin-bottom:12px}
    .dot{width:9px;height:9px;border-radius:50%;display:inline-block;margin-right:5px}
    .dot.ok{background:#34d399}.dot.err{background:#f87171}.dot.warn{background:#fbbf24}
  </style>
</head>
<body>
  <a class="back" href="/">&#8592; Control Panel</a>
  <div style="max-width:560px;margin:0 auto">
    <h1 id="h1">&#128225; B-BIRD Dashboard</h1>
    <div class="grid">
      <div class="card">
        <h3>Device ID</h3>
        <div class="val" id="devid">-</div>
      </div>
      <div class="card">
        <h3>Firmware</h3>
        <div class="val sm" id="fw">-</div>
      </div>
      <div class="card">
        <h3>WiFi</h3>
        <span class="badge" id="wifi_b">-</span>
        <div class="lbl" id="ip" style="margin-top:5px;font-size:13px"></div>
      </div>
      <div class="card">
        <h3>MQTT</h3>
        <span class="badge" id="mqtt_b">-</span>
      </div>
      <div class="card">
        <h3>B-BIRD Link</h3>
        <span class="badge" id="bird_b">-</span>
      </div>
      <div class="card">
        <h3>Status</h3>
        <span class="badge" id="busy_b">-</span>
      </div>
      <div class="card full">
        <h3>&#128230; Last Order (BAG)</h3>
        <div class="row"><span class="lbl">Bag ID</span><span id="o_id">-</span></div>
        <div class="row"><span class="lbl">Location</span><span id="o_loc">-</span></div>
        <div class="row"><span class="lbl">QTY</span><span id="o_qty" style="font-size:24px;font-weight:700;color:#60a5fa">-</span></div>
        <div class="row"><span class="lbl">Result</span><span class="badge" id="o_ok">-</span></div>
      </div>
      <div class="card full">
        <h3>&#9202; &#3588;&#3636;&#3623;&#3619;&#3629;&#3604;&#3635;&#3648;&#3609;&#3636;&#3609;&#3585;&#3634;&#3619;</h3>
        <div class="row"><span class="lbl">&#3619;&#3629;&#3651;&#3609;&#3588;&#3636;&#3623;</span><span class="badge" id="q_size">0</span></div>
        <div id="q_empty" style="color:#475569;font-size:12px;padding:6px 0">&#3652;&#3617;&#3656;&#3617;&#3637;&#3588;&#3635;&#3626;&#3633;&#3656;&#3591;&#3651;&#3609;&#3588;&#3636;&#3623;</div>
        <div id="q_list"></div>
      </div>
      <div class="card full">
        <h3>Last Message</h3>
        <div class="val sm ok" id="last">-</div>
      </div>
      <div class="card full">
        <h3>Last Dispense</h3>
        <div class="row"><span class="lbl">Result</span><span class="badge" id="dr">-</span></div>
        <div class="row"><span class="lbl">Product ID</span><span id="di">-</span></div>
        <div class="row"><span class="lbl">Message</span><span id="dm">-</span></div>
        <div class="row"><span class="lbl">Use Time</span><span id="dms">-</span></div>
      </div>
      <div class="card full">
        <h3>UART Config</h3>
        <div class="row"><span class="lbl">Line</span><span id="line">-</span></div>
        <div class="row"><span class="lbl">Pins</span><span id="pins">-</span></div>
        <div class="row"><span class="lbl">Baud</span><span id="baud">-</span></div>
      </div>
      <div class="card full">
        <h3>MQTT Topics</h3>
        <div class="row"><span class="lbl">cmnd</span><span class="mono" id="tc">-</span></div>
        <div style="height:4px"></div>
        <div class="row"><span class="lbl">stat</span><span class="mono" id="ts2">-</span></div>
        <div style="height:4px"></div>
        <div class="row"><span class="lbl">tele</span><span class="mono" id="tt">-</span></div>
      </div>
    </div>
    <div class="ts" id="ts">-</div>
  </div>
  <script>
    function b(text,cls){return'<span class="badge '+cls+'">'+text+'</span>';}
    async function r(){
      try{
        const d=await(await fetch('/dash-status')).json();
        document.getElementById('devid').textContent=d.id||'-';
        document.getElementById('fw').textContent=d.fw||'-';
        document.getElementById('h1').textContent='\uD83D\uDCE1 '+d.id+' Dashboard';
        const wo=d.wifi==='ONLINE';
        document.getElementById('wifi_b').innerHTML=b(d.wifi,wo?'bg-ok':'bg-err');
        document.getElementById('ip').textContent=d.ip||'';
        const mo=d.mqtt==='READY';
        document.getElementById('mqtt_b').innerHTML=b(d.mqtt,mo?'bg-ok':'bg-err');
        const bo=d.bird==='ONLINE';
        document.getElementById('bird_b').innerHTML=b(d.bird,bo?'bg-ok':'bg-warn');
        document.getElementById('busy_b').innerHTML=d.busy?b('BUSY','bg-warn'):b('IDLE','bg-ok');
        document.getElementById('last').textContent=d.last||'-';
        if(d.lastOrderQty>0){
          document.getElementById('o_id').textContent=d.lastOrderId;
          document.getElementById('o_loc').textContent=d.lastOrderLocation;
          document.getElementById('o_qty').textContent=d.lastOrderQty;
          document.getElementById('o_ok').innerHTML=b(d.lastOrderOk?'SUCCESS':'PENDING/FAIL',d.lastOrderOk?'bg-ok':'bg-warn');
        }
        const bq=d.bagQueue||[],dq=d.dispQueue||[];
        const allQ=[...bq.map(x=>({cmd:'BAG#'+x.orderId+'#'+x.location+'#'+x.qty,pos:x.pos})),...dq.map(x=>({cmd:'DISPENSE#'+x.productId+'#'+x.mode,pos:x.pos}))];
        document.getElementById('q_size').innerHTML=b(allQ.length,allQ.length?'bg-warn':'bg-ok');
        document.getElementById('q_empty').style.display=allQ.length?'none':'';
        document.getElementById('q_list').innerHTML=allQ.map(q=>'<div class="row"><span class="lbl" style="color:#fbbf24;min-width:30px">#'+q.pos+'</span><span class="mono">'+q.cmd+'</span></div>').join('');
        document.getElementById('dr').innerHTML=b(d.dispOk?'SUCCESS':'FAIL',d.dispOk?'bg-ok':'bg-err');
        document.getElementById('di').textContent=d.dispId!==undefined?d.dispId:'-';
        document.getElementById('dm').textContent=d.dispMsg||'-';
        document.getElementById('dms').textContent=d.dispMs?d.dispMs+' ms':'-';
        document.getElementById('line').textContent=d.line||'-';
        document.getElementById('pins').textContent=d.pins||'-';
        document.getElementById('baud').textContent=d.baud||'-';
        document.getElementById('tc').textContent=d.topicCmd||'-';
        document.getElementById('ts2').textContent=d.topicStat||'-';
        document.getElementById('tt').textContent=d.topicTele||'-';
        document.getElementById('ts').textContent='updated '+new Date().toLocaleTimeString();
      }catch(e){document.getElementById('ts').textContent='error: '+e.message;}
    }
    r();setInterval(r,2000);
  </script>
</body>
</html>)rawliteral";
#endif

const char bagtest_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>BAG Test</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:sans-serif;background:#0f1218;color:#e2e8f0;padding:16px}
    .card{background:#1a2130;border-radius:12px;padding:16px;max-width:480px;margin:0 auto}
    h2{color:#60a5fa;margin:0 0 16px;font-size:20px}
    label{display:block;font-size:13px;color:#94a3b8;margin:10px 0 4px}
    input[type=number]{width:100%;padding:10px;background:#edf2f7;color:#111;border:0;border-radius:8px;font-size:16px}
    button{width:100%;padding:12px;margin-top:14px;background:#1d8f6d;color:white;border:0;border-radius:10px;font-size:16px;font-weight:700;cursor:pointer}
    button:disabled{background:#374151;color:#6b7280}
    .log{margin-top:16px;background:#0d1622;border-radius:10px;padding:12px;min-height:80px;max-height:320px;overflow-y:auto}
    .ll{font-family:monospace;font-size:13px;padding:3px 0;border-bottom:1px solid #1e2d40;word-break:break-all}
    .ok{color:#34d399}.err{color:#f87171}.info{color:#93c5fd}.warn{color:#fbbf24}
    a.back{display:block;text-align:center;color:#3b82f6;font-size:13px;text-decoration:none;margin-bottom:12px}
    .row{display:flex;gap:12px}
    .row>*{flex:1}
  </style>
</head>
<body>
  <a class="back" href="/">&#8592; Control Panel</a>
  <div class="card">
    <h2>&#129514; BAG Dispense Test</h2>
    <div class="row">
      <div>
        <label>Spiral Motor (1-12)</label>
        <input type="number" id="loc" min="1" max="12" value="1">
      </div>
      <div>
        <label>QTY (1-20)</label>
        <input type="number" id="qty" min="1" max="20" value="2">
      </div>
    </div>
    <label>Bag / Order ID</label>
    <input type="text" id="bagid" maxlength="70" value="" placeholder="e.g. A-002">
    <button id="btn" onclick="runTest()">&#9654; RUN BAG TEST</button>
    <div class="log" id="log"><div class="ll info">Ready &#8212; set values and press RUN</div></div>
  </div>
  <script>
    function addLog(text, cls) {
      const el = document.getElementById('log');
      const d = document.createElement('div');
      d.className = 'll ' + (cls||'');
      d.textContent = text;
      el.appendChild(d);
      el.scrollTop = el.scrollHeight;
    }
    async function runTest() {
      const loc = document.getElementById('loc').value;
      const qty = document.getElementById('qty').value;
      let bagid = document.getElementById('bagid').value.trim();
      if (!bagid) {
        bagid = 'WEBTEST-' + Date.now();
        document.getElementById('bagid').value = bagid;
      }
      const btn = document.getElementById('btn');
      document.getElementById('log').innerHTML = '';
      btn.disabled = true;
      btn.textContent = 'Running...';
      const cmd = 'BAG#' + bagid + '#' + loc + '#' + qty;
      addLog('>> ' + cmd, 'info');
      addLog('qty=' + qty + '  expected spins=' + qty, 'info');
      try {
        const r = await fetch('/bag?location=' + encodeURIComponent(loc) + '&qty=' + encodeURIComponent(qty) + '&bagid=' + encodeURIComponent(bagid));
        const d = await r.json();
        if (d.queued) {
          addLog('QUEUED #' + d.queuePos + ' \u2014 \u0e23\u0e31\u0e1a\u0e04\u0e33\u0e2a\u0e31\u0e48\u0e07\u0e41\u0e25\u0e49\u0e27 \u0e01\u0e33\u0e25\u0e31\u0e07\u0e23\u0e2d\u0e04\u0e34\u0e27', 'warn');
          await watchBag(d.bagId || bagid, Number(d.qty) || Number(qty));
        } else if (d.error) {
          addLog('ERROR: ' + d.error, 'err');
        } else {
          (d.spins || []).forEach(s => {
            const state = s.msg === 'SENT_NO_ACK' ? 'NO_ACK' : (s.ok ? 'OK' : 'FAIL');
            addLog('[BAG] spin ' + s.spin + '/' + d.qty + ' ...', 'info');
            addLog('[BAG] spin ' + s.spin + '/' + d.qty + ' => ' + state + '  code=' + s.code + '  msg=' + s.msg + '  time=' + s.ms + 'ms', state === 'OK' ? 'ok' : 'warn');
          });
          const done = (d.spins || []).length;
          const noAck = (d.spins || []).some(s => s.msg === 'SENT_NO_ACK');
          addLog('Result: ' + done + '/' + d.qty + ' spins ' + (!d.allOk ? 'FAILED' : (noAck ? 'NO_ACK' : 'SUCCESS')), !d.allOk || noAck ? 'warn' : 'ok');
        }
      } catch(e) { addLog('fetch error: ' + e.message, 'err'); }
      btn.disabled = false;
      btn.textContent = '&#9654; RUN BAG TEST';
    }

    async function watchBag(orderId, expectedQty) {
      const deadline = Date.now() + Math.max(60000, expectedQty * 12000);
      let shownSent = -1;
      while (Date.now() < deadline) {
        try {
          const status = await (await fetch('/status')).json();
          if (status.recoveryRequired) {
            addLog('RECOVERY REQUIRED: ' + (status.recoveryReason || 'manual check required'), 'err');
            return;
          }
          if (status.bagOrderId === orderId && status.bagSentQty !== undefined && status.bagSentQty !== shownSent) {
            shownSent = status.bagSentQty;
            addLog('[BAG] sent ' + shownSent + '/' + expectedQty + '  confirmed=' + (status.bagConfirmedQty || 0), 'info');
          }
          if (status.queueSize === 0 && typeof status.last === 'string' && status.last.indexOf(orderId) >= 0) {
            addLog('Result: ' + status.last, status.last.indexOf('SUCCESS') >= 0 ? 'ok' : 'warn');
            return;
          }
        } catch (error) {
          addLog('status error: ' + error.message, 'err');
          return;
        }
        await new Promise(resolve => setTimeout(resolve, 1000));
      }
      addLog('TIMEOUT: ตรวจสอบ /status เพิ่มเติม', 'err');
    }
  </script>
</body>
</html>)rawliteral";

String getValue(const String &data, char separator, int index) {
  int found = 0;
  int start = 0;
  for (int i = 0; i <= data.length(); ++i) {
    if (i == data.length() || data.charAt(i) == separator) {
      if (found == index) {
        return data.substring(start, i);
      }
      ++found;
      start = i + 1;
    }
  }
  return "";
}

bool getSpiralMotorCode(uint8_t motorNumber, uint8_t &productId) {
  if (motorNumber < 1 || motorNumber > SPIRAL_MOTOR_COUNT) {
    return false;
  }

  productId = SPIRAL_MOTOR_CODES[motorNumber - 1];
  return true;
}

bool isSpiralMotorMode(uint8_t mode) {
  return mode == 2 || mode == 5;
}

bool pinReleasedAtBoot(int pin) {
  uint8_t highSamples = 0;
  const uint8_t sampleCount = 20;

  for (uint8_t index = 0; index < sampleCount; ++index) {
    if (digitalRead(pin) == HIGH) {
      ++highSamples;
    }
    delay(2);
  }

  return highSamples >= sampleCount - 1;
}

const char *birdTransportModeLabel() {
  return birdTransportMode == BirdTransportRS485 ? "RS485" : "TTL";
}

const char *birdUartPinModeLabel() {
  if (birdUartPinMode == BirdPinsRx17Tx16) return "RX17/TX16";
  if (birdUartPinMode == BirdPinsRx26Tx27) return "RX26/TX27";
  if (birdUartPinMode == BirdPinsRx27Tx26) return "RX27/TX26";
  return "RX16/TX17";
}

bool parseBirdTransportMode(const String &valueText, BirdTransportMode &mode) {
  String upper = valueText;
  upper.trim();
  upper.toUpperCase();

  if (upper == "TTL" || upper == "DIRECT" || upper == "UART") {
    mode = BirdTransportTTL;
    return true;
  }

  if (upper == "RS485" || upper == "485") {
    mode = BirdTransportRS485;
    return true;
  }

  return false;
}

bool parseBirdUartPinMode(const String &valueText, BirdUartPinMode &mode) {
  String upper = valueText;
  upper.trim();
  upper.toUpperCase();

  if (upper == "RX16TX17" || upper == "16-17" || upper == "NORMAL") {
    mode = BirdPinsRx16Tx17;
    return true;
  }

  if (upper == "RX17TX16" || upper == "17-16" || upper == "SWAP") {
    mode = BirdPinsRx17Tx16;
    return true;
  }

  if (upper == "RX26TX27" || upper == "26-27" || upper == "TTL26") {
    mode = BirdPinsRx26Tx27;
    return true;
  }

  if (upper == "RX27TX26" || upper == "27-26" || upper == "TTL27") {
    mode = BirdPinsRx27Tx26;
    return true;
  }

  return false;
}

bool parseAddressArg(const String &valueText, uint8_t &address) {
  return parseRangeValue(valueText, 0, 255, address);
}

bool parseCommandArg(const String &valueText, uint8_t &command) {
  String text = valueText;
  text.trim();
  if (text.length() == 0) {
    return false;
  }

  int base = 10;
  if (text.startsWith("0x") || text.startsWith("0X")) {
    base = 16;
  }

  char *endPtr = nullptr;
  unsigned long value = strtoul(text.c_str(), &endPtr, base);
  if (endPtr == nullptr || *endPtr != '\0' || value > 0xFF) {
    return false;
  }

  command = static_cast<uint8_t>(value);
  return true;
}

bool parseBirdBaudArg(const String &valueText, uint32_t &baudRate) {
  String text = valueText;
  text.trim();
  if (text.length() == 0) {
    return false;
  }

  char *endPtr = nullptr;
  unsigned long value = strtoul(text.c_str(), &endPtr, 10);
  if (endPtr == nullptr || *endPtr != '\0' || value < 1200 || value > 1000000) {
    return false;
  }

  baudRate = static_cast<uint32_t>(value);
  return true;
}

void birdDebugClear() {
  birdDebugLog = "";
}

void birdDebugAppend(const String &line) {
  if (birdDebugLog.length() > 1800) {
    birdDebugLog.remove(0, birdDebugLog.length() - 1200);
  }

  birdDebugLog += line;
  birdDebugLog += '\n';
}

String birdBytesToHex(const uint8_t *data, size_t length) {
  String text;
  text.reserve(length * 3);
  for (size_t i = 0; i < length; ++i) {
    if (data[i] < 0x10) {
      text += '0';
    }
    text += String(data[i], HEX);
    if (i + 1 < length) {
      text += ' ';
    }
  }
  text.toUpperCase();
  return text;
}

String getSavedWiFiSSID() {
  wifi_config_t wifiConfig;
  memset(&wifiConfig, 0, sizeof(wifiConfig));

  if (esp_wifi_get_config(WIFI_IF_STA, &wifiConfig) != ESP_OK) {
    return "";
  }

  return String(reinterpret_cast<const char *>(wifiConfig.sta.ssid));
}

bool getSavedWiFiCredentials(String &ssid, String &password) {
  wifi_config_t wifiConfig;
  memset(&wifiConfig, 0, sizeof(wifiConfig));

  if (esp_wifi_get_config(WIFI_IF_STA, &wifiConfig) != ESP_OK) {
    ssid = "";
    password = "";
    return false;
  }

  ssid = String(reinterpret_cast<const char *>(wifiConfig.sta.ssid));
  password = String(reinterpret_cast<const char *>(wifiConfig.sta.password));
  return ssid.length() > 0;
}

String getWiFiSSID() {
  if (WiFi.status() == WL_CONNECTED) {
    String currentSSID = WiFi.SSID();
    if (currentSSID.length() > 0) {
      return currentSSID;
    }
  }

  return getSavedWiFiSSID();
}

String jsonEscape(const String &input) {
  String output;
  output.reserve(input.length() + 8);
  for (size_t i = 0; i < input.length(); ++i) {
    char ch = input.charAt(i);
    if (ch == '\\' || ch == '"') {
      output += '\\';
      output += ch;
    } else if (ch == '\n' || ch == '\r') {
      output += ' ';
    } else if (static_cast<uint8_t>(ch) >= 32) {
      output += ch;
    }
  }
  return output;
}

bool parseJsonStringField(const String &json, const char *fieldName, String &value) {
  value = "";
  String key = "\"" + String(fieldName) + "\"";
  int keyStart = json.indexOf(key);

  while (keyStart >= 0) {
    int colon = json.indexOf(':', keyStart + key.length());
    if (colon < 0) {
      return false;
    }

    int valueStart = colon + 1;
    while (valueStart < static_cast<int>(json.length()) &&
           (json.charAt(valueStart) == ' ' || json.charAt(valueStart) == '\t' ||
            json.charAt(valueStart) == '\r' || json.charAt(valueStart) == '\n')) {
      ++valueStart;
    }

    if (valueStart >= static_cast<int>(json.length()) || json.charAt(valueStart) != '"') {
      keyStart = json.indexOf(key, keyStart + key.length());
      continue;
    }

    bool escaped = false;
    for (int index = valueStart + 1; index < static_cast<int>(json.length()); ++index) {
      char ch = json.charAt(index);
      if (escaped) {
        switch (ch) {
          case '"': value += '"'; break;
          case '\\': value += '\\'; break;
          case '/': value += '/'; break;
          case 'b': value += '\b'; break;
          case 'f': value += '\f'; break;
          case 'n': value += '\n'; break;
          case 'r': value += '\r'; break;
          case 't': value += '\t'; break;
          default: return false;
        }
        escaped = false;
      } else if (ch == '\\') {
        escaped = true;
      } else if (ch == '"') {
        return true;
      } else {
        if (static_cast<uint8_t>(ch) < 32) {
          return false;
        }
        value += ch;
      }
    }
    return false;
  }

  return false;
}

void copyStringToBuffer(const String &value, char *buffer, size_t bufferSize) {
  if (bufferSize == 0) {
    return;
  }
  String trimmed = value.substring(0, bufferSize - 1);
  trimmed.toCharArray(buffer, bufferSize);
}

void beep(uint8_t count, uint16_t onMs, uint16_t offMs) {
  for (uint8_t i = 0; i < count; ++i) {
    digitalWrite(buzzer, HIGH);
    delay(onMs);
    digitalWrite(buzzer, LOW);
    if (i + 1 < count) {
      delay(offMs);
    }
  }
}

// Forward declarations สำหรับฟังก์ชันใน display.ino
void printDisplayLine(int x, int y, uint16_t color, const String &text, uint8_t textSize, int maxPad = 20);
void updateDisplay(bool force = false);
void updateQueueDisplay(bool force = false);

size_t totalPendingJobCount() {
  return bagQueue.size() + dispenseQueue.size();
}

void saveConfig(const String &serverValue, const String &portValue, const String &userValue, const String &passValue, const String &deviceIdValue) {
  prefs.begin("config", false);
  prefs.putString("server", serverValue);
  prefs.putString("port", portValue);
  prefs.putString("user", userValue);
  prefs.putString("pass", passValue);
  prefs.putString("devid", deviceIdValue);
  prefs.end();

  copyStringToBuffer(serverValue, mqtt_server, sizeof(mqtt_server));
  copyStringToBuffer(portValue, mqtt_port, sizeof(mqtt_port));
  copyStringToBuffer(userValue, mqtt_user, sizeof(mqtt_user));
  copyStringToBuffer(passValue, mqtt_pass, sizeof(mqtt_pass));
  copyStringToBuffer(deviceIdValue, client_id, sizeof(client_id));
}

void saveBagJournal(bool inFlight) {
  Preferences journal;
  journal.begin(BAG_JOURNAL_NAMESPACE, false);
  journal.putUChar("version", BAG_JOURNAL_VERSION);
  journal.putUChar("count", static_cast<uint8_t>(bagQueue.size()));
  journal.putBool("inflight", inFlight);
  journal.putString("lastId", lastBagOrderId);
  journal.putUChar("lastLoc", lastBagOrderLocation);
  journal.putUChar("lastQty", lastBagOrderQty);
  journal.putBool("lastOk", lastBagOrderOk);

  for (size_t index = 0; index < bagQueue.size(); ++index) {
    const PendingBagJob &job = bagQueue[index];
    String orderKey = "order" + String(index);
    String locationKey = "loc" + String(index);
    String qtyKey = "qty" + String(index);
    String sentKey = "sent" + String(index);
    String confirmedKey = "conf" + String(index);
    String allOkKey = "ok" + String(index);
    journal.putString(orderKey.c_str(), job.orderId);
    journal.putUChar(locationKey.c_str(), job.location);
    journal.putUChar(qtyKey.c_str(), job.qty);
    journal.putUChar(sentKey.c_str(), job.sentQty);
    journal.putUChar(confirmedKey.c_str(), job.confirmedQty);
    journal.putBool(allOkKey.c_str(), job.allOk);
  }
  journal.end();
}

void loadBagJournal() {
  Preferences journal;
  journal.begin(BAG_JOURNAL_NAMESPACE, true);
  uint8_t version = journal.getUChar("version", 0);
  uint8_t savedCount = journal.getUChar("count", 0);
  bool inFlight = journal.getBool("inflight", false);
  lastBagOrderId = journal.getString("lastId", "");
  lastBagOrderLocation = journal.getUChar("lastLoc", 0);
  lastBagOrderQty = journal.getUChar("lastQty", 0);
  lastBagOrderOk = journal.getBool("lastOk", false);

  if (version != BAG_JOURNAL_VERSION) {
    journal.end();
    return;
  }

  if (savedCount > MAX_QUEUE_SIZE) {
    savedCount = MAX_QUEUE_SIZE;
  }

  for (uint8_t index = 0; index < savedCount; ++index) {
    String orderKey = "order" + String(index);
    String locationKey = "loc" + String(index);
    String qtyKey = "qty" + String(index);
    String sentKey = "sent" + String(index);
    String confirmedKey = "conf" + String(index);
    String allOkKey = "ok" + String(index);
    String orderId = journal.getString(orderKey.c_str(), "");
    uint8_t location = journal.getUChar(locationKey.c_str(), 0);
    uint8_t qty = journal.getUChar(qtyKey.c_str(), 0);
    if (orderId.length() == 0 || location < 1 || location > SPIRAL_MOTOR_COUNT || qty < 1 || qty > 20) {
      continue;
    }

    PendingBagJob job;
    job.orderId = orderId;
    job.location = location;
    job.qty = qty;
    job.source = "recovery";
    job.sentQty = journal.getUChar(sentKey.c_str(), 0);
    job.confirmedQty = journal.getUChar(confirmedKey.c_str(), 0);
    job.allOk = journal.getBool(allOkKey.c_str(), true);
    bagQueue.push_back(job);
  }
  journal.end();

  if (!bagQueue.empty() || inFlight) {
    bagRecoveryRequired = true;
    bagSpinInFlight = false;
    bagRecoveryReason = inFlight ? "POWER_LOSS_DURING_SPIN" : "POWER_LOSS_WITH_PENDING_BAG";
    lastBirdMessage = "RECOVERY REQUIRED";
  }
}

void loadConfig() {
  prefs.begin("config", true);
  copyStringToBuffer(prefs.getString("server", mqtt_server), mqtt_server, sizeof(mqtt_server));
  copyStringToBuffer(prefs.getString("port", mqtt_port), mqtt_port, sizeof(mqtt_port));
  copyStringToBuffer(prefs.getString("user", mqtt_user), mqtt_user, sizeof(mqtt_user));
  copyStringToBuffer(prefs.getString("pass", mqtt_pass), mqtt_pass, sizeof(mqtt_pass));
  copyStringToBuffer(prefs.getString("devid", client_id), client_id, sizeof(client_id));
  totalDispenseCount = prefs.getInt("dispTotal", 0);
  uint8_t savedLineMode = prefs.getUChar("lineMode", static_cast<uint8_t>(DEFAULT_BIRD_TRANSPORT_MODE));
  uint8_t savedPinMode = prefs.getUChar("uartPins", static_cast<uint8_t>(DEFAULT_BIRD_UART_PIN_MODE));
  birdBaudRate = prefs.getULong("uartBaud", DEFAULT_BIRD_BAUD_RATE);
  uint32_t legacyMotorCycleMs = prefs.getULong("motorMs", DEFAULT_BIRD_SPIN_TIME_MS);
  uint32_t savedSpinTimeMs = prefs.getULong("spinMs", legacyMotorCycleMs);
  uint32_t savedRestTimeMs = prefs.getULong("restMs", DEFAULT_BIRD_REST_TIME_MS);
  backendDelayEnabled = prefs.getBool("beDelayOn", DEFAULT_BACKEND_DELAY_ENABLED);
  uint8_t savedBackendDelaySeconds = prefs.getUChar("beDelaySec", DEFAULT_BACKEND_DELAY_SECONDS);
  backendDelaySeconds = savedBackendDelaySeconds <= MAX_BACKEND_DELAY_SECONDS
                          ? savedBackendDelaySeconds
                          : DEFAULT_BACKEND_DELAY_SECONDS;
  birdSpinTimeMs = savedSpinTimeMs >= MIN_BIRD_SPIN_TIME_MS && savedSpinTimeMs <= MAX_BIRD_SPIN_TIME_MS
                     ? savedSpinTimeMs
                     : DEFAULT_BIRD_SPIN_TIME_MS;
  birdRestTimeMs = savedRestTimeMs >= MIN_BIRD_REST_TIME_MS && savedRestTimeMs <= MAX_BIRD_REST_TIME_MS
                     ? savedRestTimeMs
                     : DEFAULT_BIRD_REST_TIME_MS;
  birdTransportMode = savedLineMode == static_cast<uint8_t>(BirdTransportTTL) ? BirdTransportTTL : BirdTransportRS485;
  if (savedPinMode == static_cast<uint8_t>(BirdPinsRx17Tx16)) birdUartPinMode = BirdPinsRx17Tx16;
  else if (savedPinMode == static_cast<uint8_t>(BirdPinsRx26Tx27)) birdUartPinMode = BirdPinsRx26Tx27;
  else if (savedPinMode == static_cast<uint8_t>(BirdPinsRx27Tx26)) birdUartPinMode = BirdPinsRx27Tx26;
  else birdUartPinMode = BirdPinsRx16Tx17;
  birdDeReInvert = prefs.getBool("deReInv", false);
  prefs.end();
}

void saveBirdTransportMode() {
  prefs.begin("config", false);
  prefs.putUChar("lineMode", static_cast<uint8_t>(birdTransportMode));
  prefs.end();
}

void saveBirdUartPinMode() {
  prefs.begin("config", false);
  prefs.putUChar("uartPins", static_cast<uint8_t>(birdUartPinMode));
  prefs.end();
}

void saveBirdBaudRate() {
  prefs.begin("config", false);
  prefs.putULong("uartBaud", birdBaudRate);
  prefs.end();
}

void saveBirdMotorTiming() {
  prefs.begin("config", false);
  prefs.putULong("spinMs", birdSpinTimeMs);
  prefs.putULong("restMs", birdRestTimeMs);
  prefs.end();
}

void saveBackendMotorDelay() {
  prefs.begin("config", false);
  prefs.putBool("beDelayOn", backendDelayEnabled);
  prefs.putUChar("beDelaySec", backendDelaySeconds);
  prefs.end();
}

void saveBirdDeReInvert() {
  prefs.begin("config", false);
  prefs.putBool("deReInv", birdDeReInvert);
  prefs.end();
}

void initBirdSerialPort() {
  int rxPin, txPin;
  if (birdUartPinMode == BirdPinsRx26Tx27) {
    rxPin = 26; txPin = 27;
  } else if (birdUartPinMode == BirdPinsRx27Tx26) {
    rxPin = 27; txPin = 26;
  } else if (birdUartPinMode == BirdPinsRx17Tx16) {
    rxPin = RS485_TX; txPin = RS485_RX;
  } else {
    rxPin = RS485_RX; txPin = RS485_TX;
  }
  Serial2.end();
  delay(20);
  Serial2.begin(birdBaudRate, SERIAL_8N1, rxPin, txPin);
  Serial.print("BIRD UART pins: ");
  Serial.println(birdUartPinModeLabel());
  Serial.print("BIRD UART baud: ");
  Serial.println(birdBaudRate);
  birdDebugAppend("UART PINS " + String(birdUartPinModeLabel()));
  birdDebugAppend("UART BAUD " + String(birdBaudRate));
}

void applyBirdTransportMode() {
  if (birdTransportMode == BirdTransportTTL) {
    // TTL mode: disable MAX485 driver AND receiver by setting EN=HIGH (RE#=HIGH → RO hi-z)
    // This prevents MAX485 RO from fighting the TTL signal on D16 when using RX16/TX17 pins.
    digitalWrite(RS485_EN, HIGH);
  } else {
    // RS485 receive idle: EN=LOW (driver off, receiver on), respecting invert flag
    digitalWrite(RS485_EN, birdDeReInvert ? HIGH : LOW);
  }
  Serial.print("BIRD line mode: ");
  Serial.println(birdTransportModeLabel());
  Serial.print("BIRD DE/RE invert: ");
  Serial.println(birdDeReInvert ? "YES" : "NO");
  birdDebugAppend("LINE MODE " + String(birdTransportModeLabel()) + (birdDeReInvert ? " INV" : ""));
}

void setBirdUartPinMode(BirdUartPinMode mode, const String &source) {
  if (birdUartPinMode != mode) {
    birdUartPinMode = mode;
    saveBirdUartPinMode();
  }

  initBirdSerialPort();
  birdFlushInput();
  birdLinkOk = false;
  lastBirdMessage = "Pins " + String(birdUartPinModeLabel()) + " via " + source;
  updateDisplay(true);
  sendStatus();
}

void setBirdBaudRate(uint32_t baudRate, const String &source) {
  if (birdBaudRate != baudRate) {
    birdBaudRate = baudRate;
    saveBirdBaudRate();
  }

  initBirdSerialPort();
  birdFlushInput();
  birdLinkOk = false;
  lastBirdMessage = "Baud " + String(birdBaudRate) + " via " + source;
  updateDisplay(true);
  sendStatus();
}

void setBirdTransportMode(BirdTransportMode mode, const String &source) {
  if (birdTransportMode != mode) {
    birdTransportMode = mode;
    saveBirdTransportMode();
  }

  applyBirdTransportMode();
  birdFlushInput();
  birdLinkOk = false;
  lastBirdMessage = "Line " + String(birdTransportModeLabel()) + " via " + source;
  updateDisplay(true);
  sendStatus();
}

void stopWebServerIfRunning() {
  if (!webServerStarted) {
    return;
  }

  server.stop();
  webServerStarted = false;
}

void startWebServerIfNeeded() {
  if (webServerStarted) {
    return;
  }

  server.begin();
  webServerStarted = true;
}

void setBirdTransmitMode(bool enable) {
  if (birdTransportMode == BirdTransportRS485) {
    // Invert: LOW=transmit/HIGH=receive instead of HIGH=transmit/LOW=receive
    bool pinHigh = birdDeReInvert ? !enable : enable;
    digitalWrite(RS485_EN, pinHigh ? HIGH : LOW);
  }
}

void birdWriteBytes(const uint8_t *data, size_t length, bool discardEcho = false) {
  setBirdTransmitMode(true);
  delayMicroseconds(200);
  Serial2.write(data, length);
  Serial2.flush();
  // Wait for last byte to physically leave the UART shift register before
  // releasing DE/RE.  Each byte = 10 bits at the current baud rate.
  uint32_t usPerByte = (10000000UL + birdBaudRate - 1) / birdBaudRate; // ceil
  delayMicroseconds(usPerByte * 2 + 300);
  setBirdTransmitMode(false);
  if (discardEcho) {
    delayMicroseconds(500);
    while (Serial2.available()) Serial2.read();
  }
}

void birdFlushInput() {
  size_t flushed = 0;
  while (Serial2.available() > 0) {
    Serial2.read();
    ++flushed;
    delay(1);
  }

  if (flushed > 0) {
    birdDebugAppend("FLUSHED " + String(flushed) + " bytes");
  }
}

void birdPrintBytes(const char *prefix, const uint8_t *data, size_t length) {
  Serial.print(prefix);
  for (size_t i = 0; i < length; ++i) {
    if (data[i] < 0x10) {
      Serial.print('0');
    }
    Serial.print(data[i], HEX);
    if (i + 1 < length) {
      Serial.print(' ');
    }
  }
  Serial.println();
  birdDebugAppend(String(prefix) + birdBytesToHex(data, length));
}

uint8_t birdChecksum(const uint8_t *packet) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < BIRD_PACKET_SIZE - 1; ++i) {
    checksum += packet[i];
  }
  return checksum;
}

void buildBirdPacket(uint8_t destinationAddress, uint8_t rw, uint8_t command, const uint8_t *data, uint8_t dataLength, uint8_t *packet) {
  memset(packet, 0, BIRD_PACKET_SIZE);
  packet[0] = BIRD_HEADER;
  packet[1] = BIRD_MASTER_ADDRESS;
  packet[2] = destinationAddress;
  packet[3] = rw;
  packet[4] = command;
  packet[5] = dataLength;
  for (uint8_t i = 0; i < dataLength && i < BIRD_DATA_SIZE; ++i) {
    packet[6 + i] = data[i];
  }
  packet[38] = BIRD_EOT;
  packet[39] = birdChecksum(packet);
}

bool birdReadExact(uint8_t *buffer, size_t length, uint32_t timeoutMs, size_t alreadyHave = 0) {
  size_t offset = alreadyHave;
  unsigned long start = millis();
  while (offset < length && millis() - start < timeoutMs) {
    esp_task_wdt_reset();
    while (Serial2.available() > 0 && offset < length) {
      buffer[offset++] = static_cast<uint8_t>(Serial2.read());
      start = millis();
    }
    delay(1);
  }
  return offset == length;
}

BirdControlResult birdWaitForControl(uint8_t *packet, uint32_t timeoutMs) {
  unsigned long start = millis();
  unsigned long lastMqttPoll = millis();
  while (millis() - start < timeoutMs) {
    esp_task_wdt_reset();
    if (Serial2.available() > 0) {
      uint8_t value = static_cast<uint8_t>(Serial2.read());
      if (value == BIRD_ACK) {
        Serial.println("BIRD RX ACK");
        birdDebugAppend("RX ACK");
        return BirdControlAck;
      }
      if (value == BIRD_NAK) {
        Serial.println("BIRD RX NAK");
        birdDebugAppend("RX NAK");
        return BirdControlNak;
      }
      if (value == BIRD_HEADER) {
        packet[0] = value;
        Serial.println("BIRD RX PACKET WITHOUT ACK");
        birdDebugAppend("RX PACKET WITHOUT ACK");
        return BirdControlPacketStart;
      }
      birdDebugAppend("RX CTRL 0x" + String(value, HEX));
    }
    if (millis() - lastMqttPoll >= 150) {
      lastMqttPoll = millis();
      if (client.connected()) client.loop();
    }
    delay(1);
  }
  Serial.println("BIRD RX TIMEOUT");
  birdDebugAppend("RX TIMEOUT");
  return BirdControlTimeout;
}

bool birdValidatePacket(const uint8_t *packet) {
  if (packet[0] != BIRD_HEADER) {
    return false;
  }
  if (packet[38] != BIRD_EOT) {
    return false;
  }
  return birdChecksum(packet) == packet[39];
}

bool birdReceivePacket(uint8_t *packet, uint32_t timeoutMs, size_t alreadyHave = 0) {
  for (uint8_t attempt = 0; attempt < BIRD_MAX_RETRY; ++attempt) {
    if (!birdReadExact(packet, BIRD_PACKET_SIZE, timeoutMs, alreadyHave)) {
      birdDebugAppend("RX TIMEOUT");
      return false;
    }

    birdPrintBytes("BIRD RX PACKET: ", packet, BIRD_PACKET_SIZE);

    uint8_t control = birdValidatePacket(packet) ? BIRD_ACK : BIRD_NAK;
    birdWriteBytes(&control, 1, true);
    Serial.println(control == BIRD_ACK ? "BIRD TX ACK" : "BIRD TX NAK");
    birdDebugAppend(control == BIRD_ACK ? "TX ACK" : "TX NAK");

    if (control == BIRD_ACK) {
      return true;
    }

    alreadyHave = 0;
  }
  return false;
}

bool birdTransaction(uint8_t destinationAddress, uint8_t rw, uint8_t command, const uint8_t *requestData, uint8_t requestLength, uint8_t *responsePacket, uint32_t responseTimeoutMs, uint32_t ackTimeoutMs = BIRD_ACK_TIMEOUT_MS, uint8_t maxRetry = BIRD_MAX_RETRY) {
  uint8_t packet[BIRD_PACKET_SIZE];
  buildBirdPacket(destinationAddress, rw, command, requestData, requestLength, packet);
  birdDebugClear();
  birdDebugAppend("START cmd=" + String(command) + " rw=" + String(rw) + " sz=40 mode=" + String(birdTransportModeLabel()));

  for (uint8_t attempt = 1; attempt <= maxRetry; ++attempt) {
    birdDebugAppend("ATTEMPT " + String(attempt));
    birdFlushInput();
    birdPrintBytes("BIRD TX PACKET: ", packet, BIRD_PACKET_SIZE);
    birdWriteBytes(packet, BIRD_PACKET_SIZE);

    BirdControlResult control = birdWaitForControl(responsePacket, ackTimeoutMs);
    if (control == BirdControlTimeout || control == BirdControlNak) {
      continue;
    }

    size_t alreadyHave = control == BirdControlPacketStart ? 1 : 0;
    if (birdReceivePacket(responsePacket, responseTimeoutMs, alreadyHave)) {
      return true;
    }
  }

  birdDebugAppend("RESULT FAIL");

  return false;
}

String dispenseResultText(uint8_t resultCode) {
  switch (resultCode) {
    case 1: return "SUCCESS";
    case 2: return "SOLENOID_OPEN_FAIL";
    case 3: return "SOLENOID_OPEN_BEFORE_RUN";
    case 4: return "HOME_POSITION_TIMEOUT";
    case 5: return "DISPENSE_TIMEOUT";
    case 6: return "DROP_SENSOR_ERROR";
    case 7: return "DROP_SENSOR_TIMEOUT";
    default: return "UNKNOWN_RESULT";
  }
}

String buildStatusJson() {
  String payload = "{";
  String wifiSSID = getWiFiSSID();
  payload += "\"id\":\"" + jsonEscape(String(client_id)) + "\"";
  payload += ",\"fw\":\"" + String(FIRMWARE_VERSION) + "\"";
  payload += ",\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE") + "\"";
  payload += ",\"ssid\":\"" + jsonEscape(wifiSSID) + "\"";
  payload += ",\"mqtt\":\"" + String(client.connected() ? "READY" : "WAIT") + "\"";
  payload += ",\"rssi\":" + String(WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : -100);
  payload += ",\"total\":" + String(totalDispenseCount);
  payload += ",\"bird\":\"" + String(birdLinkOk ? "ONLINE" : "WAIT") + "\"";
  payload += ",\"line\":\"" + String(birdTransportModeLabel()) + "\"";
  payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
  payload += ",\"baud\":" + String(birdBaudRate);
  payload += ",\"spinTimeMs\":" + String(birdSpinTimeMs);
  payload += ",\"restTimeMs\":" + String(birdRestTimeMs);
  payload += ",\"backendDelayEnabled\":";
  payload += backendDelayEnabled ? "true" : "false";
  payload += ",\"backendDelaySeconds\":" + String(backendDelaySeconds);
  if (WiFi.status() == WL_CONNECTED) {
    payload += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
  }
  payload += ",\"busy\":";
  payload += dispenseBusy ? "true" : "false";
  payload += ",\"queueSize\":" + String(totalPendingJobCount());
  payload += ",\"bagQueueSize\":" + String(bagQueue.size());
  payload += ",\"recoveryRequired\":";
  payload += bagRecoveryRequired ? "true" : "false";
  payload += ",\"recoveryReason\":\"" + jsonEscape(bagRecoveryReason) + "\"";
  payload += ",\"bagSpinInFlight\":";
  payload += bagSpinInFlight ? "true" : "false";
  if (!bagQueue.empty()) {
    const PendingBagJob &job = bagQueue.front();
    payload += ",\"bagOrderId\":\"" + jsonEscape(job.orderId) + "\"";
    payload += ",\"bagLocation\":" + String(job.location);
    payload += ",\"bagQty\":" + String(job.qty);
    payload += ",\"bagSentQty\":" + String(job.sentQty);
    payload += ",\"bagConfirmedQty\":" + String(job.confirmedQty);
  }
  payload += ",\"last\":\"" + jsonEscape(lastBirdMessage) + "\"";
  payload += "}";
  return payload;
}

String buildDispenseJson(const BirdDispenseResult &result, const String &source) {
  String payload = "{";
  payload += "\"source\":\"" + jsonEscape(source) + "\"";
  payload += ",\"transportOk\":";
  payload += result.transportOk ? "true" : "false";
  payload += ",\"success\":";
  payload += result.success ? "true" : "false";
  payload += ",\"productId\":" + String(result.productId);
  payload += ",\"mode\":" + String(result.mode);
  payload += ",\"resultCode\":" + String(result.resultCode);
  payload += ",\"useTimeMs\":" + String(result.useTimeMs);
  payload += ",\"message\":\"" + jsonEscape(result.message) + "\"";
  payload += "}";
  return payload;
}

bool publishText(const String &topic, const String &payload, bool retain = false) {
  if (!client.connected()) {
    return false;
  }
  return client.publish(topic.c_str(), payload.c_str(), retain);
}

void publishWiFiConfigStatus(const String &status, bool success, const String &message, const String &ssid) {
  String payload = "{";
  payload += "\"event\":\"wifi_config\"";
  payload += ",\"status\":\"" + jsonEscape(status) + "\"";
  payload += ",\"success\":" + String(success ? "true" : "false");
  payload += ",\"device_id\":\"" + jsonEscape(String(client_id)) + "\"";
  payload += ",\"ssid\":\"" + jsonEscape(ssid) + "\"";
  payload += ",\"message\":\"" + jsonEscape(message) + "\"";
  payload += "}";
  publishText("stat/" + String(client_id) + "/STATUS", payload, false);
}

void queueWiFiConfigResult(const String &status, const String &message, const String &ssid) {
  wifiConfigResultPending = true;
  wifiConfigResultStatus = status;
  wifiConfigResultMessage = message;
  wifiConfigResultSSID = ssid;
}

void publishDispenseResult(const BirdDispenseResult &result, const String &source) {
  String eventTopic = "tele/" + String(client_id) + "/BIRD";
  String statTopic = "stat/" + String(client_id);
  String payload = buildDispenseJson(result, source);
  publishText(eventTopic, payload, true);
  publishText(statTopic, "ID " + String(result.productId) + " => " + result.message, false);
}

void sendStatus() {
  String topic = "tele/" + String(client_id) + "/STATE";
  publishText(topic, buildStatusJson(), true);
}

void handleWiFiConfigCommand(const String &commandText) {
  String requestedSSID;
  String requestedPassword;
  if (!parseJsonStringField(commandText, "ssid", requestedSSID) || requestedSSID.length() == 0) {
    publishWiFiConfigStatus("rejected", false, "missing ssid", "");
    return;
  }

  if (!parseJsonStringField(commandText, "pass", requestedPassword)) {
    requestedPassword = "";
  }

  if (requestedSSID.length() > 32 || requestedPassword.length() > 63) {
    publishWiFiConfigStatus("rejected", false, "ssid or password is too long", requestedSSID);
    return;
  }

  if (pendingWiFiChange) {
    publishWiFiConfigStatus("rejected", false, "wifi change already pending", requestedSSID);
    return;
  }

  pendingWiFiSSID = requestedSSID;
  pendingWiFiPassword = requestedPassword;
  pendingWiFiChange = true;
  lastBirdMessage = "WiFi change queued";
  publishWiFiConfigStatus("queued", false, "wifi change queued", requestedSSID);
  updateDisplay(true);
}

bool birdCommunicationTestAtAddress(uint8_t destinationAddress, uint8_t command, String &replyText) {
  uint8_t responsePacket[BIRD_PACKET_SIZE];
  if (!birdTransaction(destinationAddress, 0x00, command, nullptr, 0, responsePacket, 1000)) {
    birdLinkOk = false;
    replyText = "NO_RESPONSE";
    return false;
  }

  uint8_t dataLength = responsePacket[5];
  replyText = "";
  for (uint8_t i = 0; i < dataLength && i < BIRD_DATA_SIZE; ++i) {
    char value = static_cast<char>(responsePacket[6 + i]);
    if (value >= 32 && value <= 126) {
      replyText += value;
    }
  }

  if (replyText.length() == 0) {
    replyText = "OK";
  }

  birdLinkOk = true;
  return true;
}

bool birdCommunicationTestAtAddress(uint8_t destinationAddress, String &replyText) {
  return birdCommunicationTestAtAddress(destinationAddress, BIRD_COMM_TEST_COMMAND, replyText);
}

bool birdCommunicationTest(String &replyText) {
  return birdCommunicationTestAtAddress(BIRD_BOARD_ADDRESS, replyText);
}

bool birdDispenseCooldownActive() {
  if (birdDispenseCooldownUntil == 0) {
    return false;
  }

  if (static_cast<int32_t>(birdDispenseCooldownUntil - millis()) <= 0) {
    birdDispenseCooldownUntil = 0;
    return false;
  }

  return true;
}

void waitForBirdDispenseCooldown() {
  if (!birdDispenseCooldownActive()) {
    return;
  }

  Serial.println("[DISPENSE] waiting for motor cooldown");
  while (birdDispenseCooldownActive()) {
    if (client.connected()) {
      client.loop();
    }
    esp_task_wdt_reset();
    delay(10);
  }
}

void waitForBackendMotorDelay(const String &source) {
  if (source != "mqtt" || !backendDelayEnabled || backendDelaySeconds == 0) {
    return;
  }

  dispenseBusy = true;
  lastBirdMessage = "MQTT delay " + String(backendDelaySeconds) + "s";
  updateDisplay(true);
  sendStatus();

  uint32_t delayMs = static_cast<uint32_t>(backendDelaySeconds) * 1000UL;
  unsigned long startedAt = millis();
  while (millis() - startedAt < delayMs) {
    if (client.connected()) {
      client.loop();
    }
    esp_task_wdt_reset();
    delay(10);
  }
}

BirdDispenseResult executeDispenseAtAddress(uint8_t destinationAddress, uint8_t productId, uint8_t mode, const String &source) {
  BirdDispenseResult result = {false, false, productId, mode, 0, 0, "NO_RESPONSE"};
  uint8_t requestData[2] = {productId, mode};
  uint8_t responsePacket[BIRD_PACKET_SIZE];

  waitForBirdDispenseCooldown();
  dispenseBusy = true;
  lastBirdMessage = "Running ID " + String(productId) + " mode " + String(mode);
  updateDisplay();
  uint32_t spinStartedAt = millis();
  uint32_t spinReadyAt = spinStartedAt + birdSpinTimeMs;

  // Send once only (maxRetry=1) — prevent duplicate motor spins on retry.
  // Allow the board to finish a short motor cycle before its ACK/response is expected.
  uint32_t dispenseAckTimeoutMs = birdSpinTimeMs > BIRD_REPLY_TIMEOUT_MS
                                    ? birdSpinTimeMs
                                    : BIRD_REPLY_TIMEOUT_MS;
  bool transportOk = birdTransaction(destinationAddress, 0x01, 0x0E, requestData, 2, responsePacket, BIRD_DISPENSE_TIMEOUT_MS, dispenseAckTimeoutMs, 1);
  result.transportOk = transportOk;

  if (transportOk) {
    uint8_t dataLength = responsePacket[5];
    if (dataLength >= 3) {
      result.productId = responsePacket[6];
      result.mode = responsePacket[7];
      result.resultCode = responsePacket[8];
      if (dataLength >= 5) {
        result.useTimeMs = (static_cast<uint16_t>(responsePacket[9]) << 8) | responsePacket[10];
      }
      result.message = dispenseResultText(result.resultCode);
      result.success = result.resultCode == 1;
      birdLinkOk = true;
    } else {
      result.message = "SHORT_RESPONSE";
      birdLinkOk = false;
    }
  } else {
    result.success = false;
    result.message = "SENT_NO_ACK";
    birdLinkOk = false;
    Serial.println("[DISPENSE] no ACK — command sent, result unconfirmed");
  }

  uint32_t restStartedAt = millis();
  if (static_cast<int32_t>(spinReadyAt - restStartedAt) > 0) {
    restStartedAt = spinReadyAt;
  }
  birdDispenseCooldownUntil = restStartedAt + birdRestTimeMs;

  lastDispense = result;
  lastBirdMessage = "ID " + String(result.productId) + ": " + result.message;
  updateDisplay();
  publishDispenseResult(result, source);
  sendStatus();

  if (result.success) {
    totalDispenseCount++;
    prefs.begin("config", false);
    prefs.putInt("dispTotal", totalDispenseCount);
    prefs.end();
    beep(1, 80, 50);
  } else {
    beep(2, 60, 60);
  }

  dispenseBusy = false;
  return result;
}

BirdDispenseResult executeDispense(uint8_t productId, uint8_t mode, const String &source) {
  return executeDispenseAtAddress(BIRD_BOARD_ADDRESS, productId, mode, source);
}

String buildBagSpinJson(const PendingBagJob &job, uint8_t spinNumber, const BirdDispenseResult &result) {
  String bagCmd = "BAG#" + job.orderId + "#" + String(job.location) + "#" + String(job.qty);
  String status = result.success ? "spin_success" : "spin_failed";
  if (!result.success && result.message == "SENT_NO_ACK") {
    status = "unconfirmed";
  } else if (!result.success && result.message == "SHORT_RESPONSE") {
    status = "recovery_required";
  }

  String payload = "{";
  payload += "\"event\":\"bag_spin\"";
  payload += ",\"status\":\"" + status + "\"";
  payload += ",\"message\":\"" + String(result.success ? "success" : "fail") + "\"";
  payload += ",\"success\":" + String(result.success ? "true" : "false");
  payload += ",\"accepted\":true";
  payload += ",\"processing\":true";
  payload += ",\"completed\":false";
  payload += ",\"device_id\":\"" + jsonEscape(String(client_id)) + "\"";
  payload += ",\"orderid\":\"" + jsonEscape(job.orderId) + "\"";
  payload += ",\"command\":\"" + jsonEscape(bagCmd) + "\"";
  payload += ",\"location\":" + String(job.location);
  payload += ",\"qty\":" + String(job.qty);
  payload += ",\"spin\":" + String(spinNumber);
  payload += ",\"sent_qty\":" + String(job.sentQty);
  payload += ",\"confirmed_qty\":" + String(job.confirmedQty);
  payload += ",\"transportOk\":" + String(result.transportOk ? "true" : "false");
  payload += ",\"resultCode\":" + String(result.resultCode);
  payload += ",\"useTimeMs\":" + String(result.useTimeMs);
  payload += ",\"result\":\"" + jsonEscape(result.message) + "\"";
  payload += "}";
  return payload;
}

String buildBagCompletionJson(const PendingBagJob &job, bool allOk) {
  String bagCmd = "BAG#" + job.orderId + "#" + String(job.location) + "#" + String(job.qty);
  String payload = "{";
  payload += "\"event\":\"bag_complete\"";
  payload += ",\"status\":\"completed\"";
  payload += ",\"message\":\"" + String(allOk ? "success" : "fail") + "\"";
  payload += ",\"success\":" + String(allOk ? "true" : "false");
  payload += ",\"accepted\":true";
  payload += ",\"processing\":false";
  payload += ",\"completed\":true";
  payload += ",\"device_id\":\"" + jsonEscape(String(client_id)) + "\"";
  payload += ",\"orderid\":\"" + jsonEscape(job.orderId) + "\"";
  payload += ",\"total_items\":1";
  payload += ",\"summary\":[{\"productcode\":\"" + jsonEscape(job.orderId) + "\",\"liter\":0,\"qty\":" + String(job.qty) + "}]";
  payload += ",\"commands_sent\":[{\"location\":\"" + String(job.location) + "\"";
  payload += ",\"productcode\":\"" + jsonEscape(job.orderId) + "\"";
  payload += ",\"qty\":" + String(job.qty);
  payload += ",\"command\":\"" + jsonEscape(bagCmd) + "\"";
  payload += ",\"success\":" + String(allOk ? "true" : "false");
  payload += ",\"sent_qty\":" + String(job.sentQty);
  payload += ",\"confirmed_qty\":" + String(job.confirmedQty);
  payload += ",\"stock_before\":0,\"stock_after\":0}]}";
  return payload;
}

void publishBagSpinResult(const PendingBagJob &job, uint8_t spinNumber, const BirdDispenseResult &result) {
  String payload = buildBagSpinJson(job, spinNumber, result);
  publishText("stat/" + String(client_id), payload, false);
  publishText("tele/" + String(client_id) + "/BAG", payload, false);
}

void publishBagCompletion(const PendingBagJob &job, bool allOk) {
  String payload = buildBagCompletionJson(job, allOk);
  publishText("stat/" + String(client_id), payload, false);
  publishText("tele/" + String(client_id) + "/BAG", payload, false);
}

bool executeBagDispenseSpin(PendingBagJob &job) {
  uint8_t productId = 0;
  if (!getSpiralMotorCode(job.location, productId)) {
    job.allOk = false;
    return false;
  }

  uint8_t spinNumber = job.sentQty + 1;
  String bagCmd = "BAG#" + job.orderId + "#" + String(job.location) + "#" + String(job.qty);
  Serial.printf("[BAG] spin %d/%d ...\n", spinNumber, job.qty);
  esp_task_wdt_reset();
  if (job.sentQty == 0) {
    waitForBackendMotorDelay(job.source);
  }
  bagSpinInFlight = true;
  saveBagJournal(true);
  BirdDispenseResult result = executeDispenseAtAddress(BIRD_BOARD_ADDRESS, productId, 2, job.source);
  bagSpinInFlight = false;
  job.sentQty = spinNumber;
  if (result.success) {
    job.confirmedQty++;
  } else {
    job.allOk = false;
    publishText("stat/" + String(client_id), bagCmd + " UNCONFIRMED spin" + String(spinNumber) + " " + result.message, false);
  }

  Serial.printf("[BAG] spin %d/%d => %s code=%d ms=%d sent=%d confirmed=%d\n",
                spinNumber, job.qty,
                result.success ? "OK" : result.message.c_str(),
                result.resultCode, result.useTimeMs,
                job.sentQty, job.confirmedQty);

  saveBagJournal(false);
  publishBagSpinResult(job, spinNumber, result);

  if (!result.success) {
    if (result.message == "SENT_NO_ACK") {
      return true;
    }
    if (result.message == "SHORT_RESPONSE") {
      bagRecoveryRequired = true;
      bagRecoveryReason = result.message;
    }
    return false;
  }

  return true;
}

void completeBagDispense(const PendingBagJob &job) {
  bool allOk = job.allOk && job.sentQty == job.qty && job.confirmedQty == job.qty;
  bool allSent = job.sentQty == job.qty;
  lastBagOrderId = job.orderId;
  lastBagOrderLocation = job.location;
  lastBagOrderQty = job.qty;
  lastBagOrderOk = allOk;
  lastBirdMessage = allOk ? "BAG " + job.orderId + " SUCCESS"
                          : (allSent ? "BAG " + job.orderId + " UNCONFIRMED" : "BAG " + job.orderId + " FAIL");
  publishBagCompletion(job, allOk);
  saveBagJournal(false);

  sendStatus();
}

bool parseRangeValue(const String &valueText, uint8_t minValue, uint8_t maxValue, uint8_t &outValue) {
  if (valueText.length() == 0) {
    return false;
  }

  long value = valueText.toInt();
  if (value < minValue || value > maxValue) {
    return false;
  }

  outValue = static_cast<uint8_t>(value);
  return true;
}

bool parseDispenseCommand(const String &commandText, uint8_t &productId, uint8_t &mode) {
  String trimmed = commandText;
  trimmed.trim();
  String upper = trimmed;
  upper.toUpperCase();

  if (!(upper.startsWith("DISPENSE#") || upper.startsWith("VEND#"))) {
    return false;
  }

  if (!parseRangeValue(getValue(trimmed, '#', 2), 0, 5, mode)) {
    return false;
  }

  uint8_t requestedId = 0;
  if (isSpiralMotorMode(mode)) {
    if (!parseRangeValue(getValue(trimmed, '#', 1), 1, SPIRAL_MOTOR_COUNT, requestedId)) {
      return false;
    }
    return getSpiralMotorCode(requestedId, productId);
  }

  return parseRangeValue(getValue(trimmed, '#', 1), 0, 99, productId);
}

bool parseBagCommand(const String &commandText, String &orderId, uint8_t &location, uint8_t &qty) {
  String trimmed = commandText;
  trimmed.trim();
  String upper = trimmed;
  upper.toUpperCase();

  if (!upper.startsWith("BAG#")) {
    return false;
  }

  String part1 = getValue(trimmed, '#', 1);
  String part2 = getValue(trimmed, '#', 2);
  String part3 = getValue(trimmed, '#', 3);

  if (part1.length() == 0 || part2.length() == 0 || part3.length() == 0) {
    return false;
  }

  long v2 = part2.toInt();
  long v3 = part3.toInt();

  if (part1.length() > 70) return false;
  if (v2 < 1 || v2 > SPIRAL_MOTOR_COUNT) return false;  // location 1-based
  if (v3 < 1 || v3 > 20)  return false;

  orderId  = part1;
  location = static_cast<uint8_t>(v2);
  qty      = static_cast<uint8_t>(v3);
  return true;
}

bool bagOrderAlreadyTracked(const String &orderId, uint8_t location) {
  if (orderId.length() == 0) {
    return false;
  }
  if (lastBagOrderId == orderId && lastBagOrderLocation == location && lastBagOrderQty > 0) {
    return true;
  }
  for (const PendingBagJob &job : bagQueue) {
    if (job.orderId == orderId && job.location == location) {
      return true;
    }
  }
  return false;
}

void queueDispense(uint8_t productId, uint8_t mode, const String &source) {
  PendingDispenseJob job;
  job.productId = productId;
  job.mode = mode;
  job.source = source;
  dispenseQueue.push_back(job);
  lastBirdMessage = "Queued ID " + String(productId) + " #" + String(dispenseQueue.size());
  queueDisplayUntil = millis() + QUEUE_DISPLAY_HOLD_MS;
  updateQueueDisplay(true);
}

void queueBag(const String &orderId, uint8_t location, uint8_t qty, const String &source) {
  PendingBagJob job;
  job.orderId    = orderId;
  job.location   = location;
  job.qty        = qty;
  job.source     = source;
  job.sentQty = 0;
  job.confirmedQty = 0;
  job.allOk = true;
  bagQueue.push_back(job);
  saveBagJournal(bagSpinInFlight);
  lastBirdMessage = "Queued BAG#" + orderId + "#" + String(location) + "#" + String(qty) + " #" + String(bagQueue.size());
  Serial.printf("[QUEUE] +bag loc=%d qty=%d qsize=%d\n", location, qty, (int)bagQueue.size());
  queueDisplayUntil = millis() + QUEUE_DISPLAY_HOLD_MS;
  updateQueueDisplay(true);
}

void runOtaUpdate(const String &url) {
  if (url.length() == 0 || WiFi.status() != WL_CONNECTED) {
    lastBirdMessage = "OTA skipped";
    updateDisplay(true);
    return;
  }

  dispenseBusy = true;
  lastBirdMessage = "OTA downloading";
  _otaLastPct = -1;
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(25, 55);
  tft.print("OTA Updating...");
  publishText("stat/" + String(client_id), "OTA downloading", false);

  httpUpdate.onProgress([](int current, int total) {
    if (total <= 0) return;
    int pct = (int)((long)current * 100 / total);
    if (pct == _otaLastPct) return;
    _otaLastPct = pct;
    tft.fillRect(0, 90, 320, 55, ST77XX_BLACK);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(5);
    String pctStr = String(pct) + "%";
    int pctW = (int)pctStr.length() * 30;
    int pctX = (320 - pctW) / 2;
    if (pctX < 0) pctX = 0;
    tft.setCursor(pctX, 98);
    tft.print(pctStr);
  });

  t_httpUpdate_return result;
  if (url.startsWith("https://")) {
    WiFiClientSecure secureClient;
    secureClient.setInsecure();
    result = httpUpdate.update(secureClient, url, FIRMWARE_VERSION);
  } else {
    WiFiClient plainClient;
    result = httpUpdate.update(plainClient, url, FIRMWARE_VERSION);
  }

  if (result == HTTP_UPDATE_OK) {
    publishText("stat/" + String(client_id), "OTA success, restarting", false);
    delay(500);
    ESP.restart();
  }

  lastBirdMessage = "OTA failed";
  publishText("stat/" + String(client_id), "OTA failed: " + String(httpUpdate.getLastErrorString()), false);
  updateDisplay(true);
  dispenseBusy = false;
}

void enterWiFiManager() {
#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
  esp_task_wdt_delete(NULL);
#else
  esp_task_wdt_deinit();
#endif
  tft.fillScreen(ST77XX_ORANGE);
  tft.setTextColor(ST77XX_BLACK);
  tft.setTextSize(3);
  tft.setCursor(30, 50);
  tft.println("SETUP MODE");
  tft.setTextSize(2);
  tft.setCursor(30, 95);
  tft.println("SSID: iLife-Wifi-Setup");
  lastBirdMessage = "WiFi setup mode";
  wifiSetupHint = "Starting AP...";
  beep(1, 100, 50);
  stopWebServerIfRunning();
  WiFi.disconnect();
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  delay(200);

  WiFiManager wm;
  WiFiManagerParameter cServer("server", "MQTT Server", mqtt_server, sizeof(mqtt_server));
  WiFiManagerParameter cPort("port", "MQTT Port", mqtt_port, sizeof(mqtt_port));
  WiFiManagerParameter cUser("user", "MQTT User", mqtt_user, sizeof(mqtt_user));
  WiFiManagerParameter cPass("pass", "MQTT Pass", mqtt_pass, sizeof(mqtt_pass));
  WiFiManagerParameter cDeviceId("devid", "Device ID", client_id, sizeof(client_id));

  wm.addParameter(&cServer);
  wm.addParameter(&cPort);
  wm.addParameter(&cUser);
  wm.addParameter(&cPass);
  wm.addParameter(&cDeviceId);
  wm.setTitle("iLife WiFi Setup");
  wm.setHostname(client_id);
  wm.setAPStaticIPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  wm.setWiFiAPChannel(1);
  wm.setWiFiAPHidden(false);
  wm.setCaptivePortalEnable(true);
  wm.setWiFiAutoReconnect(true);
  wm.setCleanConnect(true);
  wm.setConnectRetries(3);
  wm.setConnectTimeout(20);
  wm.setConfigPortalTimeout(180);
  wm.setBreakAfterConfig(true);
  wm.setAPCallback([](WiFiManager *manager) {
    lastBirdMessage = String("AP: ") + manager->getConfigPortalSSID();
    wifiSetupHint = String("Open ") + WiFi.softAPIP().toString();
    Serial.println("WiFi setup portal ready");
    Serial.print("AP SSID: ");
    Serial.println(manager->getConfigPortalSSID());
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    tft.fillScreen(ST77XX_ORANGE);
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(3);
    tft.setCursor(30, 30);
    tft.println("SETUP MODE");
    tft.setTextSize(2);
    tft.setCursor(10, 90);
    tft.print("SSID: ");
    tft.println(manager->getConfigPortalSSID());
    tft.setCursor(10, 115);
    tft.print("IP: ");
    tft.println(WiFi.softAPIP().toString());
  });
  wm.setSaveConfigCallback([]() {
    lastBirdMessage = "Saving WiFi config";
    wifiSetupHint = "Wait for restart";
    Serial.println("WiFi credentials saved");
    updateDisplay(true);
  });
  wm.setConfigPortalTimeoutCallback([]() {
    lastBirdMessage = "WiFi portal timeout";
    wifiSetupHint = "Hold MENU to retry";
    Serial.println("WiFi setup portal timeout");
    updateDisplay(true);
  });

  if (wm.startConfigPortal(WIFI_SETUP_AP_NAME)) {
    saveConfig(cServer.getValue(), cPort.getValue(), cUser.getValue(), cPass.getValue(), cDeviceId.getValue());
    delay(500);
    ESP.restart();
  }

  lastBirdMessage = "WiFi setup cancelled";
  wifiSetupHint = "Hold MENU to retry";
  updateDisplay(true);
  if (webRoutesConfigured) {
    startWebServerIfNeeded();
  }
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.setHostname(client_id);
  wifiSetupHint = "";

  String savedSSID = getSavedWiFiSSID();

  if (savedSSID.length() == 0) {
    lastBirdMessage = "No WiFi config";
    wifiSetupHint = "Open AP to configure";
    Serial.println("No saved WiFi config in STA NVS");
    updateDisplay(true);
    enterWiFiManager();
    return;
  }

  lastBirdMessage = "Join " + savedSSID;
  Serial.print("Connecting to saved WiFi SSID: ");
  Serial.println(savedSSID);
  updateDisplay(true);
  WiFi.begin();

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    updateDisplay(true);
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiSetupHint = "";
    lastBirdMessage = "WiFi connected " + WiFi.localIP().toString();
    Serial.print("WiFi connected, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    lastBirdMessage = "WiFi failed, setup AP";
    wifiSetupHint = "Use 2.4GHz router";
    Serial.println("WiFi connect failed, starting setup AP");
    updateDisplay(true);
    enterWiFiManager();
    return;
  }
  updateDisplay(true);
}

bool connectWifiWithCredentials(const String &ssid, const String &password) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(false);
  WiFi.setHostname(client_id);
  WiFi.disconnect(true);
  delay(200);
  WiFi.begin(ssid.c_str(), password.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    esp_task_wdt_reset();
    updateDisplay(true);
    delay(250);
  }

  return WiFi.status() == WL_CONNECTED;
}

void processPendingWiFiChange() {
  if (!pendingWiFiChange) {
    return;
  }

  String requestedSSID = pendingWiFiSSID;
  String requestedPassword = pendingWiFiPassword;
  pendingWiFiSSID = "";
  pendingWiFiPassword = "";
  pendingWiFiChange = false;

  String previousSSID;
  String previousPassword;
  if (!getSavedWiFiCredentials(previousSSID, previousPassword)) {
    publishWiFiConfigStatus("rejected", false, "current wifi credentials unavailable", requestedSSID);
    lastBirdMessage = "WiFi change rejected";
    updateDisplay(true);
    return;
  }

  publishWiFiConfigStatus("testing", false, "testing new wifi", requestedSSID);
  client.disconnect();

  bool connected = connectWifiWithCredentials(requestedSSID, requestedPassword);
  if (connected) {
    WiFi.setAutoReconnect(true);
    lastBirdMessage = "WiFi changed " + WiFi.localIP().toString();
    queueWiFiConfigResult("success", "wifi credentials changed", requestedSSID);
  } else {
    Serial.println("WiFi change failed, restoring previous credentials");
    bool restored = connectWifiWithCredentials(previousSSID, previousPassword);
    WiFi.setAutoReconnect(true);
    if (restored) {
      lastBirdMessage = "WiFi change failed, old WiFi restored";
      queueWiFiConfigResult("failed", "new wifi failed; previous wifi restored", requestedSSID);
    } else {
      lastBirdMessage = "WiFi restore failed";
      queueWiFiConfigResult("failed", "new wifi and previous wifi failed", requestedSSID);
    }
  }

  updateDisplay(true);
}

void reconnectMqtt() {
  if (client.connected() || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (millis() - lastMqttReconnect < 5000) {
    return;
  }

  lastMqttReconnect = millis();

  String lwtTopic = "tele/" + String(client_id) + "/STATE";
  String lwtPayload = "{";
  lwtPayload += "\"ssid\":\"" + jsonEscape(getWiFiSSID()) + "\"";
  lwtPayload += ",\"id\":\"" + jsonEscape(String(client_id)) + "\"";
  lwtPayload += ",\"wifi\":\"OFFLINE\"";
  lwtPayload += ",\"mqtt\":\"LOST\"";
  lwtPayload += ",\"rssi\":-100";
  lwtPayload += ",\"total\":" + String(totalDispenseCount);
  lwtPayload += ",\"bird\":\"WAIT\"";
  lwtPayload += "}";

  bool connected = false;
  if (strlen(mqtt_user) > 0) {
    connected = client.connect(client_id, mqtt_user, mqtt_pass, lwtTopic.c_str(), 1, true, lwtPayload.c_str());
  } else {
    connected = client.connect(client_id, lwtTopic.c_str(), 1, true, lwtPayload.c_str());
  }

  if (connected) {
    String commandTopic = "cmnd/" + String(client_id);
    client.subscribe(commandTopic.c_str());
    String wifiCommandTopic = commandTopic + "/WIFI";
    client.subscribe(wifiCommandTopic.c_str());
    publishText("stat/" + String(client_id), "MQTT connected", false);
    sendStatus();
    if (wifiConfigResultPending) {
      publishWiFiConfigStatus(wifiConfigResultStatus, wifiConfigResultStatus == "success",
                              wifiConfigResultMessage, wifiConfigResultSSID);
      wifiConfigResultPending = false;
    }
    updateDisplay(true);
  }
}

void callback(char *topic, byte *payload, unsigned int length) {
  String commandText;
  for (unsigned int i = 0; i < length; ++i) {
    commandText += static_cast<char>(payload[i]);
  }
  commandText.trim();

  String topicText = topic == nullptr ? "" : String(topic);
  String wifiCommandTopic = "cmnd/" + String(client_id) + "/WIFI";
  bool isWiFiConfigTopic = topicText == wifiCommandTopic;
  if (isWiFiConfigTopic) {
    Serial.printf("[MQTT] topic=%s payload=<wifi credentials>\n", topic);
    handleWiFiConfigCommand(commandText);
    return;
  }

  Serial.printf("[MQTT] topic=%s payload=%s\n", topic, commandText.c_str());

  // JSON command format: {"message":"...","commands_sent":[{"command":"BAG#...#loc#qty"}]}
  // Also handles items array: {"items":[{"command":"BAG#..."},{"command":"BAG#..."},...]}}
  if (commandText.startsWith("{")) {
    int searchFrom = 0;
    int foundCount = 0;
    int skippedCount = 0;
    String firstOrderId = "";
    uint8_t firstLocation = 0;
    uint8_t firstQty = 0;
    while (true) {
      int cmdIdx = commandText.indexOf("\"command\":\"", searchFrom);
      if (cmdIdx < 0) break;
      int cStart = cmdIdx + 11;
      int cEnd = commandText.indexOf('"', cStart);
      if (cEnd <= cStart) break;
      String innerCmd = commandText.substring(cStart, cEnd);
      String jOrderId;
      uint8_t jLoc = 0, jQty = 0;
      if (parseBagCommand(innerCmd, jOrderId, jLoc, jQty)) {
        if (firstOrderId.length() == 0) {
          firstOrderId = jOrderId;
          firstLocation = jLoc;
          firstQty = jQty;
        }
        if (bagRecoveryRequired || bagOrderAlreadyTracked(jOrderId, jLoc) || totalPendingJobCount() >= MAX_QUEUE_SIZE) {
          skippedCount++;
        } else {
          queueBag(jOrderId, jLoc, jQty, "mqtt");
          foundCount++;
        }
      }
      searchFrom = cEnd + 1;
    }
    if (foundCount > 0 || skippedCount > 0) {
      bool accepted = foundCount > 0;
      String ack = "{\"message\":\"" + String(accepted ? "success" : "fail") + "\"";
      ack += ",\"status\":\"" + String(accepted ? "completed" : "rejected") + "\"";
      ack += ",\"success\":" + String(accepted ? "true" : "false");
      ack += ",\"accepted\":" + String(accepted ? "true" : "false");
      ack += ",\"processing\":false";
      ack += ",\"completed\":" + String(accepted ? "true" : "false");
      ack += ",\"queued\":" + String(foundCount);
      if (skippedCount > 0) ack += ",\"skipped\":" + String(skippedCount);
      ack += ",\"queueSize\":" + String(totalPendingJobCount());
      if (firstOrderId.length() > 0) {
        ack += ",\"orderid\":\"" + jsonEscape(firstOrderId) + "\"";
        ack += ",\"location\":" + String(firstLocation);
        ack += ",\"qty\":" + String(firstQty);
      }
      ack += ",\"device\":\"" + jsonEscape(String(client_id)) + "\"";
      ack += ",\"device_id\":\"" + jsonEscape(String(client_id)) + "\"}";
      publishText("stat/" + String(client_id), ack, false);
      return;
    }
    publishText("stat/" + String(client_id), "{\"message\":\"unknown_command\",\"device\":\"" + String(client_id) + "\"}", false);
    return;
  }

  uint8_t productId = 0;
  uint8_t mode = 0;
  String upper = commandText;
  upper.toUpperCase();

  if (upper.startsWith("ORDER0#")) {
    publishText("stat/" + String(client_id), "{\"success\":true}", false);
    return;
  }

  String bagOrderId = "";
  uint8_t bagLocation = 0;
  uint8_t bagQty = 0;
  if (parseBagCommand(commandText, bagOrderId, bagLocation, bagQty)) {
    Serial.printf("[BAG] parsed orderId=%s location=%d qty=%d\n", bagOrderId.c_str(), bagLocation, bagQty);
    String bagCmd = "BAG#" + bagOrderId + "#" + String(bagLocation) + "#" + String(bagQty);
    if (bagRecoveryRequired) {
      String recovery = "{\"message\":\"recovery_required\",\"status\":\"recovery_required\",\"success\":false,\"accepted\":false,\"processing\":false,\"completed\":false";
      recovery += ",\"reason\":\"" + jsonEscape(bagRecoveryReason) + "\"";
      recovery += ",\"orderid\":\"" + jsonEscape(bagOrderId) + "\",\"device\":\"" + jsonEscape(String(client_id)) + "}";
      publishText("stat/" + String(client_id), recovery, false);
    } else if (bagOrderAlreadyTracked(bagOrderId, bagLocation)) {
      String duplicate = "{\"message\":\"duplicate\",\"status\":\"";
      duplicate += bagRecoveryRequired ? "recovery_required" : "duplicate";
      duplicate += "\",\"success\":false,\"accepted\":false,\"processing\":false,\"completed\":false";
      duplicate += ",\"orderid\":\"" + jsonEscape(bagOrderId) + "\"";
      duplicate += ",\"device\":\"" + jsonEscape(String(client_id)) + "\"}";
      publishText("stat/" + String(client_id), duplicate, false);
    } else if (totalPendingJobCount() >= MAX_QUEUE_SIZE) {
      String reject = "{\"message\":\"fail\",\"success\":false,\"accepted\":false,\"processing\":false,\"completed\":false";
      reject += ",\"reason\":\"queue_full\",\"queueSize\":" + String(totalPendingJobCount());
      reject += ",\"maxQueue\":" + String(MAX_QUEUE_SIZE);
      reject += ",\"command\":\"" + jsonEscape(bagCmd) + "\",\"device\":\"" + jsonEscape(String(client_id)) + "\"}";
      publishText("stat/" + String(client_id), reject, false);
    } else {
      size_t futurePos = totalPendingJobCount() + 1;
      String ack = "{";
      ack += "\"event\":\"bag_complete\"";
      ack += ",\"message\":\"success\"";
      ack += ",\"status\":\"completed\"";
      ack += ",\"accepted\":true";
      ack += ",\"processing\":false";
      ack += ",\"completed\":true";
      ack += ",\"location\":\"" + String(bagLocation) + "\"";
      ack += ",\"productcode\":\"" + jsonEscape(bagOrderId) + "\"";
      ack += ",\"qty\":" + String(bagQty);
      ack += ",\"command\":\"" + jsonEscape(bagCmd) + "\"";
      ack += ",\"success\":true";
      ack += ",\"stock_before\":0";
      ack += ",\"stock_after\":0";
      ack += ",\"queuePos\":" + String(futurePos);
      ack += ",\"queueSize\":" + String(futurePos);
      ack += ",\"device\":\"" + jsonEscape(String(client_id)) + "\"";
      ack += ",\"device_id\":\"" + jsonEscape(String(client_id)) + "\"";
      queueBag(bagOrderId, bagLocation, bagQty, "mqtt");
      publishText("stat/" + String(client_id), bagCmd, false);
      publishText("stat/" + String(client_id), ack, false);
    }
    return;
  }

  if (parseDispenseCommand(commandText, productId, mode)) {
    if (dispenseQueue.size() >= MAX_QUEUE_SIZE) {
      String rej = "{";
      rej += "\"status\":\"REJECTED\"";
      rej += ",\"reason\":\"queue_full\"";
      rej += ",\"cmd\":\"DISPENSE#" + String(productId) + "#" + String(mode) + "\"";
      rej += ",\"queueSize\":" + String(dispenseQueue.size());
      rej += ",\"maxQueue\":" + String(MAX_QUEUE_SIZE);
      rej += ",\"device\":\"" + jsonEscape(String(client_id)) + "\"";
      rej += "}";
      publishText("stat/" + String(client_id), rej, false);
    } else {
      size_t futurePos = dispenseQueue.size() + 1;
      String ack = "{";
      ack += "\"status\":\"QUEUED\"";
      ack += ",\"cmd\":\"DISPENSE#" + String(productId) + "#" + String(mode) + "\"";
      ack += ",\"productId\":" + String(productId);
      ack += ",\"mode\":" + String(mode);
      ack += ",\"queuePos\":" + String(futurePos);
      ack += ",\"queueSize\":" + String(futurePos);
      ack += ",\"device\":\"" + jsonEscape(String(client_id)) + "\"";
      ack += "}";
      publishText("stat/" + String(client_id), ack, false);
      queueDispense(productId, mode, "mqtt");
    }
    return;
  }

  if (upper == "TEST" || upper == "PING" || upper == "STATUS") {
    pendingBirdTest = true;
    return;
  }

  if (upper == "REBOOT" || upper == "RESTART") {
    publishText("stat/" + String(client_id), "Rebooting", false);
    delay(250);
    ESP.restart();
  }

  BirdTransportMode requestedMode;
  if (upper == "TTL" || upper == "RS485" || upper.startsWith("LINE#") || upper.startsWith("PHY#")) {
    String modeText = commandText;
    if (commandText.indexOf('#') >= 0) {
      modeText = getValue(commandText, '#', 1);
    }

    if (parseBirdTransportMode(modeText, requestedMode)) {
      setBirdTransportMode(requestedMode, "mqtt");
      publishText("stat/" + String(client_id), "Line " + String(birdTransportModeLabel()), false);
      return;
    }
  }

  BirdUartPinMode requestedPinMode;
  if (upper == "RX16TX17" || upper == "RX17TX16" || upper.startsWith("PINS#") || upper.startsWith("UART#")) {
    String pinText = commandText;
    if (commandText.indexOf('#') >= 0) {
      pinText = getValue(commandText, '#', 1);
    }

    if (parseBirdUartPinMode(pinText, requestedPinMode)) {
      setBirdUartPinMode(requestedPinMode, "mqtt");
      publishText("stat/" + String(client_id), "Pins " + String(birdUartPinModeLabel()), false);
      return;
    }
  }

  if (upper == "WIFISETUP" || upper == "WIFI_SETUP" || upper == "CONFIG") {
    pendingWiFiSetup = true;
    publishText("stat/" + String(client_id), "Queued WiFi setup", false);
    return;
  }

  if (upper.startsWith("OTA#")) {
    pendingOtaUrl = commandText.substring(4);
    pendingOtaUrl.trim();
    return;
  }

  publishText("stat/" + String(client_id), "Unknown: " + commandText, false);
}

void setupWebServer() {
  if (!webRoutesConfigured) {
    server.on("/", HTTP_GET, []() {
      server.send_P(200, "text/html", index_html);
    });

    server.on("/motor-delay", HTTP_GET, []() {
      server.send_P(200, "text/html", motor_delay_html);
    });

    server.on("/motor-delay-config", HTTP_GET, []() {
      if (!server.hasArg("enabled") || !server.hasArg("seconds")) {
        server.send(400, "application/json", "{\"error\":\"missing enabled or seconds\"}");
        return;
      }

      String enabledText = server.arg("enabled");
      enabledText.toLowerCase();
      bool requestedEnabled;
      if (enabledText == "1" || enabledText == "true" || enabledText == "on") {
        requestedEnabled = true;
      } else if (enabledText == "0" || enabledText == "false" || enabledText == "off") {
        requestedEnabled = false;
      } else {
        server.send(400, "application/json", "{\"error\":\"enabled must be 0 or 1\"}");
        return;
      }

      String secondsText = server.arg("seconds");
      secondsText.trim();
      if (secondsText.length() == 0) {
        server.send(400, "application/json", "{\"error\":\"seconds must be 0-5\"}");
        return;
      }
      for (size_t index = 0; index < secondsText.length(); ++index) {
        if (secondsText.charAt(index) < '0' || secondsText.charAt(index) > '9') {
          server.send(400, "application/json", "{\"error\":\"seconds must be 0-5\"}");
          return;
        }
      }

      long requestedSeconds = secondsText.toInt();
      if (requestedSeconds < 0 || requestedSeconds > MAX_BACKEND_DELAY_SECONDS) {
        server.send(400, "application/json", "{\"error\":\"seconds must be 0-5\"}");
        return;
      }

      backendDelayEnabled = requestedEnabled;
      backendDelaySeconds = static_cast<uint8_t>(requestedSeconds);
      saveBackendMotorDelay();

      String payload = "{\"ok\":true,\"enabled\":";
      payload += backendDelayEnabled ? "true" : "false";
      payload += ",\"seconds\":" + String(backendDelaySeconds) + "}";
      sendStatus();
      server.send(200, "application/json", payload);
    });

    server.on("/status", HTTP_GET, []() {
      server.send(200, "application/json", buildStatusJson());
    });

    server.on("/bag-recovery", HTTP_GET, []() {
      if (!bagRecoveryRequired) {
        server.send(200, "application/json", "{\"ok\":true,\"status\":\"not_required\"}");
        return;
      }

      String action = server.hasArg("action") ? server.arg("action") : "";
      action.toLowerCase();
      if (action != "clear") {
        server.send(409, "application/json", "{\"ok\":false,\"error\":\"manual_check_required\",\"action\":\"clear\"}");
        return;
      }

      size_t discarded = bagQueue.size();
      bagQueue.clear();
      bagRecoveryRequired = false;
      bagSpinInFlight = false;
      bagRecoveryReason = "";
      lastBirdMessage = "BAG recovery cleared";
      saveBagJournal(false);
      updateDisplay(true);
      sendStatus();

      String payload = "{\"ok\":true,\"status\":\"cleared\",\"discarded\":" + String(discarded) + "}";
      server.send(200, "application/json", payload);
    });

    server.on("/transport", HTTP_GET, []() {
      if (!server.hasArg("mode")) {
        server.send(400, "application/json", "{\"error\":\"missing mode\"}");
        return;
      }

      BirdTransportMode requestedMode;
      if (!parseBirdTransportMode(server.arg("mode"), requestedMode)) {
        server.send(400, "application/json", "{\"error\":\"mode must be ttl or rs485\"}");
        return;
      }

      setBirdTransportMode(requestedMode, "web");

      String payload = "{";
      payload += "\"ok\":true";
      payload += ",\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"message\":\"retest link\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/uart-pins", HTTP_GET, []() {
      if (!server.hasArg("mode")) {
        server.send(400, "application/json", "{\"error\":\"missing mode\"}");
        return;
      }

      BirdUartPinMode requestedPinMode;
      if (!parseBirdUartPinMode(server.arg("mode"), requestedPinMode)) {
        server.send(400, "application/json", "{\"error\":\"mode must be rx16tx17 or rx17tx16\"}");
        return;
      }

      setBirdUartPinMode(requestedPinMode, "web");

      String payload = "{";
      payload += "\"ok\":true";
      payload += ",\"mode\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"message\":\"retest link\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/uart-baud", HTTP_GET, []() {
      if (!server.hasArg("rate")) {
        server.send(400, "application/json", "{\"error\":\"missing rate\"}");
        return;
      }

      uint32_t requestedBaud = 0;
      if (!parseBirdBaudArg(server.arg("rate"), requestedBaud)) {
        server.send(400, "application/json", "{\"error\":\"rate must be 1200-1000000\"}");
        return;
      }

      setBirdBaudRate(requestedBaud, "web");

      String payload = "{";
      payload += "\"ok\":true";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"line\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"message\":\"retest link\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/motor-time", HTTP_GET, []() {
      long requestedSpin = server.hasArg("spin")
                             ? server.arg("spin").toInt()
                             : (server.hasArg("ms") ? server.arg("ms").toInt() : -1);
      long requestedRest = server.hasArg("rest")
                             ? server.arg("rest").toInt()
                             : static_cast<long>(birdRestTimeMs);
      if (requestedSpin < MIN_BIRD_SPIN_TIME_MS || requestedSpin > MAX_BIRD_SPIN_TIME_MS ||
          requestedRest < MIN_BIRD_REST_TIME_MS || requestedRest > MAX_BIRD_REST_TIME_MS) {
        server.send(400, "application/json", "{\"error\":\"spin 1000-15000, rest 0-15000\"}");
        return;
      }

      birdSpinTimeMs = static_cast<uint32_t>(requestedSpin);
      birdRestTimeMs = static_cast<uint32_t>(requestedRest);
      saveBirdMotorTiming();

      String payload = "{";
      payload += "\"ok\":true";
      payload += ",\"spinTimeMs\":" + String(birdSpinTimeMs);
      payload += ",\"restTimeMs\":" + String(birdRestTimeMs);
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/bird-debug", HTTP_GET, []() {
      String payload = "{";
      payload += "\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"debug\":\"" + jsonEscape(birdDebugLog) + "\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/test", HTTP_GET, []() {
      if (dispenseBusy) {
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
      }

      uint8_t address = BIRD_BOARD_ADDRESS;
      uint8_t command = BIRD_COMM_TEST_COMMAND;
      if (server.hasArg("addr") && !parseAddressArg(server.arg("addr"), address)) {
        server.send(400, "application/json", "{\"error\":\"addr must be 0-255\"}");
        return;
      }

      if (server.hasArg("cmd") && !parseCommandArg(server.arg("cmd"), command)) {
        server.send(400, "application/json", "{\"error\":\"cmd must be 0-255 or 0x00-0xFF\"}");
        return;
      }

      String replyText;
      bool ok = birdCommunicationTestAtAddress(address, command, replyText);
      lastBirdMessage = ok ? "TEST " + replyText : "TEST FAIL";
      updateDisplay(true);
      sendStatus();

      String payload = "{";
      payload += "\"ok\":";
      payload += ok ? "true" : "false";
      payload += ",\"addr\":" + String(address);
      payload += ",\"cmd\":" + String(command);
      payload += ",\"reply\":\"" + jsonEscape(replyText) + "\"";
      payload += ",\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"debug\":\"" + jsonEscape(birdDebugLog) + "\"";
      payload += "}";
      server.send(ok ? 200 : 504, "application/json", payload);
    });

    server.on("/scan", HTTP_GET, []() {
      if (dispenseBusy) {
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
      }

      uint8_t minAddr = 0;
      uint8_t maxAddr = 15;
      if (server.hasArg("min")) parseRangeValue(server.arg("min"), 0, 254, minAddr);
      if (server.hasArg("max")) parseRangeValue(server.arg("max"), 0, 254, maxAddr);
      if (maxAddr < minAddr) maxAddr = minAddr;

      String hits = "";
      for (uint8_t addr = minAddr; addr <= maxAddr; ++addr) {
        String replyText;
        bool ok = birdCommunicationTestAtAddress(addr, BIRD_COMM_TEST_COMMAND, replyText);
        if (ok) {
          if (hits.length() > 0) hits += ",";
          hits += String(addr);
        }
        // short gap between attempts to let bus settle
        delay(50);
      }

      String payload = "{";
      payload += "\"scan\":\"addr\"",
      payload += ",\"min\":" + String(minAddr);
      payload += ",\"max\":" + String(maxAddr);
      payload += ",\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"found\":[" + hits + "]";
      payload += ",\"debug\":\"" + jsonEscape(birdDebugLog) + "\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/baud-scan", HTTP_GET, []() {
      if (dispenseBusy) {
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
      }

      static const uint32_t bauds[] = {9600, 19200, 38400, 57600, 115200};
      static const size_t baudCount = sizeof(bauds) / sizeof(bauds[0]);

      String hits = "";
      uint32_t originalBaud = birdBaudRate;

      for (size_t bi = 0; bi < baudCount; ++bi) {
        birdBaudRate = bauds[bi];
        initBirdSerialPort();
        birdFlushInput();
        delay(50);

        String replyText;
        bool ok = birdCommunicationTestAtAddress(BIRD_BOARD_ADDRESS, BIRD_COMM_TEST_COMMAND, replyText);
        if (ok) {
          if (hits.length() > 0) hits += ",";
          hits += String(bauds[bi]);
        }
        delay(50);
      }

      // restore or keep best found baud
      if (hits.length() == 0) {
        birdBaudRate = originalBaud;
        initBirdSerialPort();
      } else {
        // keep last found baud (already set) and persist it
        saveBirdBaudRate();
      }

      String payload = "{";
      payload += "\"scan\":\"baud\"";
      payload += ",\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"found\":[" + hits + "]";
      payload += ",\"debug\":\"" + jsonEscape(birdDebugLog) + "\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/wifi-setup", HTTP_GET, []() {
      pendingWiFiSetup = true;
      server.send(202, "application/json", "{\"ok\":true,\"message\":\"switching to wifi setup\"}");
    });

    server.on("/de-re-invert", HTTP_GET, []() {
      birdDeReInvert = !birdDeReInvert;
      saveBirdDeReInvert();
      applyBirdTransportMode();
      birdFlushInput();
      birdLinkOk = false;
      String payload = "{";
      payload += "\"ok\":true";
      payload += ",\"deReInvert\":" + String(birdDeReInvert ? "true" : "false");
      payload += ",\"message\":\"DE/RE invert " + String(birdDeReInvert ? "ON" : "OFF") + ", retest link\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/loopback-test", HTTP_GET, []() {
      // Direct Serial2 loopback test — bypasses birdWriteBytes echo flush.
      // Requires jumper between RX pin and TX pin of current UART configuration.
      if (dispenseBusy) {
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
      }
      // Flush RX buffer
      while (Serial2.available()) Serial2.read();
      // Send 4 known bytes directly
      const uint8_t probe[] = {0x55, 0xAA, 0x48, 0x04};
      Serial2.write(probe, sizeof(probe));
      Serial2.flush();
      // Allow time for last byte + loopback propagation
      uint32_t usPerByte = (10000000UL + birdBaudRate - 1) / birdBaudRate;
      delayMicroseconds(usPerByte * (sizeof(probe) + 1) + 500);
      // Collect echoed bytes
      String hex = "";
      int count = 0;
      unsigned long start = millis();
      while (millis() - start < 500) {
        while (Serial2.available()) {
          uint8_t b = static_cast<uint8_t>(Serial2.read());
          if (hex.length() > 0) hex += " ";
          if (b < 16) hex += "0";
          hex += String(b, HEX);
          count++;
        }
        delay(1);
      }
      bool passed = (count == (int)sizeof(probe)) && (hex == "55 aa 48 04");
      String payload = "{";
      payload += "\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"txBytes\":" + String(sizeof(probe));
      payload += ",\"rxCount\":" + String(count);
      payload += ",\"rxHex\":\"" + hex + "\"";
      payload += ",\"expected\":\"55 aa 48 04\"";
      payload += ",\"passed\":" + String(passed ? "true" : "false");
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/gpio-test", HTTP_GET, []() {
      // Test digital connectivity between two GPIO pins (requires jumper wire between them).
      // Usage: /gpio-test?out=27&in=26
      int outPin = server.hasArg("out") ? server.arg("out").toInt() : 27;
      int inPin  = server.hasArg("in")  ? server.arg("in").toInt()  : 26;
      // Briefly reconfigure pins as digital I/O (restores Serial2 after)
      Serial2.end();
      delay(5);
      pinMode(outPin, OUTPUT);
      pinMode(inPin,  INPUT);
      delay(2);
      digitalWrite(outPin, HIGH);
      delay(2);
      int readHigh = digitalRead(inPin);
      digitalWrite(outPin, LOW);
      delay(2);
      int readLow = digitalRead(inPin);
      // Restore Serial2
      delay(5);
      initBirdSerialPort();

      bool passed = (readHigh == HIGH) && (readLow == LOW);
      String payload = "{";
      payload += "\"outPin\":" + String(outPin);
      payload += ",\"inPin\":"  + String(inPin);
      payload += ",\"readHigh\":" + String(readHigh);
      payload += ",\"readLow\":"  + String(readLow);
      payload += ",\"passed\":"   + String(passed ? "true" : "false");
      payload += ",\"note\":\"passed=true means jumper wire is between correct pins\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/raw-rx", HTTP_GET, []() {
      if (dispenseBusy) {
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
      }
      birdFlushInput();
      String hex = "";
      int count = 0;
      unsigned long start = millis();
      uint32_t listenMs = 3000;
      if (server.hasArg("ms")) {
        uint32_t v = server.arg("ms").toInt();
        if (v >= 100 && v <= 10000) listenMs = v;
      }
      while (millis() - start < listenMs) {
        while (Serial2.available()) {
          uint8_t b = static_cast<uint8_t>(Serial2.read());
          if (hex.length() > 0) hex += " ";
          if (b < 16) hex += "0";
          hex += String(b, HEX);
          count++;
        }
        delay(1);
      }
      String payload = "{";
      payload += "\"count\":" + String(count);
      payload += ",\"ms\":" + String(listenMs);
      payload += ",\"hex\":\"" + hex + "\"";
      payload += ",\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"deReInvert\":" + String(birdDeReInvert ? "true" : "false");
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/tx-raw", HTTP_GET, []() {
      // Send one 40-byte test packet then collect raw bytes for listenMs.
      // Does NOT enforce any protocol — shows whatever actually comes back.
      if (dispenseBusy) {
        server.send(409, "application/json", "{\"error\":\"busy\"}");
        return;
      }

      uint8_t address = BIRD_BOARD_ADDRESS;
      if (server.hasArg("addr")) parseAddressArg(server.arg("addr"), address);

      uint32_t listenMs = 2000;
      if (server.hasArg("ms")) {
        uint32_t v = server.arg("ms").toInt();
        if (v >= 100 && v <= 10000) listenMs = v;
      }

      uint8_t packet[BIRD_PACKET_SIZE];
      buildBirdPacket(address, 0x00, BIRD_COMM_TEST_COMMAND, nullptr, 0, packet);

      birdFlushInput();
      birdWriteBytes(packet, BIRD_PACKET_SIZE);  // TX with proper DE/RE

      String hex = "";
      int count = 0;
      unsigned long start = millis();
      while (millis() - start < listenMs) {
        while (Serial2.available()) {
          uint8_t b = static_cast<uint8_t>(Serial2.read());
          if (hex.length() > 0) hex += " ";
          if (b < 16) hex += "0";
          hex += String(b, HEX);
          count++;
        }
        delay(1);
      }

      String txHex = "";
      for (size_t i = 0; i < BIRD_PACKET_SIZE; i++) {
        if (i > 0) txHex += " ";
        if (packet[i] < 16) txHex += "0";
        txHex += String(packet[i], HEX);
      }

      String payload = "{";
      payload += "\"txBytes\":" + String(BIRD_PACKET_SIZE);
      payload += ",\"rxCount\":" + String(count);
      payload += ",\"listenMs\":" + String(listenMs);
      payload += ",\"mode\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"deReInvert\":" + String(birdDeReInvert ? "true" : "false");
      payload += ",\"addr\":" + String(address);
      payload += ",\"txPacket\":\"" + txHex + "\"";
      payload += ",\"rxHex\":\"" + hex + "\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

    server.on("/dispense", HTTP_GET, []() {
      if (!server.hasArg("id")) {
        server.send(400, "application/json", "{\"error\":\"missing id\"}");
        return;
      }

      uint8_t productId = 0;
      uint8_t mode = 2;
      uint8_t requestedMotor = 0;

      if (server.hasArg("mode") && !parseRangeValue(server.arg("mode"), 0, 5, mode)) {
        server.send(400, "application/json", "{\"error\":\"mode must be 0-5\"}");
        return;
      }

      if (isSpiralMotorMode(mode)) {
        if (!parseRangeValue(server.arg("id"), 1, SPIRAL_MOTOR_COUNT, requestedMotor) ||
            !getSpiralMotorCode(requestedMotor, productId)) {
          server.send(400, "application/json", "{\"error\":\"spiral motor must be 1-12\"}");
          return;
        }
      } else if (!parseRangeValue(server.arg("id"), 0, 99, productId)) {
        server.send(400, "application/json", "{\"error\":\"id must be 0-99\"}");
        return;
      }

      size_t pendingCount = totalPendingJobCount();
      if (pendingCount >= MAX_QUEUE_SIZE) {
        server.send(400, "application/json", "{\"error\":\"queue_full\",\"queueSize\":" + String(pendingCount) + ",\"maxQueue\":" + String(MAX_QUEUE_SIZE) + "}");
        return;
      }

      queueDispense(productId, mode, "web");
      String payload = "{";
      payload += "\"queued\":true";
      if (isSpiralMotorMode(mode)) payload += ",\"motor\":" + String(requestedMotor);
      payload += ",\"productId\":" + String(productId);
      payload += ",\"mode\":" + String(mode);
      payload += ",\"queuePos\":" + String(pendingCount + 1);
      payload += ",\"message\":\"QUEUED #" + String(pendingCount + 1) + "\"";
      payload += "}";
      server.send(202, "application/json", payload);
    });

    server.on("/bag-test", HTTP_GET, []() {
      server.send_P(200, "text/html", bagtest_html);
    });

    server.on("/bag", HTTP_GET, []() {
      if (bagRecoveryRequired) {
        String payload = "{\"error\":\"recovery_required\",\"accepted\":false,\"reason\":\"";
        payload += jsonEscape(bagRecoveryReason);
        payload += "\"}";
        server.send(409, "application/json", payload);
        return;
      }

      String bagId = "";
      uint8_t location = 1;
      uint8_t qty = 1;
      if (server.hasArg("location")) {
        uint8_t v = 0;
        if (!parseRangeValue(server.arg("location"), 1, SPIRAL_MOTOR_COUNT, v)) {
          server.send(400, "application/json", "{\"error\":\"location must be 1-12\"}");
          return;
        }
        location = v;
      }
      if (server.hasArg("qty")) {
        uint8_t v = 0;
        if (!parseRangeValue(server.arg("qty"), 1, 20, v)) {
          server.send(400, "application/json", "{\"error\":\"qty must be 1-20\"}");
          return;
        }
        qty = v;
      }
      if (server.hasArg("bagid")) {
        bagId = server.arg("bagid");
        if (bagId.length() > 70) bagId = bagId.substring(0, 70);
      }
      uint8_t productId = 0;
      if (!getSpiralMotorCode(location, productId)) {
        server.send(400, "application/json", "{\"error\":\"spiral motor must be 1-12\"}");
        return;
      }
      if (bagOrderAlreadyTracked(bagId, location)) {
        server.send(409, "application/json", "{\"error\":\"duplicate_order\",\"accepted\":false}");
        return;
      }
      size_t pendingCount = totalPendingJobCount();
      if (pendingCount >= MAX_QUEUE_SIZE) {
        server.send(400, "application/json", "{\"error\":\"queue_full\",\"queueSize\":" + String(pendingCount) + ",\"maxQueue\":" + String(MAX_QUEUE_SIZE) + "}");
        return;
      }

      queueBag(bagId, location, qty, "web");
      String payload = "{";
      payload += "\"message\":\"success\"";
      payload += ",\"status\":\"queued\"";
      payload += ",\"accepted\":true";
      payload += ",\"processing\":true";
      payload += ",\"completed\":false";
      payload += ",\"queued\":true";
      payload += ",\"bagId\":\"" + jsonEscape(bagId) + "\"";
      payload += ",\"location\":" + String(location);
      payload += ",\"productId\":" + String(productId);
      payload += ",\"qty\":" + String(qty);
      payload += ",\"queuePos\":" + String(pendingCount + 1);
      payload += ",\"queueMessage\":\"QUEUED #" + String(pendingCount + 1) + "\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });

#if 0
    server.on("/dash", HTTP_GET, []() {
      server.send_P(200, "text/html", dash_html);
    });

    server.on("/dash-status", HTTP_GET, []() {
      String payload = "{";
      String wifiSSID = getWiFiSSID();
      payload += "\"id\":\"" + jsonEscape(String(client_id)) + "\"";
      payload += ",\"fw\":\"" + String(FIRMWARE_VERSION) + "\"";
      payload += ",\"wifi\":\"" + String(WiFi.status() == WL_CONNECTED ? "ONLINE" : "OFFLINE") + "\"";
      payload += ",\"ssid\":\"" + jsonEscape(wifiSSID) + "\"";
      payload += ",\"mqtt\":\"" + String(client.connected() ? "READY" : "WAIT") + "\"";
      payload += ",\"bird\":\"" + String(birdLinkOk ? "ONLINE" : "WAIT") + "\"";
      payload += ",\"line\":\"" + String(birdTransportModeLabel()) + "\"";
      payload += ",\"pins\":\"" + String(birdUartPinModeLabel()) + "\"";
      payload += ",\"baud\":" + String(birdBaudRate);
      payload += ",\"spinTimeMs\":" + String(birdSpinTimeMs);
      payload += ",\"restTimeMs\":" + String(birdRestTimeMs);
      if (WiFi.status() == WL_CONNECTED) {
        payload += ",\"ip\":\"" + WiFi.localIP().toString() + "\"";
      }
      payload += ",\"busy\":" + String(dispenseBusy ? "true" : "false");
      payload += ",\"last\":\"" + jsonEscape(lastBirdMessage) + "\"";
      payload += ",\"dispOk\":" + String(lastDispense.success ? "true" : "false");
      payload += ",\"dispId\":" + String(lastDispense.productId);
      payload += ",\"dispMsg\":\"" + jsonEscape(lastDispense.message) + "\"";
      payload += ",\"dispMs\":" + String(lastDispense.useTimeMs);
      payload += ",\"bagPending\":" + String(!bagQueue.empty() ? "true" : "false");
      payload += ",\"queueSize\":" + String(bagQueue.size() + dispenseQueue.size());
      payload += ",\"bagQueue\":[";
      for (size_t i = 0; i < bagQueue.size(); i++) {
        if (i > 0) payload += ",";
        payload += "{\"pos\":" + String(i + 1);
        payload += ",\"orderId\":\"" + jsonEscape(bagQueue[i].orderId) + "\"";
        payload += ",\"location\":" + String(bagQueue[i].location);
        payload += ",\"qty\":" + String(bagQueue[i].qty) + "}";
      }
      payload += "]";
      payload += ",\"dispQueue\":[";
      for (size_t i = 0; i < dispenseQueue.size(); i++) {
        if (i > 0) payload += ",";
        payload += "{\"pos\":" + String(i + 1);
        payload += ",\"productId\":" + String(dispenseQueue[i].productId);
        payload += ",\"mode\":" + String(dispenseQueue[i].mode) + "}";
      }
      payload += "]";
      payload += ",\"lastOrderId\":\"" + jsonEscape(lastBagOrderId) + "\"";
      payload += ",\"lastOrderLocation\":" + String(lastBagOrderLocation);
      payload += ",\"lastOrderQty\":" + String(lastBagOrderQty);
      payload += ",\"lastOrderOk\":" + String(lastBagOrderOk ? "true" : "false");
      payload += ",\"topicCmd\":\"cmnd/" + jsonEscape(String(client_id)) + "\"";
      payload += ",\"topicStat\":\"stat/" + jsonEscape(String(client_id)) + "\"";
      payload += ",\"topicTele\":\"tele/" + jsonEscape(String(client_id)) + "/STATE\"";
      payload += "}";
      server.send(200, "application/json", payload);
    });
#endif

    webRoutesConfigured = true;
  }

  startWebServerIfNeeded();
}

void handleButtons() {
  bool menuPressed = menuButtonEnabled && digitalRead(btnMenu) == LOW;
  bool rebootRaw = digitalRead(btnReboot) == LOW;
  if (!rebootButtonEnabled && !rebootRaw) {
    rebootButtonEnabled = true;  // re-enable once button is released after boot
  }
  bool rebootPressed = rebootButtonEnabled && rebootRaw;

  if (menuPressed) {
    if (menuPressStarted == 0) {
      menuPressStarted = millis();
    } else if (millis() - menuPressStarted >= 3000) {
      menuPressStarted = 0;
      enterWiFiManager();
    }
  } else {
    menuPressStarted = 0;
  }

  if (rebootPressed) {
    if (rebootPressStarted == 0) {
      rebootPressStarted = millis();
    } else if (millis() - rebootPressStarted >= 1500) {
      ESP.restart();
    }
  } else {
    rebootPressStarted = 0;
  }
}

void processPendingJobs() {
  if (dispenseBusy) {
    return;
  }

  if (queueDisplayUntil != 0) {
    if (static_cast<int32_t>(queueDisplayUntil - millis()) > 0) {
      return;
    }
    queueDisplayUntil = 0;
  }

  if (pendingWiFiChange) {
    processPendingWiFiChange();
    return;
  }

  if (pendingWiFiSetup) {
    pendingWiFiSetup = false;
    enterWiFiManager();
    return;
  }

  if (pendingOtaUrl.length() > 0) {
    String otaUrl = pendingOtaUrl;
    pendingOtaUrl = "";
    runOtaUpdate(otaUrl);
    return;
  }

  if (bagRecoveryRequired) {
    if (lastBirdMessage != "RECOVERY REQUIRED") {
      lastBirdMessage = "RECOVERY REQUIRED";
      updateDisplay(true);
    }
    return;
  }

  if (!bagQueue.empty()) {
    PendingBagJob &job = bagQueue.front();
    if (job.sentQty == 0) {
      lastBirdMessage = "BAG#" + job.orderId + "#" + String(job.location) + "#" + String(job.qty);
      Serial.printf("[QUEUE] start bag loc=%d qty=%d qsize=%d\n", job.location, job.qty, (int)bagQueue.size());
      updateDisplay(true);
    }

    bool canContinue = executeBagDispenseSpin(job);
    if (job.sentQty >= job.qty) {
      PendingBagJob completedJob = job;
      bagQueue.pop_front();
      completeBagDispense(completedJob);
      updateDisplay(true);
    } else if (!canContinue && !bagRecoveryRequired) {
      PendingBagJob completedJob = job;
      bagQueue.pop_front();
      completeBagDispense(completedJob);
      updateDisplay(true);
    } else if (!canContinue) {
      saveBagJournal(false);
      lastBirdMessage = "RECOVERY REQUIRED";
      updateDisplay(true);
    }
    return;
  }

  if (birdDispenseCooldownActive()) {
    return;
  }

  if (!dispenseQueue.empty()) {
    PendingDispenseJob job = dispenseQueue.front();
    dispenseQueue.pop_front();
    waitForBackendMotorDelay(job.source);
    executeDispense(job.productId, job.mode, job.source);
    return;
  }

  if (pendingBirdTest) {
    pendingBirdTest = false;
    String replyText;
    bool ok = birdCommunicationTest(replyText);
    lastBirdMessage = ok ? "TEST " + replyText : "TEST FAIL";
    updateDisplay(true);
    publishText("stat/" + String(client_id), lastBirdMessage, false);
    sendStatus();
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(RELAY1_PIN, OUTPUT);
  digitalWrite(RELAY1_PIN, LOW);

  pinMode(RS485_EN, OUTPUT);
  digitalWrite(RS485_EN, LOW);

  pinMode(btnMenu, INPUT);
  pinMode(btnReboot, INPUT);
  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  menuButtonEnabled = pinReleasedAtBoot(btnMenu);
  rebootButtonEnabled = pinReleasedAtBoot(btnReboot);
  Serial.print("Menu button: ");
  Serial.println(menuButtonEnabled ? "enabled" : "disabled (floating/stuck low)");
  Serial.print("Reboot button: ");
  Serial.println(rebootButtonEnabled ? "enabled" : "disabled (floating/stuck low)");

  tft.init(172, 320);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextWrap(false);
  showSplashScreen();
  updateDisplay(true);

  loadConfig();
  loadBagJournal();
  initBirdSerialPort();
  applyBirdTransportMode();
  updateDisplay(true);

  client.setServer(mqtt_server, atoi(mqtt_port));
  client.setCallback(callback);
  client.setKeepAlive(60);
  client.setBufferSize(4096);

  connectWifi();
  configTime(25200, 0, "pool.ntp.org", "time.cloudflare.com");
  setupWebServer();

  String replyText;
  if (birdCommunicationTest(replyText)) {
    lastBirdMessage = "TEST " + replyText;
  } else {
    lastBirdMessage = "BIRD no response";
  }
  if (bagRecoveryRequired) {
    lastBirdMessage = "RECOVERY REQUIRED";
  }
  updateDisplay(true);
}

void loop() {
  handleButtons();

  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiRetry = 0;
    if (millis() - lastWifiRetry >= 15000) {
      lastWifiRetry = millis();
      WiFi.reconnect();
    }
  }

  reconnectMqtt();

  if (client.connected()) {
    client.loop();
  }

  server.handleClient();

  processPendingJobs();

  if (!dispenseBusy && millis() - lastBirdProbe >= 60000) {
    lastBirdProbe = millis();
    String replyText;
    if (birdCommunicationTest(replyText)) {
      lastBirdMessage = "TEST " + replyText;
    } else {
      lastBirdMessage = "BIRD no response";
    }
    updateDisplay(true);
  }

  if (millis() - lastHeartbeat >= 10000) {
    lastHeartbeat = millis();
    sendStatus();
  }

  updateDisplay();
}