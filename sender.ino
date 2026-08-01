/*
 * ============================================================
 *  ESP32-C3 SuperMini 发送端模块
 * ============================================================
 * 功能说明:
 *   - 开机 10 秒内为配对模式, 板载 LED 闪烁
 *   - 配对完成后 LED 熄灭
 *   - 输入引脚(GPIO10) LOW→HIGH 时发送 "find"
 *   - 输入引脚(GPIO10) HIGH→LOW 时发送 "unfind"
 *   - 每 1 分钟检测电池电压, 多次平均并记录
 *   - 连续 5 次电压 < 3.3V 时, 电量 LED 闪烁 10 下后深度睡眠
 *   - 触发开关(中断): 电量 LED 显示电量等级, 循环 3 次
 * 通信方式: ESP-NOW (2.4GHz 无线广播, 免连接)
 * 组网说明:
 *   - 配对期间广播自身角色(sender), 接收端会记录发送端 MAC
 *   - 多组可并存: 不同时段开机的设备组互不干扰(MAC 过滤)
 *   - 情景1: A,B + 1 配对 → A/B 发送, 1 响应
 *   - 情景2: C + 2,3 配对 → C 发送, 2/3 响应
 *   - 情景3: AB1配对后, 再开机C23 → 两路独立互不干扰
 * 引脚分配:
 *   - GPIO8  : 板载 LED (普通 LED, digitalWrite 控制)
 *              LED熄灭时引脚状态: HIGH
 *              LED点亮时引脚状态: LOW
 *   - GPIO10 : 输入信号引脚 (内部下拉, 低→高发find, 高→低发unfind)
 *   - GPIO0  : ADC 电压检测引脚 (1:1 限压电阻分压)
 *   - GPIO1  : 电量 LED (接 LED 到地, 高电平点亮/低电平熄灭)
 *   - GPIO2  : 开关 (接开关到地, 内部上拉, 低电平触发)
 * 兼容 ESP32 Arduino Core 2.x / 3.x
 * ============================================================
 */
#include <esp_now.h>
#include <WiFi.h>
#include <esp_sleep.h>

// ==================== 配置 ====================
#define LED_PIN            8       // 板载 LED 引脚
#define INPUT_PIN          10     // 输入信号引脚
#define PAIRING_TIME_MS    10000   // 配对模式持续 10 秒
#define PAIRING_TX_MS      500     // 配对广播间隔 500ms
#define DEBOUNCE_MS        50      // 输入消抖 50ms

// LED 电平配置 (ESP32-C3 SuperMini 板载 LED 为低电平点亮)
#define LED_ON   LOW              // 点亮 LED
#define LED_OFF  HIGH             // 熄灭 LED

// 电量检测配置
#define BATTERY_ADC_PIN    0      // ADC 电压检测引脚 (1:1 分压)
#define BATTERY_LED_PIN    1      // 电量 LED 引脚 (接LED到地, 高电平点亮)
#define SWITCH_PIN         2      // 开关引脚 (接开关到地, 内部上拉, 低电平触发)
#define BATTERY_CHECK_MS   60000  // 电量检测间隔 1 分钟
#define BATTERY_LOW_COUNT  5      // 连续低电压次数阈值
#define BATTERY_LOW_TH     3.3    // 低电压阈值 (V)
#define ADC_SAMPLES        10     // ADC 采样次数 (多次平均)

// 电量 LED 电平 (接 LED 到地, 高电平点亮)
#define BAT_LED_ON   HIGH
#define BAT_LED_OFF  LOW
// ===============================================

// ESP-NOW 广播地址
const uint8_t BROADCAST_ADDR[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// 消息魔数 (用于识别本系统的消息, 过滤其他 ESP-NOW 设备)
#define MSG_MAGIC 0xF5

// 消息类型
#define MSG_TYPE_PAIRING  0
#define MSG_TYPE_DATA     1

// 角色
#define ROLE_SENDER   0
#define ROLE_RECEIVER 1

// 命令
#define CMD_FIND   0
#define CMD_UNFIND 1

// 消息结构体 (4 字节)
struct Message {
  uint8_t magic;    // 魔数 0xF5
  uint8_t type;     // 消息类型
  uint8_t role;     // 发送者角色
  uint8_t command;  // 命令
};

// ==================== 状态变量 ====================
bool isPairing = true;
unsigned long pairingStartTime = 0;
unsigned long lastBroadcastTime = 0;

// 输入引脚消抖
int lastInputReading = LOW;
int currentInputState = LOW;
unsigned long lastDebounceTime = 0;

// 电量检测状态
volatile bool switchPressed = false;
volatile unsigned long lastSwitchISRTime = 0;
unsigned long lastBatteryCheck = 0;
int lowBatteryCount = 0;

// ==================== 电量检测函数 ====================

// 读取电池电压 (1:1 分压, 实际电压 = ADC电压 × 2)
float readBatteryVoltage() {
  uint32_t sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(BATTERY_ADC_PIN);
    delay(5);
  }
  float avg = (float)sum / ADC_SAMPLES;
  // 12-bit ADC (0-4095), 3.3V 参考电压, 1:1 分压 → 乘 2
  return avg * 3.3 * 2.0 / 4095.0;
}

// 电量检测: 每 1 分钟调用, 连续 5 次低电压则关机
void checkBattery() {
  float v = readBatteryVoltage();
  Serial.printf("[电量] 电压: %.2fV\n", v);

  if (v < BATTERY_LOW_TH) {
    lowBatteryCount++;
    Serial.printf("[电量] 低电压计数: %d/%d\n", lowBatteryCount, BATTERY_LOW_COUNT);
    if (lowBatteryCount >= BATTERY_LOW_COUNT) {
      Serial.println("[电量] 电量严重不足, 即将关机...");
      // 关闭板载 LED
      digitalWrite(LED_PIN, LED_OFF);
      // 电量 LED 闪烁 10 下
      for (int i = 0; i < 10; i++) {
        digitalWrite(BATTERY_LED_PIN, BAT_LED_ON);
        delay(200);
        digitalWrite(BATTERY_LED_PIN, BAT_LED_OFF);
        delay(200);
      }
      // 深度睡眠, 可按开关唤醒重启
      esp_deep_sleep_enable_gpio_wakeup(1ULL << SWITCH_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);
      esp_deep_sleep_start();
    }
  } else {
    if (lowBatteryCount > 0) {
      lowBatteryCount = 0;
      Serial.println("[电量] 电压恢复, 计数清零");
    }
  }
}

// 显示电量等级: 快速连闪 + 停顿, 循环 3 次
void showBatteryLevel() {
  float v = readBatteryVoltage();
  Serial.printf("[电量] 当前电压: %.2fV\n", v);

  int flashCount;  // 0 = 常亮
  if (v >= 4.15) {
    flashCount = 0;  // 常亮
    Serial.println("[电量] 等级: 满电 (常亮)");
  } else if (v >= 3.95) {
    flashCount = 3;
    Serial.println("[电量] 等级: 闪3下");
  } else if (v >= 3.75) {
    flashCount = 2;
    Serial.println("[电量] 等级: 闪2下");
  } else {
    flashCount = 1;
    Serial.println("[电量] 等级: 闪1下");
  }

  // 循环 3 次: 快速连闪 + 停顿
  for (int cycle = 0; cycle < 3; cycle++) {
    if (flashCount == 0) {
      // 常亮 1s
      digitalWrite(BATTERY_LED_PIN, BAT_LED_ON);
      delay(1000);
      digitalWrite(BATTERY_LED_PIN, BAT_LED_OFF);
      delay(300);
    } else {
      // 快速闪烁
      for (int i = 0; i < flashCount; i++) {
        digitalWrite(BATTERY_LED_PIN, BAT_LED_ON);
        delay(100);
        digitalWrite(BATTERY_LED_PIN, BAT_LED_OFF);
        delay(100);
      }
      // 停顿
      delay(500);
    }
  }
  digitalWrite(BATTERY_LED_PIN, BAT_LED_OFF);
}

// 开关中断服务函数 (消抖 200ms)
void IRAM_ATTR onSwitchPress() {
  unsigned long now = millis();
  if (now - lastSwitchISRTime > 200) {
    switchPressed = true;
    lastSwitchISRTime = now;
  }
}

// ==================== ESP-NOW 回调 ====================

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  // 发送状态回调 (未使用)
}
#else
void onDataSent(const uint8_t *macAddr, esp_now_send_status_t status) {
  // 发送状态回调 (未使用)
}
#endif

void processRecv(const uint8_t *mac, const uint8_t *data, int len) {
  // 发送端使用广播发送, 无需存储接收端 MAC
  (void)mac;
  (void)data;
  (void)len;
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  processRecv(info->src_addr, data, len);
}
#else
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  processRecv(mac, data, len);
}
#endif

// ==================== 消息发送 ====================

void sendPairingMessage() {
  Message msg;
  msg.magic   = MSG_MAGIC;
  msg.type    = MSG_TYPE_PAIRING;
  msg.role    = ROLE_SENDER;
  msg.command = 0;
  esp_now_send(BROADCAST_ADDR, (uint8_t *)&msg, sizeof(msg));
}

void sendDataMessage(uint8_t command) {
  Message msg;
  msg.magic   = MSG_MAGIC;
  msg.type    = MSG_TYPE_DATA;
  msg.role    = ROLE_SENDER;
  msg.command = command;
  esp_now_send(BROADCAST_ADDR, (uint8_t *)&msg, sizeof(msg));
}

// ==================== 初始化 ====================

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n====== ESP32-C3 发送端 ======");

  // 初始化板载 LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);

  // 初始化输入引脚 (内部下拉, 默认低电平)
  pinMode(INPUT_PIN, INPUT_PULLDOWN);
  lastInputReading = digitalRead(INPUT_PIN);
  currentInputState = lastInputReading;

  // 初始化电量检测引脚
  analogReadResolution(12);
  pinMode(BATTERY_ADC_PIN, INPUT);
  pinMode(BATTERY_LED_PIN, OUTPUT);
  digitalWrite(BATTERY_LED_PIN, BAT_LED_OFF);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SWITCH_PIN), onSwitchPress, FALLING);
  lastBatteryCheck = millis();

  // 初始化 WiFi (STA 模式, 不连接 AP)
  WiFi.mode(WIFI_STA);
  Serial.print("MAC 地址: ");
  Serial.println(WiFi.macAddress());

  // 初始化 ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 初始化失败!");
    // 错误指示: LED 快闪
    while (true) {
      digitalWrite(LED_PIN, LED_ON);
      delay(200);
      digitalWrite(LED_PIN, LED_OFF);
      delay(200);
    }
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // 添加广播对端 (ESP-NOW 广播需先添加广播地址为 peer)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, BROADCAST_ADDR, 6);
  peerInfo.channel = 0;       // 0 = 当前信道
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;    // 广播不支持加密
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("添加广播对端失败!");
  }

  // 进入配对模式
  isPairing = true;
  pairingStartTime = millis();
  lastBroadcastTime = 0;
  Serial.println("配对模式已启动 (10秒)...");
}

// ==================== 主循环 ====================

void loop() {
  unsigned long now = millis();

  // ========== 电量检测 (每 1 分钟) ==========
  if (now - lastBatteryCheck >= BATTERY_CHECK_MS) {
    lastBatteryCheck = now;
    checkBattery();
  }

  // ========== 开关触发: 显示电量 ==========
  if (switchPressed) {
    switchPressed = false;
    showBatteryLevel();
  }

  // ========== 配对模式 ==========
  if (isPairing) {
    // 检查配对是否结束
    if (now - pairingStartTime >= PAIRING_TIME_MS) {
      isPairing = false;
      digitalWrite(LED_PIN, LED_OFF);  // 配对完成, LED 熄灭
      // 配对结束后重新读取输入状态 (避免漏掉配对期间的电平变化)
      lastInputReading = digitalRead(INPUT_PIN);
      currentInputState = lastInputReading;
      Serial.println("配对完成, 进入工作模式。");
      return;
    }
    // LED 闪烁 (200ms 周期: 亮200ms / 灭200ms)
    digitalWrite(LED_PIN, (now / 200) % 2 == 0 ? LED_ON : LED_OFF);
    // 每 500ms 广播一次配对消息
    if (now - lastBroadcastTime >= PAIRING_TX_MS) {
      lastBroadcastTime = now;
      sendPairingMessage();
    }
    return;
  }

  // ========== 工作模式: 监控输入引脚 ==========
  int reading = digitalRead(INPUT_PIN);

  // 消抖: 引脚电平变化时重置计时器
  if (reading != lastInputReading) {
    lastDebounceTime = now;
  }
  lastInputReading = reading;

  // 消抖后确认电平变化
  if ((now - lastDebounceTime) > DEBOUNCE_MS) {
    if (reading != currentInputState) {
      currentInputState = reading;
      if (currentInputState == HIGH) {
        // LOW → HIGH: 发送 find
        Serial.println("[发送] find");
        sendDataMessage(CMD_FIND);
      } else {
        // HIGH → LOW: 发送 unfind
        Serial.println("[发送] unfind");
        sendDataMessage(CMD_UNFIND);
      }
    }
  }
}
