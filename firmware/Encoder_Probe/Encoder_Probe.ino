/*
 * 格物编码器探针（临时诊断草图，不属于正式固件）
 * ================================================
 *
 * 用途：正式固件里编码器不响应，先撇清「MCU 到底看不看得见信号」。
 * 本草图不点灯、不碰 IMU，只盯着编码器三根线（A / B / 按压），把电平、电压和跳变打出来。
 *
 * 输出分两段：
 *   一、上电静态体检（约 0.5 秒一次，只打一遍）
 *       每个引脚在 INPUT（只靠板上 4.7k 外部上拉）、INPUT_PULLDOWN、INPUT_PULLUP 三种模式下各读一次，
 *       同时用 ADC 读电压。这一段的用途是区分三种故障：
 *         外部上拉正常但转不动  → 编码器本体 / C 脚接地 / 机械故障
 *         PULLDOWN 下也掉到 0V  → 外部上拉没焊、断线，或线路被短路到 GND
 *         INPUT 下就是低电平    → 触点在静止时就闭合 / 短路到地
 *   二、动态监视（每 500 ms 一行，直到断电）
 *       A / B 每变化一次就记一条边沿（带微秒时间戳与解出的方向），并累计正交跳变与「格」。
 *       边沿同时进环形缓冲，主循环取出来打印，ISR 里不碰串口。
 *
 * 怎么用：烧完打开串口（921600），先别碰编码器看静态体检，然后慢慢转 3~5 格、再快速转几圈、最后按几下。
 *   转得动 → 会出现 [EDGE] 行，[STA] 行的 edges / detent 跟着涨，硬件链路是通的，问题回固件。
 *   转不动 → [EDGE] 一条都不出，比较静态体检里 A / B 的三种模式读数定位断点。
 *
 * 引脚以《硬件接线.md》4.3 / 5.5 节为准：A = GPIO1、B = GPIO0、按压 = GPIO3。
 * 三个脚都配 4.7k 上拉到 3.3V，编码器 C（中间脚）接 GND。按下时 A / B 被短到 GND，低电平有效。
 *
 * 板型与波特率与正式固件一致：AirM2M CORE ESP32C3，CDCOnBoot=Disabled，921600。
 */

#include <Arduino.h>

// ============================ 引脚（GPIO 号） ============================
static const uint8_t PIN_ENCA  = 1;   // 编码器 A 相，4.7k 上拉，无电容
static const uint8_t PIN_ENCB  = 0;   // 编码器 B 相，4.7k 上拉，无电容
static const uint8_t PIN_ENCSW = 3;   // 编码器按压，4.7k 上拉 + 100nF 到地

static const uint32_t SERIAL_BAUD = 921600;
static const uint32_t STATUS_MS   = 500;    // 动态监视的状态行周期
static const uint32_t EDGE_PRINT_MAX = 400; // 超过这么多条边沿后只留状态行，免得刷屏
static const uint16_t EDGE_RING   = 512;    // 边沿环形缓冲深度

// ============================ 正交解码 ============================
// 索引 = 上次状态 × 4 + 本次状态，状态值 = (A << 1) | B。与正式固件同一张表。
// 非法跳变（两相同时变）返回 0，这里单独记数——转得快时出现少量是正常的，静止时一直不归零才说明有毛刺。
static const int8_t ENC_TABLE[16] = {
  //  本次:  00   01   10   11
             0,  -1,  +1,   0,   // 上次 00
            +1,   0,   0,  -1,   // 上次 01
            -1,   0,   0,  +1,   // 上次 10
             0,  +1,  -1,   0,   // 上次 11
};

// ============================ 边沿记录 ============================
// 索引 0 = A，1 = B，2 = 按压。A / B 共用同一套正交状态，按压只计边沿。
enum ProbePin : uint8_t { EP_A = 0, EP_B = 1, EP_SW = 2, EP_COUNT = 3 };
static const uint8_t PROBE_PINS[EP_COUNT] = { PIN_ENCA, PIN_ENCB, PIN_ENCSW };
static const char   *PROBE_NAMES[EP_COUNT] = { "A(GPIO1)", "B(GPIO0)", "SW(GPIO3)" };

struct EdgeEvent {
  uint32_t us;      // micros() 时间戳
  uint8_t  pin;     // ProbePin
  uint8_t  level;   // 该脚变化后的电平
  uint8_t  state;   // A/B 两相合成状态 (A << 1) | B
  int8_t   step;    // 本次解出的正交步进，仅 A / B 有意义
};

static volatile EdgeEvent edgeRing[EDGE_RING];
static volatile uint16_t  ringHead = 0;   // 写入位置
static volatile uint16_t  ringTail = 0;   // 读取位置
static volatile uint32_t  ringLost = 0;   // 缓冲满时丢掉的边沿数

static volatile uint32_t edgeCount[EP_COUNT] = { 0, 0, 0 };
static volatile uint32_t lastEdgeUs[EP_COUNT] = { 0, 0, 0 };
static volatile uint32_t minGapUs[EP_COUNT]  = { 0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu };

static volatile int8_t  encPrev  = 0;
static volatile int8_t  encAccum = 0;
static volatile int32_t encDetent = 0;
static volatile uint32_t invalidSteps = 0;

// ============================ 状态统计 ============================
static uint32_t printedEdges   = 0;
static bool     edgeCapAnnounced = false;
static uint32_t statusCount    = 0;

// ============================ ISR ============================
// 只做三件事：读当前两相状态、查表累加、把事件塞进环形缓冲。串口一律留给主循环。
static IRAM_ATTR void pushEdge(uint8_t pin, uint8_t level, uint8_t state, int8_t step) {
  const uint32_t now = micros();
  const uint32_t prevUs = lastEdgeUs[pin];
  if (prevUs != 0) {
    const uint32_t gap = now - prevUs;
    if (gap < minGapUs[pin]) minGapUs[pin] = gap;
  }
  lastEdgeUs[pin] = now;
  edgeCount[pin]++;

  const uint16_t next = (uint16_t)((ringHead + 1) % EDGE_RING);
  if (next != ringTail) {
    edgeRing[ringHead].us    = now;
    edgeRing[ringHead].pin   = pin;
    edgeRing[ringHead].level = level;
    edgeRing[ringHead].state = state;
    edgeRing[ringHead].step  = step;
    ringHead = next;
  } else {
    ringLost++;
  }
}

static IRAM_ATTR void isrEnc() {
  const uint8_t a = (uint8_t)digitalRead(PIN_ENCA);
  const uint8_t b = (uint8_t)digitalRead(PIN_ENCB);
  const uint8_t curr = (uint8_t)((a << 1) | b);
  const uint8_t prev = (uint8_t)encPrev;
  const uint8_t changed = (uint8_t)(curr ^ prev);
  if (changed == 0) return;   // 中断晚到、毛刺已自愈，不记

  const int8_t step = ENC_TABLE[(prev << 2) | curr];
  if (step == 0) invalidSteps++;   // 两相同时变或非法中间态（快速转动 / 抖动）

  // 两相里的哪一位变了，就记到哪一相。两相同时变时按 A 记一次，step 已经由查表给出。
  const uint8_t pin = (changed & 0x2) ? EP_A : EP_B;
  pushEdge(pin, (changed & 0x2) ? a : b, curr, step);

  encPrev  = (int8_t)curr;
  encAccum += step;
  while (encAccum >= 4) { encAccum -= 4; encDetent++; }   // 满 4 个正交跳变算一格，与正式固件一致
  while (encAccum <= -4) { encAccum += 4; encDetent--; }
}

static IRAM_ATTR void isrEncSw() {
  pushEdge(EP_SW, (uint8_t)digitalRead(PIN_ENCSW), 0, 0);
}

// ============================ 输出 ============================
static const char *levelText(uint8_t level) { return level ? "高" : "低"; }

// 【必须分开做】ESP32 Arduino 的 analogReadMilliVolts() 会把引脚 IO MUX 切到 ADC 功能，而且不会自己切回；
// 之后同一脚上的 digitalRead() 会恒读到 0，直到再调一次 pinMode() 把它选回 GPIO。
// 所以这里先量电压、再 pinMode 恢复、最后才做数字采样；两件事绝不交替进行。
static uint32_t adcSampleMv(uint8_t pin, uint8_t mode) {
  pinMode(pin, mode);
  delay(5);
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 8; i++) {
    sum += (uint32_t)analogReadMilliVolts(pin);
    delay(2);
  }
  pinMode(pin, mode);   // 把数字输入功能还回来
  delay(5);
  return sum / 8;
}

// 纯数字采样：只数高低，不碰 ADC。返回最后一次电平，同时给出高电平占了多少次。
static uint8_t sampleLevel(uint8_t pin, uint32_t ms, uint16_t *highCount, uint16_t *total) {
  uint16_t hi = 0, n = 0;
  uint8_t level = 0;
  const uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    level = (uint8_t)digitalRead(pin);
    if (level) hi++;
    n++;
    delayMicroseconds(500);
  }
  *highCount = hi;
  *total = n;
  return level;
}

// 静态体检：同一个脚换三种上下拉再读，靠「PULLDOWN 下还高不高」判断外部上拉在不在。
static void staticCheck(uint8_t pin, const char *name) {
  // 1) INPUT：只靠板上 4.7k 外部上拉
  const uint32_t mvInput = adcSampleMv(pin, INPUT);
  uint16_t hiIn = 0, nIn = 0;
  const uint8_t lvInput = sampleLevel(pin, 80, &hiIn, &nIn);

  // 2) INPUT_PULLDOWN：内部约 45k 下拉，外部 4.7k 上拉若在位，分压后仍约 3.0V
  const uint32_t mvDown = adcSampleMv(pin, INPUT_PULLDOWN);
  uint16_t hiDn = 0, nDn = 0;
  const uint8_t lvDown = sampleLevel(pin, 80, &hiDn, &nDn);

  // 3) INPUT_PULLUP：只用来确认脚本身没坏
  uint16_t hiUp = 0, nUp = 0;
  pinMode(pin, INPUT_PULLUP);
  delay(10);
  const uint8_t lvUp = sampleLevel(pin, 40, &hiUp, &nUp);

  const bool bouncing = (hiIn != 0 && hiIn != nIn);

  const char *verdict;
  if (bouncing) {
    verdict = "静止时高低混跳：接触不良 / 干扰，先查焊点与走线";
  } else if (lvInput == 0 && mvInput < 500) {
    verdict = "静止即为低且电压确实低：触点常闭、线路短到 GND，或上拉缺失且被内部下拉拉住";
  } else if (lvInput == 1 && lvDown == 1) {
    verdict = "外部 4.7k 上拉在位，线路高阻正常";
  } else if (lvInput == 1 && lvDown == 0) {
    verdict = "高阻悬空：外部上拉缺失 / 走线断开（内部下拉一开就掉）";
  } else if (lvInput == 0 && mvInput >= 500) {
    verdict = "数字读低但电压不低：仍是 ADC 未恢复或脚被复用，读数不可信";
  } else {
    verdict = "读数不一致，需结合电压人工判断";
  }

  Serial.printf("  %-10s INPUT=%s(%u/%u高, %lumV) | PULLDOWN=%s(%u/%u高, %lumV) | PULLUP=%s  → %s\n",
                name,
                levelText(lvInput), (unsigned)hiIn, (unsigned)nIn, (unsigned long)mvInput,
                levelText(lvDown), (unsigned)hiDn, (unsigned)nDn, (unsigned long)mvDown,
                levelText(lvUp), verdict);

  pinMode(pin, INPUT);
}

static void staticReport() {
  Serial.println();
  Serial.println("========== 一、静态体检（此刻不要碰编码器） ==========");
  Serial.println("  判据：4.7k 上拉到 3.3V 时静止应为高；内部下拉一开就掉说明上拉/走线不在。");
  for (uint8_t i = 0; i < EP_COUNT; i++) staticCheck(PROBE_PINS[i], PROBE_NAMES[i]);
  Serial.println("  注：按压脚静止为高，按下去应为低并出现 [EDGE] SW 行。");
  Serial.println("========== 二、动态监视开始（慢慢转 3~5 格，再快转，最后按几下） ==========");
  Serial.printf("  基线 A/B 状态 = %d%d，方向 dir 正负只反映接线次序，不代表故障。\n",
                (int)digitalRead(PIN_ENCA), (int)digitalRead(PIN_ENCB));
  Serial.println();
}

// 从环形缓冲取出边沿并打印。为了不把「另一相状态」误报成边沿，A / B 的事件里只打印
// 真正变化的那一位：比较本次 state 与上一条 A/B 事件的 state。
static void printEdges() {
  static uint8_t lastAB = 0xFF;   // 上一条 A/B 事件后的两相状态
  while (ringTail != ringHead) {
    const uint16_t idx = ringTail;
    EdgeEvent ev;
    ev.us    = edgeRing[idx].us;
    ev.pin   = edgeRing[idx].pin;
    ev.level = edgeRing[idx].level;
    ev.state = edgeRing[idx].state;
    ev.step  = edgeRing[idx].step;
    ringTail = (uint16_t)((idx + 1) % EDGE_RING);

    if (ev.pin == EP_SW) {
      if (printedEdges < EDGE_PRINT_MAX) {
        Serial.printf("[EDGE] %8.3fs SW→%s  (%s)\n",
                      ev.us / 1000000.0, ev.level ? "高(松开)" : "低(按下)",
                      ev.level ? "release" : "press");
        printedEdges++;
      }
      continue;
    }

    if (lastAB != 0xFF && ev.state == lastAB) continue;   // 两相都没变，属重复记账
    const uint8_t changedPin = (lastAB == 0xFF) ? ev.pin
                              : ((ev.state ^ lastAB) & 0x2) ? EP_A : EP_B;
    lastAB = ev.state;
    if (printedEdges >= EDGE_PRINT_MAX) {
      if (!edgeCapAnnounced) {
        Serial.printf("（边沿超过 %lu 条，后续只留 [STA] 汇总行）\n", (unsigned long)EDGE_PRINT_MAX);
        edgeCapAnnounced = true;
      }
      continue;
    }
    Serial.printf("[EDGE] %8.3fs %s→%d  状态 A=%d B=%d  step=%+d  累计跳变=%+ld  格=%ld\n",
                  ev.us / 1000000.0, PROBE_NAMES[changedPin], (int)ev.level,
                  (ev.state >> 1) & 1, ev.state & 1, (int)ev.step,
                  (long)encAccum, (long)encDetent);
    printedEdges++;
  }
}

// 动态阶段只用数字读数：这里再碰一次 ADC 就会把引脚切走，编码器立刻「不响应」——上一版探针就是这么骗人的。
static void statusLine() {
  const uint8_t a = (uint8_t)digitalRead(PIN_ENCA);
  const uint8_t b = (uint8_t)digitalRead(PIN_ENCB);
  const uint8_t sw = (uint8_t)digitalRead(PIN_ENCSW);

  Serial.printf("[STA] t=%7.2fs A=%d B=%d edges A=%lu B=%lu | acc=%+d detent=%ld invalid=%lu | "
                "SW=%d edges=%lu | minGap A=%luus B=%luus | 丢弃=%lu\n",
                millis() / 1000.0, a, b,
                (unsigned long)edgeCount[EP_A], (unsigned long)edgeCount[EP_B],
                (int)encAccum, (long)encDetent, (unsigned long)invalidSteps,
                sw, (unsigned long)edgeCount[EP_SW],
                (unsigned long)minGapUs[EP_A], (unsigned long)minGapUs[EP_B],
                (unsigned long)ringLost);
  statusCount++;
}

// ============================ 生命周期 ============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  analogSetPinAttenuation(PIN_ENCA, ADC_11db);
  analogSetPinAttenuation(PIN_ENCB, ADC_11db);
  analogSetPinAttenuation(PIN_ENCSW, ADC_11db);

  pinMode(PIN_ENCA, INPUT);
  pinMode(PIN_ENCB, INPUT);
  pinMode(PIN_ENCSW, INPUT);
  delay(10);

  Serial.println();
  Serial.println("============================================================");
  Serial.println(" 格物编码器探针  —— 只测编码器，不点灯、不碰 IMU");
  Serial.printf(" 编译于 %s %s\n", __DATE__, __TIME__);
  Serial.printf(" 引脚：A=GPIO%d  B=GPIO%d  按压=GPIO%d  逻辑 3.3V，上拉 4.7k，C 脚接 GND\n",
                PIN_ENCA, PIN_ENCB, PIN_ENCSW);
  Serial.println("============================================================");

  staticReport();

  attachInterrupt(digitalPinToInterrupt(PIN_ENCA), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCB), isrEnc, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCSW), isrEncSw, CHANGE);

  // 静态体检期间可能已有边沿（手碰到板子、上电抖动），清掉计数，从干净基线开始。
  noInterrupts();
  edgeCount[EP_A] = edgeCount[EP_B] = edgeCount[EP_SW] = 0;
  lastEdgeUs[EP_A] = lastEdgeUs[EP_B] = lastEdgeUs[EP_SW] = 0;
  minGapUs[EP_A] = minGapUs[EP_B] = minGapUs[EP_SW] = 0xFFFFFFFFu;
  encPrev = (int8_t)((digitalRead(PIN_ENCA) << 1) | digitalRead(PIN_ENCB));
  encAccum = 0;
  encDetent = 0;
  invalidSteps = 0;
  ringHead = ringTail = 0;
  ringLost = 0;
  interrupts();

  Serial.println(" 计数已清零，现在开始转编码器。");
}

void loop() {
  printEdges();

  static uint32_t lastStatusMs = 0;
  if (millis() - lastStatusMs >= STATUS_MS) {
    lastStatusMs = millis();
    statusLine();
  }
  delay(2);   // 主循环只负责搬运边沿，不用跑满
}
