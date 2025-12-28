// 扩展板1代码 - 发送端（修改后）
const int trigPin = 25;    // 超声波触发引脚
const int echoPin = 26;    // 超声波回波引脚
const int ledPin = 2;      // LED指示灯引脚

// ESP32硬件串口2（仅用TX发送数据）
HardwareSerial Board1Serial(2);

unsigned long lastSendTime = 0;
const int sendInterval = 100;  // 固定100ms发送一次（每秒10次）

// LED非阻塞控制变量
unsigned long lastLedChange = 0;  // 记录LED最后一次状态变化时间
bool ledState = LOW;              // 当前LED状态
int blinkInterval = 0;            // 闪烁周期（ms），根据距离动态变化

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(ledPin, OUTPUT);
  
  Board1Serial.begin(9600, SERIAL_8N1, 16, 17);  // RX=16（未用）, TX=17
  digitalWrite(ledPin, LOW);
}

void loop() {
  unsigned long currentTime = millis();

  // 固定间隔发送数据（不受LED影响，每秒10次）
  if (currentTime - lastSendTime >= sendInterval) {
    lastSendTime = currentTime;
    float distance = getDistance();
    sendDataToBoard2(distance);
    updateBlinkInterval(distance);  // 根据距离更新LED闪烁周期
  }

  // 非阻塞控制LED闪烁（独立于数据发送节奏）
  if (currentTime - lastLedChange >= blinkInterval / 2) {  // 周期的一半切换一次状态
    lastLedChange = currentTime;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
  }
}

// 超声波测距（不变）
float getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 30000);  // 30ms超时
  float distance = duration * 0.034 / 2;
  
  if (distance > 400 || distance < 2 || duration == 0) {
    return 0;
  }
  return distance;
}

// 发送数据（不变）
void sendDataToBoard2(float dist) {
  Board1Serial.println(dist);
}

// 根据距离更新LED闪烁周期（核心修改）
void updateBlinkInterval(float dist) {
  if (dist <= 0 || dist > 400) {  // 无效距离：中等频率闪烁
    blinkInterval = 1000;  // 周期1000ms（亮500ms，灭500ms）
  } else if (dist <= 10) {  // 近距离：快速闪烁
    blinkInterval = 100;   // 周期100ms（亮50ms，灭50ms）
  } else if (dist <= 30) {  // 中近距离：中速闪烁
    blinkInterval = 400;   // 周期400ms（亮200ms，灭200ms）
  } else if (dist <= 60) {  // 中远距离：慢速闪烁
    blinkInterval = 800;   // 周期800ms（亮400ms，灭400ms）
  } else {  // 远距离：常亮（周期设为0，通过特殊处理保持常亮）
    blinkInterval = 0;
    digitalWrite(ledPin, HIGH);  // 强制常亮
  }
}
