// 扩展板2代码 - 接收端（控制扩展板上的小灯）
const int rxPin = 16;         // 接收扩展板1的数据（对应串口2的RX）
const int extLedPin = 2;      // 扩展板上的LED引脚（根据实际硬件调整）
int dataCount = 0;

// ESP32硬件串口2（RX=16，TX=17），仅用RX接收数据
HardwareSerial Board2Serial(2);

// LED非阻塞控制变量
unsigned long lastLedChange = 0;
bool ledState = LOW;
int blinkInterval = 1000;     // 默认闪烁周期（无效距离时）

void setup() {
  Serial.begin(115200);       // 连接电脑的串口（用于显示）
  pinMode(extLedPin, OUTPUT); // 初始化扩展板LED引脚为输出
  
  // 初始化硬件串口2（波特率9600，与扩展板1一致）
  Board2Serial.begin(9600, SERIAL_8N1, rxPin, 17);
  
  Serial.println("扩展板2启动，开始接收数据...");
  digitalWrite(extLedPin, LOW); // 初始关闭LED
}

void loop() {
  unsigned long currentTime = millis();

  // 接收扩展板1发送的数据
  if (Board2Serial.available() > 0) {
    String distStr = Board2Serial.readStringUntil('\n');
    float distance = distStr.toFloat();
    
    // 数据计数和显示
    dataCount++;
    Serial.print("数据");
    Serial.print(dataCount);
    Serial.print(": ");
    if (distance <= 0 || distance > 400) {
      Serial.println("无效距离");
    } else {
      Serial.print(distance);
      Serial.println(" cm");
    }
    
    // 根据距离更新LED闪烁周期（与发送端逻辑一致）
    updateBlinkInterval(distance);
  }

  // 非阻塞控制扩展板LED闪烁
  if (blinkInterval > 0) { // 若有闪烁周期，则按周期切换状态
    if (currentTime - lastLedChange >= blinkInterval / 2) {
      lastLedChange = currentTime;
      ledState = !ledState;
      digitalWrite(extLedPin, ledState);
    }
  } else { // 周期为0时，强制常亮
    digitalWrite(extLedPin, HIGH);
  }
}

// 根据距离更新LED闪烁周期（与发送端逻辑一致）
void updateBlinkInterval(float dist) {
  if (dist <= 0 || dist > 400) {  // 无效距离：中等频率闪烁
    blinkInterval = 1000;  // 周期1000ms（亮500ms，灭500ms）
  } else if (dist <= 10) {  // 近距离：快速闪烁
    blinkInterval = 100;   // 周期100ms（亮50ms，灭50ms）
  } else if (dist <= 30) {  // 中近距离：中速闪烁
    blinkInterval = 400;   // 周期400ms（亮200ms，灭200ms）
  } else if (dist <= 60) {  // 中远距离：慢速闪烁
    blinkInterval = 800;   // 周期800ms（亮400ms，灭400ms）
  } else {  // 远距离：常亮
    blinkInterval = 0;
  }
}
