#define BLINKER_WIFI
#include <Blinker.h>
char auth[] = "3afc964ba552";      // 设备密钥
char ssid[] = "iQOOneo10";        // WIFI热点
char pswd[] = "99999999";    // WIFI密码
#define LED_PIN 2            // LED引脚

// 新建组件对象
BlinkerButton Button1("btn-abc");
BlinkerNumber Number1("num-abc");
int counter = 0;

// 按下按键即会执行该函数
void button1_callback(const String &state)
{
  BLINKER_LOG("get button state: ", state);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // 翻转LED灯状态
}
// 如果未绑定的组件被触发，则会执行其中内容
void dataRead(const String &data)
{
  BLINKER_LOG("Blinker readString: ", data);
  counter++;
  Number1.print(counter);
}

void setup()
{
  // 初始化串口
  Serial.begin(115200);
  BLINKER_DEBUG.stream(Serial);
  BLINKER_DEBUG.debugAll();

  // 初始化有LED的IO
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // 初始化blinker
  Blinker.begin(auth, ssid, pswd);
  Blinker.attachData(dataRead);

  Button1.attach(button1_callback);
}

void loop() {
  Blinker.run();
}
