#define BLINKER_BLE
#define LSPEED 16  // 左轮速度引脚
#define LDIREC 14  // 左轮方向引脚
#define RSPEED 27  // 右轮速度引脚
#define RDIREC 17  // 右轮方向引脚
#define JOY 0
#define PRESS 1
#include <Blinker.h>  // 引入 Blinker 库
// 模式按钮
BlinkerButton Button("PATTERN");
// 启动按钮
BlinkerButton ButtonL("StartL");
BlinkerButton ButtonR("StartR");
// PWM 滑块
#define BALANCE_RANGE 100
BlinkerSlider SliderL("Balance");  // 微调左右轮的差异，使左右轮能走直线
BlinkerSlider SliderR("PWM");      // 值越大电机最大转速就越大
// 方向按钮
BlinkerButton ButtonZ("DirecL");
BlinkerButton ButtonY("DirecR");
// 摇杆
BlinkerJoystick Joystick("joy");
// PWM 参数
int channel_L = 0;         // 左轮 PWM 通道
int channel_R = 1;         // 右轮 PWM 通道
int freq = 500;            // PWM 频率
int resolution_bits = 10;  // 0~1023
int PWM_L = 223;           // 记录 左轮
int PWM_R = 223;           // 记录 右轮
// 模式[1 长按模式][0 摇杆模式]
int pattern = JOY;  // 两种模式，摇杆模式， 按键模式
int balance = 50;   // 居中
int pwmValue = 0;

int lastYDirection = -1;

// 回调函数
void Joystick_callback(uint8_t xAxis, uint8_t yAxis) {
	if (pattern == JOY) {
#define DEADZONE_Y 15  // Y轴死区
#define DEADZONE_X 10  // X轴死区（可选）

		// 完全归位才停止
		if (xAxis == 128 && yAxis == 128) {
			ledcWrite(channel_L, 0);
			ledcWrite(channel_R, 0);
			lastYDirection = -1;  // 重置方向状态
			return;
		}

		float distance = sqrt((xAxis - 128) * (xAxis - 128) + (yAxis - 128) * (yAxis - 128));

		// 确定当前Y轴方向
		int currentYDirection;
		if (yAxis < 128 - DEADZONE_Y) {
			currentYDirection = 1;  // 明确前进
			lastYDirection = 1;
		} else if (yAxis > 128 + DEADZONE_Y) {
			currentYDirection = -1;  // 明确后退
			lastYDirection = -1;
		} else {
			// Y轴在死区内，保持上一次的方向
			currentYDirection = lastYDirection;
		}

		// 根据方向控制
		if (currentYDirection == 1) {  // 前进
			digitalWrite(LDIREC, HIGH);
			digitalWrite(RDIREC, HIGH);
			int ySpeed = abs(128 - yAxis);  // Y轴速度分量

			if (xAxis < 128 - DEADZONE_X) {
				ledcWrite(channel_L, ySpeed * PWM_L / 128);
				ledcWrite(channel_R, PWM_R * distance / 128);
			} else if (xAxis > 128 + DEADZONE_X) {
				ledcWrite(channel_L, PWM_L * distance / 128);
				ledcWrite(channel_R, ySpeed * PWM_R / 128);
			} else {
				ledcWrite(channel_L, ySpeed * PWM_L / 128);
				ledcWrite(channel_R, ySpeed * PWM_R / 128);
			}
		} else if (currentYDirection == -1) {  // 后退
			digitalWrite(LDIREC, LOW);
			digitalWrite(RDIREC, LOW);
			int ySpeed = abs(yAxis - 128);  // Y轴速度分量

			if (xAxis < 128 - DEADZONE_X) {
				ledcWrite(channel_L, ySpeed * PWM_L / 128);
				ledcWrite(channel_R, PWM_R * distance / 128);
			} else if (xAxis > 128 + DEADZONE_X) {
				ledcWrite(channel_L, PWM_L * distance / 128);
				ledcWrite(channel_R, ySpeed * PWM_R / 128);
			} else {
				ledcWrite(channel_L, ySpeed * PWM_L / 128);
				ledcWrite(channel_R, ySpeed * PWM_R / 128);
			}
		} else {
			// lastYDirection == 0，还没确定方向，停止
			ledcWrite(channel_L, 0);
			ledcWrite(channel_R, 0);
		}
	}
}

void Button_callback(const String& state) {
	pattern = (pattern + 1) % 2;
	if (pattern == JOY) {
		Button.text("JOY");
		Button.print();
	} else if (pattern == PRESS) {
		Button.text("PRESS");
		Button.print();
	}
}
void ButtonL_callback(const String& state) {
	// 长按
	if (state == BLINKER_CMD_BUTTON_PRESSED) {
		ledcWrite(channel_L, PWM_L);
	}
	// 松开
	else {
		ledcWrite(channel_L, 0);
	}
}
void ButtonR_callback(const String& state) {
	// 长按
	if (state == BLINKER_CMD_BUTTON_PRESSED) {
		ledcWrite(channel_R, PWM_R);
	}
	// 松开
	else {
		ledcWrite(channel_R, 0);
	}
}

void setPWM() {
	// 设置 PWM_L
	PWM_L = 223 + pwmValue * 8 + BALANCE_RANGE * (balance - 50) / 50;
	if (PWM_L > 1023)
		PWM_L = 1023;
	// 设置 PWM_R
	PWM_R = 223 + pwmValue * 8;
}

void SliderL_callback(int32_t value) {
	balance = value;
	setPWM();
}

void SliderR_callback(int32_t value) {
	pwmValue = value;
	setPWM();
}

void ButtonZ_callback(const String& state) {
	// 设置 LDIREC，改变按钮文本
	digitalWrite(LDIREC, !digitalRead(LDIREC));
	if (digitalRead(LDIREC)) {
		ButtonZ.text("F");
		ButtonZ.print();
	} else {
		ButtonZ.text("B");
		ButtonZ.print();
	}
}

void ButtonY_callback(const String& state) {
	// 设置 LDIREC，改变按钮文本
	digitalWrite(RDIREC, !digitalRead(RDIREC));
	if (digitalRead(RDIREC)) {
		ButtonY.text("F");
		ButtonY.print();
	} else {
		ButtonY.text("B");
		ButtonY.print();
	}
}

void setup() {
	Serial.begin(115200);
	BLINKER_DEBUG.stream(Serial);
	pinMode(LDIREC, OUTPUT);  // 左轮方向引脚
	pinMode(RDIREC, OUTPUT);  // 右轮方向引脚

	// 设置向前
	digitalWrite(LDIREC, HIGH);
	digitalWrite(RDIREC, HIGH);

	// 设置 PWM 通道 并连接对应引脚
	ledcSetup(channel_L, freq, resolution_bits);  // 设置左轮 PWM 通道
	ledcSetup(channel_R, freq, resolution_bits);  // 设置右轮 PWM 通道

	// 将通道与对应的引脚连接
	ledcAttachPin(LSPEED, channel_L);
	ledcAttachPin(RSPEED, channel_R);

	Blinker.begin();
	// 左轮控制按钮
	ButtonL.attach(ButtonL_callback);
	ButtonZ.attach(ButtonZ_callback);
	// 右轮控制按钮
	ButtonR.attach(ButtonR_callback);

	SliderL.attach(SliderL_callback);
	SliderR.attach(SliderR_callback);

	ButtonY.attach(ButtonY_callback);
	// 模式按钮
	Button.attach(Button_callback);
	// 摇杆
	Joystick.attach(Joystick_callback);
}
void loop() {
	Blinker.run();
}
