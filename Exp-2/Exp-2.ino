#include <ESP32Servo.h>

const int touchPin = 4;
const int ledPin = 18;
const int servoPin = 17;

TaskHandle_t ledTaskHandle = NULL;
TaskHandle_t servoTaskHandle = NULL;

Servo servo;
const int fromPos = 0;
const int destPos = 180;
int curPos = 0;

const int touchThreshold = 40;  // 触摸阈值

volatile bool isPressing = false;
volatile bool isTouching = false;
volatile unsigned long touchStart = 0;

enum DoorState {
  CLOSED,
  OPENING,
  OPENED,
  CLOSING
};
volatile DoorState doorState = CLOSED;

float rotateInv = 20.0f;  // interval (ms)
const float minRotateInv = 1.0f;
const float maxRotateInv = 30.0f;
const int ledFlashInv = 100;

unsigned long closeTriggerTime = 0;

void ledTask(void* param) {
  while (true) {
    if (doorState == OPENING || doorState == CLOSING) {
      digitalWrite(ledPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(ledFlashInv));
      digitalWrite(ledPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(ledFlashInv));
    } else if (doorState == OPENED) {
      digitalWrite(ledPin, HIGH);
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {  // CLOSED
      digitalWrite(ledPin, LOW);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

void servoTask(void* param) {
  while (true) {
    if (doorState == OPENING) {
      unsigned long doorOpeningTime = millis();
      for (curPos = fromPos; curPos <= destPos; curPos++) {
        servo.write(curPos);

        if (isPressing) {
          unsigned long pressTime = millis() - touchStart;
          if (pressTime < 3000) {
            rotateInv = maxRotateInv - pressTime * (maxRotateInv - minRotateInv) / 3000.0f;
            //            float reduceFactor = (pressTime - 1000.0f) / 2000.0f;
            //            float reduction = reduceFactor * reduceFactor * (maxRotateInv - minRotateInv);
            //            rotateInv -= reduction;
          } else {
            rotateInv = minRotateInv;
          }
        }

        if (rotateInv < minRotateInv) {
          rotateInv = minRotateInv;
        }

        vTaskDelay(pdMS_TO_TICKS((TickType_t)roundf(rotateInv)));
      }

      doorState = OPENED;
      Serial.printf("Door opened within %d ms\n", millis() - doorOpeningTime);
      closeTriggerTime = millis() + 2500;
    } else if (doorState == CLOSING) {
      for (curPos = destPos; curPos >= fromPos; curPos--) {
        servo.write(curPos);
        vTaskDelay(pdMS_TO_TICKS((TickType_t)roundf(rotateInv)));
      }

      doorState = CLOSED;
      rotateInv = maxRotateInv;
      closeTriggerTime = 0;
    } else if (doorState == OPENED) {
      // 如果门是打开的
      if (isTouching) {
        // 如果此时被触摸，则重置关门计时器
        closeTriggerTime = millis() + 2500;
      } else if (millis() >= closeTriggerTime) {
        // 如果没有触摸，并且时间到了，则开始关门
        doorState = CLOSING;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  servo.attach(servoPin);
  servo.write(fromPos);

  xTaskCreatePinnedToCore(ledTask, "LED Task", 2048, NULL, 1, &ledTaskHandle, 1);
  xTaskCreatePinnedToCore(servoTask, "Servo Task", 4096, NULL, 1, &servoTaskHandle, 1);

  Serial.println("Ready");
}

void loop() {
  int touchVal = touchRead(touchPin);
  isTouching = (touchVal < touchThreshold);
  unsigned long now = millis();

  if (isTouching && !isPressing) {
    // 按下开始
    isPressing = true;
    touchStart = now;

    if (doorState == CLOSED) {
      doorState = OPENING;
      rotateInv = maxRotateInv;
      Serial.println("Touch Detected: Opening Door");
    }
  } else if (!isTouching && isPressing) {
    // 按下结束
    isPressing = false;
    unsigned long pressTime = now - touchStart;
    Serial.printf("Touch Released, time: %lu ms\n", pressTime);
  }

  vTaskDelay(pdMS_TO_TICKS(50));
}
