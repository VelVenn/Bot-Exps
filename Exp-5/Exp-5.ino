#define ENCODER_L 12 //8号引脚
#define DIRECTION_L 13 //9号引脚
#define ENCODER_R 25 //3号引脚
#define DIRECTION_R 26  //2号引脚
#define interrupt_time 20 // 中断时间
#define L_DIR 14   //7号引脚
#define L_SPEED 16  //5号引脚
#define R_DIR 17   //4号引脚
#define R_SPEED 27  //6号引脚

#define TARGET_L 1500.0
#define TARGET_R 1500.0
//ALPHA（建议在100-500之间）值越小和MAXINC（建议不越过10）越大，速度调节越快，太快容易转速调节过头
#define ALPHA 200.0
#define MAXINC 8.0
//轮子启动初始量，使轮子快束起动
#define BOOST 80.0

#include <Arduino.h>
#include<Ticker.h>
Ticker timer_read_encoder; // 中断函数定时器定义

int32_t Velocity_L, Velocity_R; //左右轮编码器数据
int16_t Velocity_Left, Velocity_Right; //左右轮速度
void READ_ENCODER_L(void);
void READ_ENCODER_R(void);
void read(void);

void READ_ENCODER_L(void) {
  if (digitalRead(ENCODER_L) == LOW) {
    //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)
      Velocity_L--;
    //根据另外一相电平判定方向
    else Velocity_L++;
  } else { //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_L) == LOW)
      Velocity_L++; //根据另外一相电平判定方向
    else Velocity_L--;
  }
}
void READ_ENCODER_R(void) {
  if (digitalRead(ENCODER_R) == LOW) { //如果是下降沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW)
      Velocity_R--;//根据另外一相电平判定方向
    else Velocity_R++;
  } else { //如果是上升沿触发的中断
    if (digitalRead(DIRECTION_R) == LOW) Velocity_R++; //根据另外一相电平判定方向
    else Velocity_R--;
  }
}
void read(void) {
  Velocity_Left = Velocity_L * 2000 / interrupt_time;
  Velocity_L = 0; //读取左轮编码器数据，并清零，这就是通过M法测速（单位时间内的脉冲数）得到速度。
  Velocity_Right = Velocity_R * 2000 / interrupt_time;
  Velocity_R = 0; //读取右轮编码器数据，并清零
}

float l_speed=0, r_speed=0;
void taskOne(void *pvParameters)
{
  int i;
  while (1) {
    l_speed = 0;r_speed = 0;
    for (i = 0; i < 80; i++) {
      digitalWrite(L_DIR, 1);
      digitalWrite(R_DIR, 1);
      analogWrite(L_SPEED, l_speed);
      analogWrite(R_SPEED, r_speed);
      if (i==10){l_speed = BOOST;r_speed = BOOST;}
      else if (i > 10)
      {
        if (Velocity_Left < (TARGET_L-50)) l_speed = l_speed + ((TARGET_L-Velocity_Left)/ALPHA>MAXINC?MAXINC:(TARGET_L-Velocity_Left)/ALPHA);
        else if (Velocity_Left > (TARGET_L+50)) l_speed = l_speed - ((Velocity_Left-TARGET_L)/ALPHA>MAXINC?MAXINC:(Velocity_Left-TARGET_L)/ALPHA);
        if (Velocity_Right < (TARGET_R-50)) r_speed = r_speed + ((TARGET_R-Velocity_Right)/ALPHA>MAXINC?MAXINC:(TARGET_R-Velocity_Right)/ALPHA);
        else if (Velocity_Right > (TARGET_R+50)) r_speed = r_speed - ((Velocity_Right-TARGET_R)/ALPHA>MAXINC?MAXINC:(Velocity_Right-TARGET_R)/ALPHA);
        if(l_speed>255) l_speed = 255;
        if(r_speed>255) r_speed = 255;
      }
      delay(50);
    }

    l_speed = 0;r_speed = 0;
    for (i = 0; i < 80; i++) {
      digitalWrite(L_DIR, 0);
      digitalWrite(R_DIR, 1);
      analogWrite(L_SPEED, l_speed);
      analogWrite(R_SPEED, r_speed);
      if (i==10){l_speed = BOOST;r_speed = BOOST;}
      else if (i > 10)
      {
        if (Velocity_Left > -(TARGET_L-50)) l_speed = l_speed + ((Velocity_Left+TARGET_L)/ALPHA>MAXINC?MAXINC:(Velocity_Left+TARGET_L)/ALPHA);
        else if (Velocity_Left < -(TARGET_L+50)) l_speed = l_speed - ((-TARGET_L-Velocity_Left)/ALPHA>MAXINC?MAXINC:(-TARGET_L-Velocity_Left)/ALPHA);
        if (Velocity_Right < (TARGET_R-50)) r_speed = r_speed + ((TARGET_R-Velocity_Right)/ALPHA>MAXINC?MAXINC:(TARGET_R-Velocity_Right)/ALPHA);
        else if (Velocity_Right > (TARGET_R+50)) r_speed = r_speed - ((Velocity_Right-TARGET_R)/ALPHA>MAXINC?MAXINC:(Velocity_Right-TARGET_R)/ALPHA);
        if(l_speed>255) l_speed = 255;
        if(r_speed>255) r_speed = 255;
      }
      delay(50);
    }
    l_speed = 0;r_speed = 0;
    for (i = 0; i < 80; i++) {
      digitalWrite(L_DIR, 0);
      digitalWrite(R_DIR, 0);
      analogWrite(L_SPEED, l_speed);
      analogWrite(R_SPEED, r_speed);
      if (i==10){l_speed = BOOST;r_speed = BOOST;}
      else if (i > 10)
      {
        if (Velocity_Left > -(TARGET_L-50)) l_speed = l_speed + ((Velocity_Left+TARGET_L)/ALPHA>MAXINC?MAXINC:(Velocity_Left+TARGET_L)/ALPHA);
        else if (Velocity_Left < -(TARGET_L+50)) l_speed = l_speed - ((-TARGET_L-Velocity_Left)/ALPHA>MAXINC?MAXINC:(-TARGET_L-Velocity_Left)/ALPHA);
        if (Velocity_Right > -(TARGET_R-50)) r_speed = r_speed + ((Velocity_Right+TARGET_R)/ALPHA>MAXINC?MAXINC:(Velocity_Right+TARGET_R)/ALPHA);
        else if (Velocity_Right < -(TARGET_R+50)) r_speed = r_speed - ((-TARGET_R-Velocity_Right)/ALPHA>MAXINC?MAXINC:(-TARGET_R-Velocity_Right)/ALPHA);
        if(l_speed>255) l_speed = 255;
        if(r_speed>255) r_speed = 255;
      }
      delay(50);
    }
    l_speed = 0;r_speed = 0;
    for (i = 0; i < 80; i++) {
      digitalWrite(L_DIR, 1);
      digitalWrite(R_DIR, 0);
      analogWrite(L_SPEED, l_speed);
      analogWrite(R_SPEED, r_speed);
      if (i==10){l_speed = BOOST;r_speed = BOOST;}
      else if (i > 10)
      {
        if (Velocity_Left < (TARGET_L-50)) l_speed = l_speed + ((TARGET_L-Velocity_Left)/ALPHA>MAXINC?MAXINC:(TARGET_L-Velocity_Left)/ALPHA);
        else if (Velocity_Left > (TARGET_L+50)) l_speed = l_speed - ((Velocity_Left-TARGET_L)/ALPHA>MAXINC?MAXINC:(Velocity_Left-TARGET_L)/ALPHA);
        if (Velocity_Right > -(TARGET_R-50)) r_speed = r_speed + ((Velocity_Right+TARGET_R)/ALPHA>MAXINC?MAXINC:(Velocity_Right+TARGET_R)/ALPHA);
        else if (Velocity_Right < -(TARGET_R+50)) r_speed = r_speed - ((-TARGET_R-Velocity_Right)/ALPHA>MAXINC?MAXINC:(-TARGET_R-Velocity_Right)/ALPHA);
        if(l_speed>255) l_speed = 255;
        if(r_speed>255) r_speed = 255;
      }
      delay(50);
    }

  }
}

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_L, INPUT); //编码器引脚 输入模式
  pinMode(ENCODER_R, INPUT); //编码器引脚 输入模式
  pinMode(DIRECTION_L, INPUT); //编码器引脚 输入模式
  pinMode(DIRECTION_R, INPUT); //编码器引脚 输入模式
  pinMode(L_DIR, OUTPUT);
  pinMode(L_SPEED, OUTPUT);
  pinMode(R_DIR, OUTPUT);
  pinMode(R_SPEED, OUTPUT);
  //编码器接口1 开启外部跳边沿中断
  attachInterrupt(ENCODER_L, READ_ENCODER_L, CHANGE);
  //中断函数READ_ENCODER_L
  //编码器接口2 开启外部跳边沿中断
  attachInterrupt(ENCODER_R, READ_ENCODER_R, CHANGE);
  //中断函数READ_ENCODER_R
  interrupts();
  //打开外部中断
  timer_read_encoder.attach_ms(interrupt_time, read);

  xTaskCreatePinnedToCore(
    taskOne, /* Task function. */
    "TaskOne", /* String with name of task. */
    1000, /* Stack size in bytes. */
    NULL, /* Parameter passed as input of the task */
    1, /* Priority of the task. */
    NULL, /* Task handle. */
    1);
}



void loop() {
  Serial.print("Velocity_L:");
  //Serial.print(Velocity_L);
  Serial.print(l_speed);
  Serial.print(" | Velocity_Left:");
  Serial.print(Velocity_Left);
  Serial.print(" ----- Velocity_R:");
  //Serial.print(Velocity_R);
  Serial.print(r_speed);
  Serial.print(" | Velocity_Right:");          
  Serial.println(Velocity_Right);
}