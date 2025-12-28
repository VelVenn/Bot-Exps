#include "Controller.h"
#include <iostream>
using namespace std;

Controller* controller = Controller::getInstance();

void turn(float degree)
{
    float Kp = 0.02; // 比例系数
    float Ki = 0.001; // 积分系数
    float Kd = 0.005; // 微分系数
    float integral = 0; // 积分项
    float prev_error = 0; // 上一次的误差
    float max_integral = 100; // 积分饱和上限

    float init = controller->getCarMessage().transform.rotation;
    auto startTime = chrono::steady_clock::now();

    while (true)
    {
        float current = controller->getCarMessage().transform.rotation;
        float error = degree - (current - init);

        if (error < -180)
            error += 360;
        else if (error > 180)
            error -= 360;

        if (fabs(error) < 1)
        {
            controller->setTwoTyres(0, 0);
            break; // 如果误差小于1，就停止
        }

        integral += error;

        // 防止积分饱和
        if (integral > max_integral)
            integral = max_integral;
        else if (integral < -max_integral)
            integral = -max_integral;
        float derivative = error - prev_error;
        float output = Kp * error + Ki * integral + Kd * derivative;

        if (output < 0)
            controller->setTwoTyres(0, -output);
        else
            controller->setTwoTyres(output, 0);

        prev_error = error;

        // 检查是否超时
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = currentTime - startTime;
        if (elapsed_seconds.count() >= 3.0) // 设置超时时间为5秒
        {
            std::cout << "Timeout reached." << std::endl;
            controller->setTwoTyres(-20, -20);
            break;
        }
    }
}

bool moveForward(float distance, float speed)
{
    // PID 控制参数
    float kp_distance = 1.0;      // 比例系数（距离）
    float ki_distance = 0.01;     // 积分系数（距离）
    float kd_distance = 0.03;     // 微分系数（距离）

    float kp_angle = 1.2;         // 比例系数（角度）
    float ki_angle = 0.005;       // 积分系数（角度）
    float kd_angle = 0.005;       // 微分系数（角度）

    float integral_distance = 0;  // 积分项（距离）
    float prev_error_distance = 0; // 上一次的误差（距离）
    float integral_angle = 0;     // 积分项（角度）
    float prev_error_angle = 0;   // 上一次的误差（角度）

    // 限制积分项最大值
    const float max_integral_distance = 10.0;
    const float max_integral_angle = 1.0;

    // 初始位置
    float initX = controller->getCarMessage().transform.position.x;
    float initY = controller->getCarMessage().transform.position.y;
    float initRotation = controller->getCarMessage().transform.rotation;

    // 目标方向角
    float targetAngle = atan2(distance, 1);  // 假设目标方向角为正前方

    int direct = 1;
    if (distance < 0)
        direct = -1;

    auto startTime = std::chrono::steady_clock::now();

    while (true)
    {
        // 当前位置
        float currentX = controller->getCarMessage().transform.position.x;
        float currentY = controller->getCarMessage().transform.position.y;
        float currentRotation = controller->getCarMessage().transform.rotation;

        // 计算当前位置到目标位置的距离
        float distanceError = distance - sqrt((currentX - initX) * (currentX - initX) + (currentY - initY) * (currentY - initY));

        // 计算当前位置的方向角
        float currentAngle = atan2(currentY - initY, currentX - initX);

        // 计算方向误差
        float angleError = targetAngle - currentAngle;
        if (angleError > M_PI)
            angleError -= 2 * M_PI;
        else if (angleError < -M_PI)
            angleError += 2 * M_PI;

        // 限制积分项
        integral_distance = std::min(max_integral_distance, std::max(-max_integral_distance, integral_distance + distanceError));
        integral_angle = std::min(max_integral_angle, std::max(-max_integral_angle, integral_angle + angleError));

        // PID 控制输出（距离）
        float derivative_distance = distanceError - prev_error_distance;
        float distanceOutput = kp_distance * distanceError + ki_distance * integral_distance + kd_distance * derivative_distance;

        // PID 控制输出（角度）
        float derivative_angle = angleError - prev_error_angle;
        float angleOutput = kp_angle * angleError + ki_angle * integral_angle + kd_angle * derivative_angle;

        // 调整车轮速度
        float leftSpeed = direct * (speed + distanceOutput - angleOutput);
        float rightSpeed = direct * (speed + distanceOutput + angleOutput);

        // 应用车轮速度
        controller->setTwoTyres(leftSpeed, rightSpeed);

        prev_error_distance = distanceError;
        prev_error_angle = angleError;

        // 检查是否到达目标位置
        if (fabs(distanceError) < 2)
        {
            controller->setTwoTyres(0, 0);
            std::cout << "Reached target position." << std::endl;
            return true;
        }

        // 检查是否超时
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = currentTime - startTime;
        if (elapsed_seconds.count() >= 5.0)  // 设置超时时间为5秒
        {
            std::cout << "Timeout reached." << std::endl;
            controller->setTwoTyres(-20, -20);
            Controller::delay(500);
            turn(30);
            return false;
        }
    }
}

bool find(const string& targetColor)
{
    float targetX = controller->getCamera().size.x / 2;
    float targetY = controller->getCamera().size.y / 2;
    cout << "targetX:" << targetX << endl;
    int index;
    if (targetColor == "blue")
        index = 0;
    else if (targetColor == "red")
        index = 1;
    else if (targetColor == "green")
        index = 2;
    else
        index = 3;
    bool isClose = false;
    while (true)
    {
        float curPosition = controller->getCamera().pos[index].x;
        cout << targetColor << ": " << curPosition << endl;
        float error = curPosition - targetX;


        if (fabs(error) < 40)
        {
            if(controller->getCamera().pos[index].y > targetY)
                isClose = true;
            return isClose;
        }

        if(error == -targetX)
        {
            turn(45);
            //cout << "巡逻" << endl;
        }
        else if (error > 0)
        {
            turn(10);
            //cout << "右转" << endl;
        }
        else if (error < 0)
        {
            turn(-10);
            //cout << "左转" << endl;
        }

        this_thread::sleep_for(chrono::milliseconds(300));
    }
}



void setup()
{
    //需要设置subscribeList 和 getList
    controller->subscribeList = "RobotBaseTransform,Camera";
    controller->getList = "Barrier,CheckPoint";//Barrier,CheckPoint
}


//---------------------------------------------用于推小球---------------------------------------
bool findRed = false;
bool findBlue = false;
bool findGreen = false;
bool findYellow = false;
//---------------------------------------------用于推小球---------------------------------------
void loop()
{


//---------------------------------------------用于推小球---------------------------------------
    if(!findBlue)
    {
        while(!find("blue"))
        {
            cout << "距离不足" << endl;
            controller->setTwoTyres(20,20);
            Controller::delay(3000);
            controller->setTwoTyres(0,0);
        }
        controller->setTwoTyres(20,20);
        Controller::delay(3000);
        controller->setTwoTyres(0,0);
        findBlue = true;
    }


    if(!findYellow)
    {
        int cnt = 1;
        if(!find("yellow"))
        {
            cout << "距离不足" << endl;
            cnt = 2;
        }
        for(int i=0; i<cnt; i++)
        {
            controller->setTwoTyres(20,20);
            Controller::delay(6000);
            controller->setTwoTyres(0,0);
            findYellow = true;
        }
    }
    if(!findGreen)
    {
        int cnt = 1;
        if(!find("green"))
        {
            cout << "距离不足" << endl;
            cnt = 2;
        }
        for(int i=0; i<cnt; i++)
        {
            controller->setTwoTyres(20,20);
            Controller::delay(6000);
            controller->setTwoTyres(0,0);
            findGreen = true;
        }
    }
    if(!findRed)
    {
        int cnt = 1;
        if(!find("red"))
            cnt = 2;
        for(int i=0; i<cnt; i++)
        {
            controller->setTwoTyres(20,20);
            Controller::delay(6000);
            controller->setTwoTyres(0,0);
            findRed = true;
        }
    }
    if(findYellow && findGreen && findBlue && findRed)
    {
        findYellow = false;
        findGreen = false;
        findBlue = false;
        findRed = false;
    }
//---------------------------------------------用于推小球---------------------------------------
}