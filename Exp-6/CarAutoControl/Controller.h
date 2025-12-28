#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <iostream>
#include <string>
#include <vector>
#include <thread>
#include <sstream>
#include <vector>
#include <ctime>
#include <cmath>
#include <sys/timeb.h>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <functional>
//#pragma comment(lib,"ws2_32.lib")

using std::function;
using std::string;
using std::thread;
using std::vector;
using std::mutex;
using std::queue;

#define MAXSIZE_BUF 1024

using CallbackType = function<void()>;

struct Vector2D
{
    float x;
    float y;
};
struct Vector3D
{
    float x;
    float y;
    float z;
};
struct CarTransform
{
    Vector2D position;
    float rotation;
};
struct Barrier
{
    string type;
    Vector2D position;
    float rotation;
    float Length;//长
    float width;//宽,不一定有
    Vector2D point1, point2, point3, point4;
};
struct CheckPoint
{
    int number;
    string type;
    Vector2D position;
    float rotation;
    float size;
};
struct Car
{
    float LSpeed;
    float RSpeed;
    CarTransform transform;//小车变换
};
struct IMU
{
    Vector3D Acceleration;//加速度
    Vector3D Gyroscope;//陀螺仪
    Vector3D Direction;//三轴角度
};
/*
红外模块，未完成
struct IRS
{

    bool flag;//是否检测的物体
    float distance;//与物体的距离
};*/
struct Camera
{
    Vector2D size;
    string color[4];
    Vector2D pos[4];
    float area[4];
};

void setup();
void loop();

class Controller
{
private:
    thread receiveThread;   //接收线程
    vector<thread> threads; //处理线程组
    char sendBuf[MAXSIZE_BUF];  //发送数据缓冲区
    char recvBuf[MAXSIZE_BUF];  //接收数据缓冲区

    //模块变量
    Car player;
    IMU imu;
    //IRS irs;
    Camera camera;
    vector<Barrier> obstacles;
    vector<CheckPoint> targets;

    //回调函数
    CallbackType callback_Car;
    CallbackType callback_IMU;
    //CallbackType callback_IRS;
    CallbackType callback_Camera;
    CallbackType callback_Barrier;
    CallbackType callback_CheckPoint;

    mutex player_mtx;	//用于保护player的互斥信号量
    mutex barrier_mtx;		//用于保护obstacles的互斥信号量
    mutex checkPoint_mtx;		//用于保护targets的互斥信号量
    //mutex irs_mtx;		//用于保护irs的互斥信号量
    mutex imu_mtx;		//用于保护imu的互斥信号量
    mutex camera_mtx;	//用于保护camera的互斥信号量

    static Controller* instance;

    Controller();
    ~Controller();

    int publish(const string& str); //发布信息
    int receive();                  //接收信息
    void handle();                  //处理接收的信息
    bool subScribe(const string& topicName);    //订阅话题,返回是否发送成功
    bool get(const string& topicName);          //订阅一次性话题,返回是否发送成功

public:
    string subscribeList;
    string getList;
    static Controller* getInstance()
    {
        if (instance == nullptr)
            instance = new Controller();
        return instance;
    }

    static void delay(int milliseconds);
    void run(); //启动
    void setTwoTyres(float LSpeed, float RSpeed);


    [[nodiscard]] const vector<Barrier>& getBarrierMessage() const;
    [[nodiscard]] const vector<CheckPoint>& getCheckPointMessage() const;
    [[nodiscard]] Car getCarMessage() const;

    //IRS getIRS() const;
    //void showIRS() const;
    [[nodiscard]] IMU getIMU() const;

    [[nodiscard]] Camera getCamera() const;

    void on_IMU_Message_Update(CallbackType cb);
    //void on_IRS_Message_Update(CallbackType cb);
    void on_Camera_Update(CallbackType cb);
    void on_Car_Message_Update(CallbackType cb);
    void on_Barrier_Message_receive(CallbackType cb);
    void on_CheckPoint_Message_receive(CallbackType cb);
};

#endif // CONTROLLER_H

