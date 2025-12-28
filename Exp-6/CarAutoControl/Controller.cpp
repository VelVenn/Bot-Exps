#define _WIN32_WINNT 0x0600
#include "Controller.h"
#include <winsock2.h>
#include <WS2tcpip.h>
#include <utility>

using namespace std;

SOCKET toServerSocket;//用于与服务器通信的套接字
sockaddr_in serverAddr;//发信息到的服务器地址

queue<string> data_queue;
condition_variable cv;
mutex queue_mtx;

bool startFlag = false;

/**
 * 分割字符串。
 *
 * 该函数将一个字符串按照指定的分隔符进行分割，并将分割后的子字符串存储到一个vector中。
 *
 * @param str 要分割的原始字符串。
 * @param split 分隔符，用于指定哪里应该分割字符串。
 * @param res 一个vector，用于存储分割后的子字符串。
 */
void stringSplit(const string& str, const char split, vector<string>& res)
{
    // 如果原始字符串为空，则无需进行分割
    if (str.empty())
        return;

    // 将原始字符串与分隔符拼接，确保最后一个元素也会被处理
    string strs = str + split;

    // 查找第一个分隔符的位置
    size_t pos = strs.find(split);

    // 遍历字符串，直到找不到分隔符
    while (pos != strs.npos)
    {
        // 提取从字符串开始到分隔符位置的子字符串，并加入结果vector
        string temp = strs.substr(0, pos);
        res.push_back(temp);

        // 更新字符串，移除已处理的部分，包括分隔符
        strs = strs.substr(pos + 1, strs.length());

        // 继续查找下一个分隔符的位置
        pos = strs.find(split);
    }
}

Controller::Controller()
{
    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA wsaData;
    if (WSAStartup(sockVersion, &wsaData) != 0)
    {
        cout << "WSAStartup error:" << GetLastError() << endl;
        return;
    }

    /*toServerSocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr.S_un.S_addr);
    serverAddr.sin_port = htons(9979);*/

    toServerSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    inet_pton(AF_INET, "127.0.0.1", &serverAddr.sin_addr.S_un.S_addr);
    serverAddr.sin_port = htons(12344);
    int res = connect(toServerSocket, (sockaddr*)&serverAddr, sizeof(serverAddr));
    if(res == -1)
    {
        perror("链接服务器失败");
        return;
    }
}

Controller::~Controller()
{
    // 遍历线程向量，等待所有线程完成执行
    for (auto& thread : threads)
    {
        if (thread.joinable())
        {
            thread.join(); // 等待线程完成
        }
    }

    // 如果接收线程仍然可加入（即正在运行），则等待其完成
    if (receiveThread.joinable())
    {
        receiveThread.join();
    }

    // 关闭套接字
    closesocket(toServerSocket);
    // 清理 Winsock 环境
    WSACleanup();
}

int Controller::publish(const string& str)
{
    sprintf_s(sendBuf, "%s", str.c_str());
    //int sendLen = sendto(toServerSocket, sendBuf, strlen(sendBuf), 0, (sockaddr*)&serverAddr, sizeof(serverAddr));
    int sendLen = send(toServerSocket, sendBuf, strlen(sendBuf), 0);
    Sleep(50);
    return sendLen;
}

int Controller::receive()
{
    int recvLen;
    for (int i = 0; i < 8; i++)
    {
        //emplace_back函数的调用方式类似于push_back，但它接受一个或多个参数，这些参数直接用于在vector的末尾构造新的元素。这些参数会传递给元素的构造函数
        //下面代码，使用线程执行类的成员函数构造线程
        threads.emplace_back(&Controller::handle, this);
    }
    while (true)
    {
        //recvLen = recvfrom(toServerSocket, recvBuf, sizeof(recvBuf), 0, nullptr, nullptr);
        memset(recvBuf, 0, sizeof(recvBuf));//清空接收缓冲区
        recvLen = recv(toServerSocket, recvBuf, sizeof(recvBuf), 0);//接收
        if (recvLen > 0)
        {
            string data(recvBuf, recvLen);
            //cout<<"**Receive:"<<endl<<data<<endl;
            lock_guard<mutex> lock(queue_mtx);
            data_queue.emplace(recvBuf, recvLen);
            cv.notify_one();
        }
        else if(recvLen == 0)
        {
            cout << "服务器已断开链接" << endl;
            break;
        }
        else
        {
            perror("接收数据失败");
            break;
        }
    }
    return recvLen;
}

void Controller::handle()
{
    while (true)
    {
        unique_lock<mutex> lock(queue_mtx);
        cv.wait(lock, [] {return !data_queue.empty(); });// 等待直到队列不为空
        string data(data_queue.front());
        data_queue.pop();
        lock.unlock();

        vector<string> res;
        stringSplit(data, ',', res);
        //cout  << "Controller::handle " << res[0] << endl;

        int pos = 0;
        while(pos<res.size())
        {
            if (res[pos].find("Start") == 0)
            {
                startFlag = true;
                cout  << "get Start command." << endl;
                pos+=1;
            }
            else if (res[pos].find("WheelSpeed") == 0)//处理车轮转速
            {
                cout << res[pos] << endl;
                //lock_guard<mutex> player_lock(player_mtx);
                player.LSpeed = stof(res[pos+1]);
                player.RSpeed = stof(res[pos+2]);
                pos+=3;
                if (callback_Car)
                {
                    callback_Car();
                }
            }
            else if (res[pos].find("Robot") == 0)//处理小车变换
            {
                lock_guard<mutex> player_lock(player_mtx);
                player.transform.position.x = stof(res[pos+1]);
                player.transform.position.y = stof(res[pos+2]);
                player.transform.rotation = stof(res[pos+6]);
                cout << "x:" << player.transform.position.x << " y:" << player.transform.position.y << " rot:" << player.transform.rotation<<endl;
                pos+=7;

                if (callback_Car)
                {
                    callback_Car();
                }
            }
            /*else if (res[0].find("IRS") == 0)//处理红外
            {
                lock_guard<mutex> irs_lock(irs_mtx);
                irs.flag = stoi(res[1]);
                irs.distance = stof(res[2]);
                if (callback_IRS)
                {
                    callback_IRS();
                }
            }*/
            else if (res[pos].find("IMU") == 0)//处理IMU
            {
                lock_guard<mutex> imu_lock(imu_mtx);
                imu.Acceleration.x = stof(res[pos+1]);
                imu.Acceleration.y = stof(res[pos+2]);
                //imu.Acceleration.z = stof(res[pos+3]);
                imu.Gyroscope.x = stof(res[pos+4]);
                imu.Gyroscope.y = stof(res[pos+5]);
                //imu.Gyroscope.z = stof(res[pos+6]);
                imu.Direction.x = stof(res[pos+7]);
                imu.Direction.y = stof(res[pos+8]);
                //imu.Direction.z = stof(res[pos+9]);
                pos+=10;
                if (callback_IMU)
                {
                    callback_IMU();
                }
            }
            else if (res[pos].find("Camera") == 0)//处理摄像机
            {
                lock_guard<mutex> camera_lock(camera_mtx);
                camera.size.x = stof(res[pos+1]);
                camera.size.y = stof(res[pos+2]);

                camera.pos[0].x = stof(res[pos+4]);
                camera.pos[0].y = stof(res[pos+5]);
                camera.area[0] = stof(res[pos+6]);

                camera.pos[1].x = stof(res[pos+8]);
                camera.pos[1].y = stof(res[pos+9]);
                camera.area[1] = stof(res[pos+10]);

                camera.pos[2].x = stof(res[pos+12]);
                camera.pos[2].y = stof(res[pos+13]);
                camera.area[2] = stof(res[pos+14]);

                camera.pos[3].x = stof(res[pos+16]);
                camera.pos[3].y = stof(res[pos+17]);
                camera.area[3] = stof(res[pos+18]);
                pos+=19;

                if (callback_Camera)
                {
                    callback_Camera();
                }
            }
            else if (res[pos] == "Barrier")//处理检查点
            {
                lock_guard<mutex> obstacle_lock(barrier_mtx);
                Barrier temp;
                temp.type = res[pos+1];
                if(temp.type == "Rect"){
                    temp.point1.x = stof(res[pos+2]);
                    temp.point1.y = stof(res[pos+3]);
                    temp.point2.x = stof(res[pos+4]);
                    temp.point2.y = stof(res[pos+5]);
                    temp.point3.x = stof(res[pos+6]);
                    temp.point3.y = stof(res[pos+7]);
                    temp.point4.x = stof(res[pos+8]);
                    temp.point4.y = stof(res[pos+9]);
                }
                else {
                    temp.position.x = stof(res[pos+2]);
                    temp.position.y = stof(res[pos+3]);
                    temp.rotation = stof(res[pos+4]);
                    temp.Length = stof(res[pos+5]);
                    temp.width = stof(res[pos+6]);
                    temp.width = 0;
                }

                obstacles.emplace_back(temp);
                pos+=11;

                if (callback_Barrier)
                {
                    callback_Barrier();
                }
            }
            else if (res[pos] == "CheckPoint")//处理检查点
            {
                lock_guard<mutex> targets_lock(checkPoint_mtx);
                CheckPoint temp;
                temp.type = res[pos+1];
                temp.position.x = stof(res[pos+2]);
                temp.position.y = stof(res[pos+3]);
                temp.rotation = stof(res[pos+4]);
                temp.size = stof(res[pos+5]);
                temp.number = stoi(res[pos+6]);
                targets.emplace_back(temp);
                pos+=7;

                if (callback_CheckPoint)
                {
                    callback_CheckPoint();
                }
            }
            else
                pos++;
        }
    }

}

bool Controller::subScribe(const string& topicName)
{
    char buf[MAXSIZE_BUF] = "Subscribe,";
    sprintf_s(buf, MAXSIZE_BUF, "%s%s", buf, topicName.c_str());
    if (publish(buf) > 0)
        return true;
    else
        return false;
}

bool Controller::get(const string& topicName)
{
    char buf[MAXSIZE_BUF] = "Get,";
    sprintf_s(buf, MAXSIZE_BUF, "%s%s", buf, topicName.c_str());
    if (publish(buf) > 0)
        return true;
    else
        return false;
}

//---------public--------------------------------------------------------------------------
void Controller::delay(int milliseconds)
{
    this_thread::sleep_for(chrono::milliseconds(milliseconds));
}

void Controller::run()
{
    receiveThread = thread(&Controller::receive, this);

    get("Start");//告知服务器已准备就绪，服务器开始倒计时时传达startFlag
    long base_time, old_time;
    base_time = old_time = clock();
    while (!startFlag)
    {
        long now_time = clock();
        if(now_time-old_time>5000)
        {
            cout << "已经等待开始:" << (now_time-base_time)/1000 << "秒" << endl;
            old_time = now_time;
        }
    }

    cout<<"do setup()"<<endl;
    setup();

    vector<string> subscribeVector;
    vector<string> getVector;
    stringSplit(subscribeList,',',subscribeVector);
    stringSplit(getList,',',getVector);
    for(const string& topicName : subscribeVector)
    {
        if(subScribe(topicName))
            cout << topicName << "订阅成功" << endl;
        else
            perror("订阅失败");
    }
    for(const string& topicName : getVector)
    {
        if(get(topicName))
            cout << topicName << "获取成功" << endl;
        else
            perror("获取失败");
    }

    delay(300);// 延时0.3秒

    while (true)
    {
        loop();
        //this_thread::sleep_for(chrono::milliseconds(100));
    }
}

/*void Controller::digitalWrite(int pin, int value)
{

}*/

/*void Controller::ledcWrite(int channel, int value)
{

}*/

void Controller::setTwoTyres(float LSpeed, float RSpeed)
{
    char buf[MAXSIZE_BUF] = "Set,Robot,Wheels,";
    sprintf_s(buf, MAXSIZE_BUF, "%s%.2f,%.2f", buf, LSpeed, RSpeed);
    publish(buf);
}

const vector<Barrier>& Controller::getBarrierMessage() const {
    //lock_guard<mutex> _lock(barrier_mtx);
    return obstacles;
}

const vector<CheckPoint>& Controller::getCheckPointMessage() const {
    //lock_guard<mutex> _lock(checkPoint_mtx);
    return targets;
}

Car Controller::getCarMessage() const
{
    Car temp = player;
    return temp;
}

/*IRS Controller::getIRS()
{
    IRS temp = irs;
    return temp;
}*/

IMU Controller::getIMU() const
{
    IMU temp = imu;
    return temp;
}

Camera Controller::getCamera() const
{
    Camera temp = camera;
    return temp;
}

void Controller::on_IMU_Message_Update(CallbackType cb)
{
    callback_IMU = std::move(cb);
}

/*void Controller::on_IRS_Message_Update(CallbackType cb)
{
    callback_IRS = cb;
}*/

void Controller::on_Camera_Update(CallbackType cb)
{
    callback_Camera = std::move(cb);
}

void Controller::on_Car_Message_Update(CallbackType cb)
{
    callback_Car = std::move(cb);
}

void Controller::on_Barrier_Message_receive(CallbackType cb)
{
    callback_Barrier = std::move(cb);
}

void Controller::on_CheckPoint_Message_receive(CallbackType cb)
{
    callback_CheckPoint = std::move(cb);
}

Controller* Controller::instance = nullptr;

int main()
{
    //Controller::getInstance()，返回实例，如果实例没有创新，就新建
    Controller::getInstance()->run();
    return 0;
}
