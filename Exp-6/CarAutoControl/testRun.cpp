#include "Controller.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <queue>
#include <vector>
#include <memory>

using namespace std;

shared_ptr<Controller> controller = shared_ptr<Controller>(Controller::getInstance(), [](Controller*) {});

// ==================== A* 算法类开始 ====================
struct Point {
    float x, y;
};

struct Node {
    int x, y;
    float g, h;
    Node* parent;

    float f() const { return g + h; }

    bool operator>(const Node& other) const
    {
        return f() > other.f();
    }
};

class AStarPlanner {
public:
    int width, height;
    float gridSize;
    float robotRadius;
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 障碍

    AStarPlanner(float mapW, float mapH, float gSize, float rRadius)
        : gridSize(gSize)
        , robotRadius(rRadius)
    {
        width = std::ceil(mapW / gridSize);
        height = std::ceil(mapH / gridSize);
        grid.resize(width, std::vector<int>(height, 0));
    }

    // 添加矩形障碍物并膨胀
    void addObstacle(float x1, float y1, float x2, float y2)
    {
        float minX = std::min(x1, x2) - robotRadius;
        float maxX = std::max(x1, x2) + robotRadius;
        float minY = std::min(y1, y2) - robotRadius;
        float maxY = std::max(y1, y2) + robotRadius;

        int startX = std::max(0, (int)(minX / gridSize));
        int endX = std::min(width - 1, (int)(maxX / gridSize));
        int startY = std::max(0, (int)(minY / gridSize));
        int endY = std::min(height - 1, (int)(maxY / gridSize));

        for (int i = startX; i <= endX; ++i) {
            for (int j = startY; j <= endY; ++j) {
                grid[i][j] = 1;
            }
        }
    }

    // 核心寻路函数
    std::vector<Point> findPath(float startX, float startY, float endX, float endY)
    {
        int sx = startX / gridSize;
        int sy = startY / gridSize;
        int ex = endX / gridSize;
        int ey = endY / gridSize;

        // 边界保护
        if (sx < 0 || sx >= width || sy < 0 || sy >= height)
            return {};
        if (ex < 0 || ex >= width || ey < 0 || ey >= height)
            return {};

        // 如果终点在墙里，简单搜寻附近最近的空点（简略处理）
        if (grid[ex][ey] == 1) {
            cout << "警告：目标点在障碍物内" << endl;
            return {};
        }

        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
        std::vector<std::vector<bool>> closedSet(width, std::vector<bool>(height, false));
        std::vector<std::vector<Node*>> allNodes(width, std::vector<Node*>(height, nullptr));

        Node* startNode = new Node { sx, sy, 0, heuristic(sx, sy, ex, ey), nullptr };
        allNodes[sx][sy] = startNode;
        openSet.push(*startNode);

        std::vector<Point> path;
        int dx[] = { 0, 0, 1, -1, 1, 1, -1, -1 };
        int dy[] = { 1, -1, 0, 0, 1, -1, 1, -1 };
        float costs[] = { 1, 1, 1, 1, 1.414, 1.414, 1.414, 1.414 };

        while (!openSet.empty()) {
            Node current = openSet.top();
            openSet.pop();

            if (current.x == ex && current.y == ey) {
                // 回溯路径
                Node* curr = allNodes[current.x][current.y];
                while (curr != nullptr) {
                    path.push_back({ curr->x * gridSize + gridSize / 2, curr->y * gridSize + gridSize / 2 });
                    curr = curr->parent;
                }
                std::reverse(path.begin(), path.end());
                // 释放内存（简单写法，实际项目应用智能指针或内存池）
                // 这里为了代码短，略过详细的 delete
                return path;
            }

            if (closedSet[current.x][current.y])
                continue;
            closedSet[current.x][current.y] = true;

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (nx >= 0 && nx < width && ny >= 0 && ny < height && grid[nx][ny] == 0 && !closedSet[nx][ny]) {
                    float newG = current.g + costs[i];

                    if (allNodes[nx][ny] == nullptr || newG < allNodes[nx][ny]->g) {
                        Node* neighbor = new Node { nx, ny, newG, heuristic(nx, ny, ex, ey), allNodes[current.x][current.y] };
                        allNodes[nx][ny] = neighbor;
                        openSet.push(*neighbor);
                    }
                }
            }
        }
        return {}; // 没找到路径
    }

    float heuristic(int x1, int y1, int x2, int y2)
    {
        return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
    }
    bool hasLineOfSight(Point p1, Point p2)
    {
        float dist = std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
        int steps = dist / (gridSize / 2.0); // 步长取栅格的一半，防止漏测
        if (steps <= 0)
            return true;

        float dx = (p2.x - p1.x) / steps;
        float dy = (p2.y - p1.y) / steps;

        for (int i = 1; i < steps; ++i) {
            float checkX = p1.x + dx * i;
            float checkY = p1.y + dy * i;

            int gx = checkX / gridSize;
            int gy = checkY / gridSize;

            // 越界检查
            if (gx < 0 || gx >= width || gy < 0 || gy >= height)
                return false;
            // 碰撞检查
            if (grid[gx][gy] == 1)
                return false;
        }
        return true;
    }

    // 路径平滑函数：去除多余的中间点
    std::vector<Point> smoothPath(const std::vector<Point>& rawPath)
    {
        if (rawPath.size() < 3)
            return rawPath;

        std::vector<Point> smoothedPath;
        smoothedPath.push_back(rawPath[0]); // 起点必须有

        int currentIdx = 0;

        while (currentIdx < rawPath.size() - 1) {
            // 贪心策略：从最后一个点开始往前找，看哪个点能和当前点直连
            for (int i = rawPath.size() - 1; i > currentIdx; --i) {
                // 如果当前点和目标点 i 之间没有障碍物
                if (hasLineOfSight(rawPath[currentIdx], rawPath[i])) {
                    smoothedPath.push_back(rawPath[i]);
                    currentIdx = i; // 直接跳到这个点
                    break;
                }
                // 如果只剩相邻点，被迫加入
                if (i == currentIdx + 1) {
                    smoothedPath.push_back(rawPath[i]);
                    currentIdx = i;
                }
            }
        }
        return smoothedPath;
    }
};
// ==================== A* 算法类结束 ====================
void turn(float degree)
{
    float Kp = 0.1; // 比例系数
    float Ki = 0.001; // 积分系数
    float Kd = 0.005; // 微分系数
    float integral = 0; // 积分项
    float prev_error = 0; // 上一次的误差
    float max_integral = 100; // 积分饱和上限

    float init = controller->getCarMessage().transform.rotation;
    auto startTime = chrono::steady_clock::now();

    while (true) {
        float current = controller->getCarMessage().transform.rotation;
        float error = degree - (current - init);

        if (error < -180)
            error += 360;
        else if (error > 180)
            error -= 360;
        // printf("%f\n", error);

        if (fabs(error) < 1) {
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
        if (elapsed_seconds.count() >= 3.0) // 设置超时时间为3秒
        {
            cout << "旋转超时" << endl;
            controller->setTwoTyres(-20, -20);
            break;
        }
    }
}

bool moveForward(float distance, float speed)
{
    // PID 控制参数
    float kp_distance = 1.0; // 比例系数（距离）
    float ki_distance = 0.01; // 积分系数（距离）
    float kd_distance = 0.03; // 微分系数（距离）

    float kp_angle = 1.2; // 比例系数（角度）
    float ki_angle = 0.005; // 积分系数（角度）
    float kd_angle = 0.005; // 微分系数（角度）

    float integral_distance = 0; // 积分项（距离）
    float prev_error_distance = 0; // 上一次的误差（距离）
    float integral_angle = 0; // 积分项（角度）
    float prev_error_angle = 0; // 上一次的误差（角度）

    // 限制积分项最大值
    const float max_integral_distance = 10.0;
    const float max_integral_angle = 1.0;

    // 初始位置
    float initX = controller->getCarMessage().transform.position.x;
    float initY = controller->getCarMessage().transform.position.y;
    float initRotation = controller->getCarMessage().transform.rotation;

    // 目标方向角
    float targetAngle = atan2(distance, 1); // 假设目标方向角为正前方

    int direct = 1;
    if (distance < 0)
        direct = -1;

    auto startTime = std::chrono::steady_clock::now();

    while (true) {
        // 当前位置
        float currentX = controller->getCarMessage().transform.position.x;
        float currentY = controller->getCarMessage().transform.position.y;
        float currentRotation = controller->getCarMessage().transform.rotation;

        // 计算当前位置到目标位置的距离
        float distanceError = distance - sqrt((currentX - initX) * (currentX - initX) + (currentY - initY) * (currentY - initY));

        // 计算当前位置的方向角
        // float currentAngle = atan2(currentY - initY, currentX - initX);
        float currentAngle = atan2(initX - currentX, initY - currentY);

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
        if (fabs(distanceError) < 2) {
            controller->setTwoTyres(0, 0);
            std::cout << "Reached target position." << std::endl;
            return true;
        }

        // 检查是否超时
        auto currentTime = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed_seconds = currentTime - startTime;
        if (elapsed_seconds.count() >= 5.0) // 设置超时时间为5秒
        {
            cout << "直行超时" << endl;
            return false;
        }
    }
}

bool moveToTarget(float x, float y, float speed)
{
    float initX = controller->getCarMessage().transform.position.x;
    float initY = controller->getCarMessage().transform.position.y;
    float initRotation = controller->getCarMessage().transform.rotation;
    float Dir = atan2(initY - y, x - initX) * 180.0 / M_PI;
    // float Dir = atan2(initX - x, initY - y) * 180.0 / M_PI;
    printf("(%f, %f) to (%f, %f) %f度 to %f度 \n", initX, initY, x, y, initRotation, Dir);
    turn(Dir - initRotation);

    float distance = sqrt((x - initX) * (x - initX) + (y - initY) * (y - initY));
    if (!moveForward(distance, speed)) {
        controller->setTwoTyres(-20, -20);
        Controller::delay(500);
        turn(-75);
        controller->setTwoTyres(20, 20);
        Controller::delay(1000);
        return false;
    } else
        return true;
}

void setup()
{
    cout << "Run setup" << endl;
    // 需要设置subscribeList 和 getList
    // controller->subscribeList = "RobotBaseTransform,Camera";
    controller->subscribeList = "IMU,RobotBaseTransform";
    controller->getList = "Barrier,CheckPoint"; // Barrier,CheckPoint
}

//---------------------------------------------用于竞速---------------------------------------
bool doOnce = true;
int target = 0;
static vector<Barrier> barriers;
// static vector<CheckPoint> checkPoints;
//---------------------------------------------用于竞速---------------------------------------

// 在 loop 外面定义规划器指针
AStarPlanner* planner = nullptr;

void loop()
{
    vector<Point> checkPoints;
    checkPoints.push_back({ 107.057f, 176.367f });
    checkPoints.push_back({ 331.716f, 174.703f });
    checkPoints.push_back({ 331.974f, 37.861f });
    checkPoints.push_back({ 115.681f, 45.8828f });

    // ================== 1. 初始化阶段 ==================
    if (doOnce) {
        barriers = controller->getBarrierMessage();
        // checkPoints = controller->getCheckPointMessage();

        // 1. 初始化规划器 (地图宽400, 高250, 栅格精度5.0, 机器人半径15.0)
        // 栅格越大计算越快但精度越低，机器人半径要比实际稍大一点作为安全余量
        planner = new AStarPlanner(400.0f, 250.0f, 2.5f, 7.0f);

        // 2. 将障碍物添加到地图中
        ofstream fout("../data.txt");
        for (const auto& barrier : barriers) {
            // 将四个点中取最小和最大值构建矩形
            float minX = min({ barrier.point1.x, barrier.point2.x, barrier.point3.x, barrier.point4.x });
            float maxX = max({ barrier.point1.x, barrier.point2.x, barrier.point3.x, barrier.point4.x });
            float minY = min({ barrier.point1.y, barrier.point2.y, barrier.point3.y, barrier.point4.y });
            float maxY = max({ barrier.point1.y, barrier.point2.y, barrier.point3.y, barrier.point4.y });

            planner->addObstacle(minX, minY, maxX, maxY);

            // 原有的日志记录
            fout << "Barrier: " << minX << "," << minY << " to " << maxX << "," << maxY << endl;
        }
        fout.close();
        doOnce = false;
        cout << "地图构建完成！" << endl;
    }

    // ================== 2. 任务执行阶段 ==================
    if (target >= 4) {
        cout << "所有目标完成 finish" << endl;
        exit(0);
    }

    cout << ">>> 开始前往第 " << target << " 个目标" << endl;

    // 获取当前位置
    float curX = controller->getCarMessage().transform.position.x;
    float curY = controller->getCarMessage().transform.position.y;
    float targetX = checkPoints[target].x;
    float targetY = checkPoints[target].y;

    // 1. 计算原始 A* 路径
    vector<Point> rawPath = planner->findPath(curX, curY, targetX, targetY);

    if (!rawPath.empty()) {
        // 2. 进行平滑处理 (合并直线点，保留拐点)
        vector<Point> finalPath = planner->smoothPath(rawPath);

        cout << "原始路径点数: " << rawPath.size() << " -> 优化后关键点数: " << finalPath.size() << endl;

        // 3. 依次访问关键点
        for (size_t i = 1; i < finalPath.size(); ++i) { // 从1开始，跳过起点
            Point p = finalPath[i];
            cout << "前往关键拐点: " << p.x << ", " << p.y << endl;
            if (!moveToTarget(p.x, p.y, 20)) {
                // 出错处理...
            }
        }
    } else {
        cout << "未找到路径，尝试直接前往目标点" << endl;
    }
    // 3. 最后精确对准真正的目标点
    cout << "前往最终目标点..." << endl;
    if (moveToTarget(targetX, targetY, 20)) {
        cout << "抵达目标点 " << target << endl;
        target++;
        Controller::delay(500); // 稍微停顿一下
    } else {
        cout << "抵达最终目标失败，重试" << endl;
        // 回退一下防止死锁
        controller->setTwoTyres(-20, -20);
        Controller::delay(500);
    }
}
