cmake_minimum_required(VERSION 3.28)
project(VirtualRobot)

set(CMAKE_CXX_STANDARD 17)

add_executable(run_virt
        #main.cpp
        Controller.cpp
        Controller.h
        testRun.cpp
)

add_executable(ball_virt
        Controller.cpp
        Controller.h
        testBall.cpp
)

# 添加此行来链接ws2_32库
target_link_libraries(run_virt ws2_32)
target_link_libraries(ball_virt ws2_32)



