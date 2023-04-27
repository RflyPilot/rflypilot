# Configuration

## TODO配置清单
- 如何切换电机输出协议
- 使用前需要在rflypilot.txt文件根据自身需求修改控制器频率
- 代码生成注意启用long long 类型

## 关键配置
在`config/configure.h`文件中

- `#define USE_RFLYPILOT 1` 表示启用RflyPilot硬件 
- `#define USE_RFLYPILOT 0` 表示使用PilotPi硬件
- `#define USE_ONESHOT_125 1` 表示使用ONESHOT125协议 
- `#define USE_ONESHOT_125 0` 表示使用400Hz PWM

## 基于WSL交叉编译
在WSL ubuntu 18.04中安装

1. `sudo apt-get install gcc-arm-linux-gnueabihf`
2. `sudo apt-get install g++-arm-linux-gnueabihf`
3. `sudo apt-get install sshpass`

## 原生编译

目前原生编译生成的程序无法稳定产生log，暂时不建议使用