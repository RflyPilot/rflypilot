# 软件系统架构
本节内容将介绍RflyPilot的软件系统架构。RflyPilot系统中的任务主要以线程的形式存在，部分线程之间独立运行互不干扰，而某些关键线程存在节拍关系。各个线程间通过内部消息系统进行通信。
# 主函数
RflyPilot在启动时会读取``rflypilot.txt``配置文件，并根据``valid_mode``的设置情况，通过``switch-case``语句，判断启动哪些线程。在RflyPilot中，所有任务都以线程的形式存在，比如控制器线程``start_usrController()``、IMU线程``start_icm42688p()``、GPS线程``start_gps()``、日志线程``start_ulog()``等等。

|线程|功能|备注|
|---|---|---|
|``start_console``|启动控制台||
|``start_screen``|启动控制台实时参数显示功能||
|``start_ulog``|启动ulog日志记录||
|``start_sbus``|运行串行总线接收机驱动|HIL/EXP/SIH|
|``start_system_app``|运行默认的系统级消息发布系统|主要用于更新消息``system_scope_msg``|
|``start_scope``|运行在线示波器系统|包括系统示波器、控制器示波器、状态估计器示波器|
|``start_icm20689_new``|启动icm20689线程|IMU，HIL，支持FIFO读取|
|``start_ist8310``|启动ist8310线程|磁力计，HIL|
|``start_baro``|启动MS5611线程|气压计，HIL/EXP|
|``start_gps``|启动GPS线程|GPS，HIL/EXP|
|``start_attitudeEstimator``|启动姿态估计线程|支持代码生成|
|``start_lpe``|启动LPE位置估计器|支持代码生成|
|``start_usrController``|启动控制器线程|支持代码生成|
|``start_icm42688p``|启动icm42688p线程|IMU，EXP|
|``start_qmc5883l``|启动qmc5883l线程|磁力计，EXP|
|``start_sih``|启动SIH仿真模型线程|被控对象模型，SIH|
|``start_offboard``|启动offboard线程|用于接收计算机指令|
|``start_calibration``|启动传感器校准线程|仅在传感器校准时运行|

!!! TIP 
    关于各个仿真验证模式运行不同线程的具体情况，感兴趣的的读者可以仔细阅读代码``src/main.cpp``。

# 系统级函数
系统级函数是有与嵌入式系统紧密相关的函数，包括线程的创建、核心绑定、时间戳等等，这些函数通常定义在``src/system_utility``中。

|函数|功能|备注|
|---|---|---|
|``create_thread``|用于创建线程|bool ret = create_thread("barometer", thread_baro, NULL);|
|``core_bind``|用于将某一线程绑定至某个CPU核心|输入参数0~3|
|``get_time_now``|用于获取当前系统时间|单位(us)|

