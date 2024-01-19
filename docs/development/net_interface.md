# 网络通信接口

本小节将介绍RflyPilot的网络通信接口，包括RflySim3D接口、示波器接口等。

|功能|接口|接/发|说明|源文件|
|---|---|---|---|---|
|远程控制台SSH|TCP:22|接发|任何模式||
|RflySim3D视景显示|UDP:20010|发送|在SIH模式下启用|``src/application/scope_thread.cpp``|
|系统示波器|UDP:3333|发送|任何模式|``src/application/scope_thread.cpp``|
|控制器示波器（自定义）|UDP:3334|发送|任何模式|``src/application/scope_thread.cpp``|
|姿态估计示波器（自定义）|UDP:3335|发送|任何模式|``src/application/scope_thread.cpp``|
|位置估计示波器（自定义）|UDP:3336|发送|任何模式|``src/application/scope_thread.cpp``|
|执行器远程调试接口|UDP:2222|接收|在OFFBOARD模式启用|``src/application/offboard_thread.cpp``|


!!! TIP
    执行器远程调试接口的功能是提供Simulink接口，使开发者可以利用Simulink调节执行器输出的PWM。相应的Simulink文件在``debug_tools/udp_send.slx``文件中。

!!! 注意
    使用RflyPilot的网络相关功能时，需要将RflyPilot连接到局域网，同时设置好``rflypilot.txt``中的``scope_ip``（在线示波器计算机的IP）和``station_ip``(RflySim3D和执行器远程调试接口计算机的IP)。
    ```C
    # string
    scope_ip = "192.168.199.233" # 152
    station_ip = "192.168.199.233"
    ```