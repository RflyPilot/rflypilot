本小结将对常用的三种仿真模式（MIL\SIH\HIL）进行操作介绍。

![MIL_SIH_HIL](../introduction/img/validation_mode.jpg)

# MIL
MIL仿真，即模型在环，该仿真模式主要用于进行控制系统的初期验证阶段，用于评估控制器的可控性。利用RflyPilot工程文件夹下的``MIL/MPC_HIL.slx``即可进行MIL仿真。这部分仿真主要在Simulink中进行，其仿真方法，这里不再赘述，具体内容由读者自行决定，相应的内容可以参考[快速使用](../quick_start/env_install.md)中的部分内容。
# SIH
SIH仿真，即仿真器在环，该仿真模式主要用于测试控制器在真实嵌入式系统中的表现。SIH的仿真步骤在[快速使用](../quick_start/env_install.md)中进行了详细介绍，这里也不再赘述。
# HIL
HIL，即半物理仿真，一般作为实飞实验的最后一步，在这个模式中，RflyPilot将直接连接到[硬件在环实时仿真系统](https://rflybuaa.github.io/RflySimRTDoc/)上，其硬件连接方法请参考[硬件连接](./hardware_connection.md)一节中的内容。