# 编译与部署

本小节将介绍RflyPilotTools对RflyPilot源码进行编译与部署的基本原理，其背后是在WSL子系统中执行Linux指令。

## 编译

在WSL子系统下

```
root@DESKTOP-LVGT24C:/mnt/d/nash.zhao/RflyPilot_DEMO/RflyPilot# cd build/
root@DESKTOP-LVGT24C:/mnt/d/nash.zhao/RflyPilot_DEMO/RflyPilot/build# cmake ..
-- Configuring done
-- Generating done
-- Build files have been written to: /mnt/d/nash.zhao/RflyPilot_DEMO/RflyPilot/build
root@DESKTOP-LVGT24C:/mnt/d/nash.zhao/RflyPilot_DEMO/RflyPilot/build# make -j128
[  9%] Built target estimator_codegen
[ 18%] Built target libsbus
[ 18%] Built target confc
[ 38%] Built target sih_codegen
[ 38%] Built target basic_controller_codegen
[ 55%] Built target controller_codegen
[100%] Built target rflypilot
root@DESKTOP-LVGT24C:/mnt/d/nash.zhao/RflyPilot_DEMO/RflyPilot/build# 
```

## 部署

RflyPilot的部署运行的指令是``make upload``。其相关脚本文件在``tools/upload.sh``。

```
#!/bin/sh
hostname="192.168.199.183"
password="raspberry"

if [ $# = 1 ]; then
    hostname=$1
fi
if [ $# = 2 ]; then
    hostname=$1
    password=$2
fi

echo "hostname: ${hostname}"
echo "password: ${password}"

sshpass -p "${password}" ssh -o StrictHostKeyChecking=no pi@"${hostname}" 'mkdir -p RflyPilot_Project/RflyPilot/'
sshpass -p "${password}" scp -r ./rflypilot ../config/rflypilot.txt ../config/parameter.txt pi@"${hostname}":/home/pi/RflyPilot_Project/RflyPilot/
# sshpass -p "${password}" scp -r ./rflypilot ../config/rflypilot.txt ../config/calibration.txt ../config/parameter.txt pi@"${hostname}":/home/pi/RflyPilot_Project/RflyPilot/
```

!!! TIP
    开发者可以将``hostname``设置为RflyPilot对应的IP地址。