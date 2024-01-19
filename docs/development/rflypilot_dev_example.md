# 开发示例
本小节将给出RflyPilot开发示例。首先介绍RflyPilot飞控目录结构，然后介绍如何为RflyPilot增加驱动和独立的线程，最终给出运行结果。

## 1 目录结构
```
.
├── CMakeLists.txt
├── README.md
├── basic_controller_codegen
│   ├── CMakeLists.txt
├── build
├── config
│   ├── calibration.txt
│   ├── configure.h
│   ├── parameter.txt
│   └── rflypilot.txt
├── controller_codegen
│   ├── CMakeLists.txt
├── estimator_codegen
│   ├── CMakeLists.txt
├── sih_codegen
│   ├── CMakeLists.txt
├── src
│   ├── 3rdparty
│   │   ├── conf-c
│   │   │   ├── CMakeLists.txt
│   │   │   ├── LICENSE
│   │   │   ├── README.md
│   │   │   ├── example
│   │   │   │   ├── Makefile
│   │   │   │   ├── configrc
│   │   │   │   ├── create
│   │   │   │   ├── create.c
│   │   │   │   ├── read
│   │   │   │   └── read.c
│   │   │   ├── include
│   │   │   │   └── conf-c
│   │   │   │       └── conf.h
│   │   │   └── src
│   │   │       ├── conf.c
│   │   │       ├── conf.h
│   │   │       ├── hash
│   │   │       │   ├── hash.c
│   │   │       │   └── hash.h
│   │   │       └── stack
│   │   │           ├── stack.c
│   │   │           └── stack.h
│   │   └── sbus
│   │       ├── CMakeLists.txt
│   │       ├── LICENSE
│   │       ├── README.md
│   │       ├── examples
│   │       │   ├── CMakeLists.txt
│   │       │   ├── blocking_receiver.cpp
│   │       │   ├── passthrough.cpp
│   │       │   └── send_to_self.cpp
│   │       ├── src
│   │       │   ├── CMakeLists.txt
│   │       │   ├── common
│   │       │   │   ├── CMakeLists.txt
│   │       │   │   └── include
│   │       │   │       └── sbus
│   │       │   │           ├── sbus_error.h
│   │       │   │           ├── sbus_packet.h
│   │       │   │           └── sbus_spec.h
│   │       │   ├── decoder
│   │       │   │   ├── CMakeLists.txt
│   │       │   │   ├── DecoderFSM.cpp
│   │       │   │   ├── include
│   │       │   │   │   └── sbus
│   │       │   │   │       ├── DecoderFSM.h
│   │       │   │   │       └── packet_decoder.h
│   │       │   │   └── packet_decoder.c
│   │       │   ├── driver
│   │       │   │   ├── CMakeLists.txt
│   │       │   │   ├── SBUS.cpp
│   │       │   │   └── include
│   │       │   │       └── SBUS.h
│   │       │   └── tty
│   │       │       ├── CMakeLists.txt
│   │       │       ├── include
│   │       │       │   └── sbus
│   │       │       │       ├── sbus_low_latency.h
│   │       │       │       ├── sbus_low_latency_impl.h
│   │       │       │       ├── sbus_tty.h
│   │       │       │       └── sbus_tty_impl.h
│   │       │       ├── sbus_low_latency_linux.c
│   │       │       ├── sbus_low_latency_none.c
│   │       │       └── sbus_tty_linux.c
│   │       └── test
│   │           ├── CMakeLists.txt
│   │           └── encode_decode.cpp
│   ├── application
│   │   ├── attitudeEstimator_thread.cpp
│   │   ├── attitudeEstimator_thread.h
│   │   ├── basicController_thread.cpp
│   │   ├── basicController_thread.h
│   │   ├── log_thread.cpp
│   │   ├── log_thread.h
│   │   ├── offboard_thread.cpp
│   │   ├── offboard_thread.h
│   │   ├── positionEstimator_thread.cpp
│   │   ├── positionEstimator_thread.h
│   │   ├── scope_thread.cpp
│   │   ├── scope_thread.h
│   │   ├── sih_thread.cpp
│   │   ├── sih_thread.h
│   │   ├── system_app.cpp
│   │   ├── system_app.h
│   │   ├── ulog_thread.cpp
│   │   ├── ulog_thread.h
│   │   ├── usrController_thread.cpp
│   │   └── usrController_thread.h
│   ├── calibration
│   │   ├── accel_calib.cpp
│   │   ├── ellipsoid_method.cpp
│   │   ├── ellipsoid_method.h
│   │   ├── ellipsoid_method_types.h
│   │   ├── gyro_calib.cpp
│   │   ├── mag_calib.cpp
│   │   ├── sensor_calibration.cpp
│   │   └── sensor_calibration.h
│   ├── console
│   │   ├── console.cpp
│   │   ├── console.h
│   │   ├── screen.cpp
│   │   └── screen.h
│   ├── drivers
│   │   ├── actuator
│   │   │   ├── fpga
│   │   │   │   ├── actuator_fpga.cpp
│   │   │   │   └── actuator_fpga.h
│   │   │   └── pca9685
│   │   │       ├── pca9685.cpp
│   │   │       └── pca9685.h
│   │   ├── barometer
│   │   │   └── ms5611
│   │   │       ├── ms5611.cpp
│   │   │       ├── ms5611.h
│   │   │       └── ms5611_main.cpp
│   │   ├── gps
│   │   │   ├── gps_api.cpp
│   │   │   ├── gps_api.h
│   │   │   └── ubx.h
│   │   ├── imu
│   │   │   ├── accelerometer
│   │   │   │   ├── PX4Accelerometer.cpp
│   │   │   │   └── PX4Accelerometer.h
│   │   │   ├── gyroscope
│   │   │   │   ├── PX4Gyroscope.cpp
│   │   │   │   └── PX4Gyroscope.h
│   │   │   ├── icm20689
│   │   │   │   ├── icm20689.cpp
│   │   │   │   └── icm20689.h
│   │   │   ├── icm20689new
│   │   │   │   ├── ICM20689.cpp
│   │   │   │   ├── ICM20689.h
│   │   │   │   └── InvenSense_ICM20689_registers.h
│   │   │   └── icm42688p
│   │   │       ├── ICM42688P.cpp
│   │   │       ├── ICM42688P.h
│   │   │       ├── InvenSense_ICM42688P_registers.h
│   │   │       ├── atomic.h
│   │   │       ├── icm42688p_main.cpp
│   │   │       ├── sensor_accel_fifo.h
│   │   │       └── sensor_gyro_fifo.h
│   │   ├── magnetometer
│   │   │   ├── ist8310
│   │   │   │   ├── iSentek_IST8310_registers.h
│   │   │   │   ├── ist8310.cpp
│   │   │   │   ├── ist8310.h
│   │   │   │   └── ist8310_main.cpp
│   │   │   └── qmc5883l
│   │   │       ├── QMC5883L.cpp
│   │   │       ├── QMC5883L.h
│   │   │       ├── QMC5883l_main.cpp
│   │   │       └── QST_QMC5883L_registers.h
│   │   ├── posix
│   │   │   ├── i2c.cpp
│   │   │   ├── i2c.h
│   │   │   ├── spi.cpp
│   │   │   └── spi.h
│   │   ├── px4lib
│   │   │   ├── limit.h
│   │   │   ├── test_time.cpp
│   │   │   └── test_time.h
│   │   └── rc
│   │       └── sbus
│   │           ├── sbus_api.cpp
│   │           └── sbus_api.h
│   ├── include.h
│   ├── lib
│   │   ├── math_function.cpp
│   │   ├── math_function.h
│   │   └── ringbuffer.h
│   ├── log
│   │   ├── binlog.cpp
│   │   └── binlog.h
│   ├── main.cpp
│   ├── msg
│   │   ├── msg_def.cpp
│   │   └── msg_def.h
│   ├── parameter
│   │   ├── parameter_read.cpp
│   │   └── parameter_read.h
│   ├── scope
│   │   ├── scope.cpp
│   │   └── scope.h
│   ├── simulink_utility
│   └── system_utility
├── tools
│   ├── compile.sh
│   └── upload.sh
```
## 2 增加IO驱动

## 3 