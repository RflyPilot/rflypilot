# 其他设置

## 1 ``rflypilot.txt``:``sih_use_real_state``
这个选项是设置在SIH仿真模式下，是否使用真实状态作为控制系统的反馈，当为``1``时，则表示不启用状态估计系统，并直接将模型状态作为反馈输出到控制器中；当为``0``时，则表示启用状态估计系统，所有系统状态由传感器融合得到。

### ``src/application/sih_thread.cpp``
```C
   /* Get model outputs here */
    if(_config.sih_use_real_state)
    {
      // SIH_Model_Y._e_cf_s;
      memcpy(&_cf_msg, &SIH_Model_Y._e_cf_s, sizeof(_cf_msg));
      cf_output_msg.publish(&_cf_msg);
      
      // SIH_Model_Y._e_lpe_s;
      memcpy(&_lpe_msg, &SIH_Model_Y._e_lpe_s, sizeof(_lpe_msg));
      lpe_output_msg.publish(&_lpe_msg);

      lpeLowPass_output_msg.publish(&_lpe_msg);
    }    
```
### ``src/main.cpp``
```C
    case SIH:
      printf("mode : SIH\n");
      start_sih();
      usleep(500000);
      if(!(_config_msg.sih_use_real_state))
      {
        start_attitudeEstimator();
        start_lpe();
      }
      usleep(500000);      
      start_usrController();
    break;
```

## 2 ``rflypilot.txt``:``accel_cutoff_hz``,``gyro_cutoff_hz``

``accel_cutoff_hz``与``gyro_cutoff_hz``分别为加速度计和陀螺仪低通滤波器截止频率，单位Hz。
!!! Warning
    低通滤波器截止频率应小于IMU采样率的1/2。

!!! TIP
    默认使用的低通滤波器为二阶巴特沃斯滤波器，源码位于``src/lib/math_function.h``。
    ```C
    class iir_lpf2_typedef{
    public:
      iir_lpf2_typedef(float sample_freq, float cutoff_freq)
      {
        // set initial parameters
        set_cutoff_frequency(sample_freq, cutoff_freq);
      }
      iir_lpf2_typedef(void)
      {

      }
      // Change filter parameters
      void set_cutoff_frequency(float sample_freq, float cutoff_freq);
      float apply(float sample);
      // Return the cutoff frequency
      float get_cutoff_freq() const { return _cutoff_freq; }
      // Reset the filter state to this value
      float reset(float sample);
    protected:
      float _cutoff_freq;
      float _a1;
      float _a2;
      float _b0;
      float _b1;
      float _b2;
      float _delay_element_1;	// buffered sample -1
      float _delay_element_2;	// buffered sample -2
    };
    ```