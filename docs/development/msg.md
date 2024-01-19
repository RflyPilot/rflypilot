# 线程间通信系统
RflyPilot采用了一套基于共享内存的线程间通信机制。通过消息``publish``和``read``机制实现一写多读的通信拓扑。同时该消息系统还支持统计消息发布频率和高质量日志记录功能。其源码位于``src/lib/ringbuffer.h``，感兴趣的读者可以自行阅读。系统内部消息定义在``src/msg/msg_def.*``文件中。

# 消息定义
在``msg_def.h``中，定义一个结构体作为消息。
```C
typedef struct
{
    uint64_t timestamp;
    float data[2];
}my_data_typedef;
```
!!! 注意
    每个结构体中必须包含``uint64_t timestamp``用于记录时间戳。

在``msg_def.cpp``中，定义消息

```C
#define N_MY_DATA 3
ringbuffer_typedef<my_data_typedef> my_data_msg(N_MY_DATA, "my_data", N);
```

这个消息定义中，利用模板``ringbuffer_typdef``，定义了``my_data_msg``消息。其中``N_MY_DATA``为环形存储器存储深度，即存储最新的``N_MY_DATA``个消息。``my_data``为消息名，N表示每``publish``N次后，写入一次日志消息，N即分频系数。当N=0时表示不记录日志消息。

在``msg_def.h``中，增加一行

```C
extern ringbuffer_typedef<my_data_typedef> my_data_msg;
```

# 消息的发布与读取


## 程序示例
```C
class adaptive_delay_typedef adp_delay(0.5,15,0);


void * thread_send(void * ptr)
{
  my_data_typedef send_my_data;
  uint32_t cnt = 0;
  for(;;)
  {
    send_my_data.data[0] = 1;
    send_my_data.data[1] = get_time_now()/1e6;
    my_data_msg.publish(&send_my_data);//发布消息
    if(cnt++ == 300)
    {
      printf("send: %f , %f\n",send_my_data.data[0], send_my_data.data[1]);
      cnt = 0;
    }
    adp_delay.delay_us(1000);//使用自适应延时函数
  }
}

void * thread_recv(void * ptr)
{
  my_data_typedef recv_my_data;

  for(;;)
  {
    my_data_msg.read(&recv_my_data);//读取消息
    printf("recv: %f , %f\n",recv_my_data.data[0], recv_my_data.data[1]);
    printf("send rate : %f \n", my_data_msg.publish_rate_hz);
    usleep(100000);
  }
}

int main(int argc, const char *argv[])
{
    bool ret = create_thread("thread_send", thread_send, NULL);
    ret = create_thread("thread_recv", thread_recv, NULL);
    while(1)
    {
        sleep(1);
    }

  return 0;

}
```

## 运行结果
```
recv: 1.000000 , 0.000012
send rate : 1000000.000000 
thread_recv starting
recv: 1.000000 , 0.100024
send rate : 956.022888 
recv: 1.000000 , 0.199830
send rate : 967.118042 
recv: 1.000000 , 0.299697
send rate : 975.609741 
send: 1.000000 , 0.311999
recv: 1.000000 , 0.399873
send rate : 981.354309 
recv: 1.000000 , 0.500497
send rate : 986.193237 
recv: 1.000000 , 0.599694
send rate : 990.098999 
send: 1.000000 , 0.617905
recv: 1.000000 , 0.700633
send rate : 992.063477 
recv: 1.000000 , 0.800291
send rate : 995.024902 
recv: 1.000000 , 0.900782
send rate : 996.015991 
send: 1.000000 , 0.920866
recv: 1.000000 , 1.000038
send rate : 997.009033 
recv: 1.000000 , 1.100356
send rate : 997.009033 
recv: 1.000000 , 1.200573
send rate : 998.004028 
send: 1.000000 , 1.222612
recv: 1.000000 , 1.300498
send rate : 1001.000977 
recv: 1.000000 , 1.400427
send rate : 1001.000977 
recv: 1.000000 , 1.501440
send rate : 1001.000977 
send: 1.000000 , 1.524425
```

!!! 注意
    程序中使用了[自适应延时函数](task_period.md)``adp_delay.delay_us()``以确保线程运行频率为设定值``1000Hz``，可以在运行结果中观察到运行频率稳定到``1000Hz``的过程。