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
