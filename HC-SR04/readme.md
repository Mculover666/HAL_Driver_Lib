# 超声波模块驱动

本驱动支持同时多个超时波模块，底层需要至少一个us级硬件定时器支持

# 示例

创建设备对象：
```c
hc_sr04_device_t hc_sr04_device1;
```

初始化设备对象：
```c
hc_sr04_device1.trig_port = GPIOB;
hc_sr04_device1.trig_pin  = GPIO_PIN_8;
hc_sr04_device1.echo_port = GPIOB;
hc_sr04_device1.echo_pin  = GPIO_PIN_9;
hc_sr04_device1.tim       = &htim2;
HC_SR04_Init(&hc_sr04_device1);
```

获取该设备的数据：
```c
ret = HC_SR04_Measure(&hc_sr04_device1);
if (ret < 0) {
printf("measure fail\r\n");
}
printf("distance:%.2f cm\r\n", hc_sr04_device1.distance);
```