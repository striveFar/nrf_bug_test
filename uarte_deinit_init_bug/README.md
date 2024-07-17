### 问题描述： 

上位机接USB转TTL模块，接到NRF52840的 26 27脚，上位机持续发送，间隔10毫秒，
52840 使用UART EasyDMA 定长接收，并将收到的数据放入环形buffer，每包数据长度固定为11个字节，
测试数据为01 05 00 01 00 00 00 00 00 00 2B，
然后在发送过程中每隔5秒调用一次nrfx_uarte_uninit或者nrfx_uarte_init,此时出现两个问题，

问题一：nrfx_uarte_uninit 内部存在不满足的while(1) 条件，无法退出，与DevZone上该帖子描述一致 https://devzone.nordicsemi.com/f/nordic-q-a/75420/infinite-loop-in-nrfx_uarte_uninit

```c
# 改为nrf 最新hal 库： https://github.com/nrfconnect/sdk-hal_nordic/blob/main/nrfx/drivers/src/nrfx_uarte.c#L152 中的用法

```
问题二：

### log报错信息:

尝试反复插拔之后，串口错误状态寄存器（ERRORSRC）会报04或者05或者是01，
之后一旦连接RX（此时上位机依旧持续发包）就一直进错误回调或无法继续接收，只能复位芯片，
尝试了多种方法清除错误均无法避免重复进错误回调的问题，哪怕是重新deinit/init UARTE也不行，

### 工程环境及编译：
```
Ubuntu 22.04.4 LTS

nRF Connect SDK v2.5.2 

CMake version  minimum required is "3.20.0"
Zephyr version: 3.4.99
toolchain: zephyr 0.16.5
dtc: minimum required is "1.4.6"
```

编译：
```
west build -o=-j8 -p always -b nrf52840dk_nrf52840 /path/to/uarte --build-dir build-uarte
```

### 复现方式:

1. 连接方式如下：

PC上位机接USB转TTL模块，USB转TTL模块的TX RX 分别连接到NRF52840的 26 27脚
上位机波特率1Mbps ,  上位机设置连续发送，间隔10ms, 发送数据为 01 05 00 01 00 00 00 00 00 00 2B
观察log 信息，840 仅在第一次调用nrfx_uarte_uninit(p_inst);之前的5秒内可以收到上位机发过来的数据并显示

2. log 查看

- 通过串口查看, 115200bps, 8 none 1 方式查看， 接线： TX-P.06, RX-P.08

- 可以通过JLinkRTTViewer 查看:
```
JLinkRTTViewer -s 4000 -sn <SN码> -if SWD -d NRF52840_XXAA --bright -ct usb -a
```

### Question

请问如何处理才能使nrfx_uarte_uninit;与nrfx_uarte_init互相生效，且deinit后再次init保证串口功能正常
