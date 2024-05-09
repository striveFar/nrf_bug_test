### 问题描述： 

上位机接USB转TTL模块，接到NRF52840的 26 27脚，上位机持续发送，间隔10毫秒，
52840 使用UART EasyDMA 定长接收，并将收到的数据放入环形buffer，每包数据长度固定为11个字节，
测试数据为01 05 00 01 00 00 00 00 00 00 2B，然后在发送过程中反复插拔rx引脚（分体键盘有线通讯涉及到串口线的插拔，此为压力测试）

### log报错信息:

尝试反复插拔之后，串口错误状态寄存器（ERRORSRC）会报04或者05或者是01，
之后一旦连接RX（此时上位机依旧持续发包）就一直进错误回调，只能复位芯片，
尝试了多种方法清除错误均无法避免重复进错误回调的问题，哪怕是重新deinit/init UARTE也不行，

### 工程环境及编译：
Ubuntu 22.04.4 LTS

nRF Connect SDK v2.5.2 

CMake version  minimum required is "3.20.0"
Zephyr version: 3.4.99
toolchain: zephyr 0.16.5
dtc: minimum required is "1.4.6"

编译：
west build -o=-j8 -p always -b nrf52840dk_nrf52840 /path/to/uarte --build-dir build-uarte

### 复现方式:

1.连接方式如下：

PC上位机接USB转TTL模块，USB转TTL模块的TX RX 分别连接到NRF52840的 26 27脚
上位机波特率1Mbps ,  反复插拔840 RX引脚，很快就可以复现

2. log 查看

2.1 可以通过JLinkRTTViewer 查看:
JLinkRTTViewer -s 4000 -sn <SN码> -if SWD -d NRF52840_XXAA --bright -ct usb -a

2.2 也可通过串口, 115200bps, 8 none 1 方式查看，TX-P.06, RX-P.08

### Question

请问如何处理才能在进中断的错误处理后，不影响之后的接收
