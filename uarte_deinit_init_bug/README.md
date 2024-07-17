### 问题描述： 

上位机接USB转TTL模块，接到NRF52840的 26 27脚，上位机持续发送，间隔10毫秒，
52840 使用UART EasyDMA 定长接收，并将收到的数据放入环形buffer，每包数据长度固定为11个字节，
测试数据为01 05 00 01 00 00 00 00 00 00 2B，
然后在发送过程中每隔5秒调用一次nrfx_uarte_uninit或者nrfx_uarte_init,此时出现两个问题，

问题一：nrfx_uarte_uninit 内部存在不满足的while(1) 条件，无法退出，与DevZone上该帖子描述一致 https://devzone.nordicsemi.com/f/nordic-q-a/75420/infinite-loop-in-nrfx_uarte_uninit

(当前hal_nordic版本： revision: 9784731461018d3e983604698fbbed6af2bea801)
改为nrf 最新hal 库中的方法后，虽然可以超时退出，但是会出现问题二的现象

```c

// https://github.com/nrfconnect/sdk-hal_nordic/blob/main/nrfx/drivers/src/nrfx_uarte.c#L152
// https://github.com/nrfconnect/sdk-hal_nordic/blob/main/nrfx/drivers/src/nrfx_uarte.c#L301

static void interrupts_disable(nrfx_uarte_t const * p_instance)
{
    nrf_uarte_int_disable(p_instance->p_reg, NRF_UARTE_INT_ENDRX_MASK |
                                             NRF_UARTE_INT_ENDTX_MASK |
                                             NRF_UARTE_INT_ERROR_MASK |
                                             NRF_UARTE_INT_RXTO_MASK  |
                                             NRF_UARTE_INT_TXSTOPPED_MASK);
    NRFX_IRQ_DISABLE(nrfx_get_irq_number((void *)p_instance->p_reg));
}

void nrfx_uarte_uninit(nrfx_uarte_t const * p_instance)
{
    uarte_control_block_t * p_cb = &m_cb[p_instance->drv_inst_idx];
    NRF_UARTE_Type * p_reg = p_instance->p_reg;

    if (p_cb->handler)
    {
        interrupts_disable(p_instance);
    }
    // Make sure all transfers are finished before UARTE is disabled
    // to achieve the lowest power consumption.
    nrf_uarte_shorts_disable(p_reg, NRF_UARTE_SHORT_ENDRX_STARTRX);

    // Check if there is any ongoing reception.
    if (p_cb->rx_buffer_length)
    {
        nrf_uarte_event_clear(p_reg, NRF_UARTE_EVENT_RXTO);
        nrf_uarte_task_trigger(p_reg, NRF_UARTE_TASK_STOPRX);
    }

    nrf_uarte_event_clear(p_reg, NRF_UARTE_EVENT_TXSTOPPED);
    nrf_uarte_task_trigger(p_reg, NRF_UARTE_TASK_STOPTX);

    // Wait for TXSTOPPED event and for RXTO event, provided that there was ongoing reception.
    bool stopped;

    // The UARTE is able to receive up to four bytes after the STOPRX task has been triggered.
    // On lowest supported baud rate (1200 baud), with parity bit and two stop bits configured
    // (resulting in 12 bits per data byte sent), this may take up to 40 ms.
    NRFX_WAIT_FOR((nrf_uarte_event_check(p_reg, NRF_UARTE_EVENT_TXSTOPPED) &&
                  (!p_cb->rx_buffer_length || nrf_uarte_event_check(p_reg, NRF_UARTE_EVENT_RXTO))),
                  40000, 1, stopped);
    if (!stopped)
    {
        NRFX_LOG_ERROR("Failed to stop instance with base address: %p.", (void *)p_instance->p_reg);
    }

    nrf_uarte_disable(p_reg);
    pins_to_default(p_instance);

#if NRFX_CHECK(NRFX_PRS_ENABLED)
    nrfx_prs_release(p_reg);
#endif

    p_cb->state   = NRFX_DRV_STATE_UNINITIALIZED;
    p_cb->handler = NULL;
    NRFX_LOG_INFO("Instance uninitialized: %d.", p_instance->drv_inst_idx);
}

```
问题二：
执行nrfx_uarte_uninit后，再次执行nrfx_uarte_init, 串口uarte 无法接收数据
观察log 信息，840 仅在第一次调用nrfx_uarte_uninit(p_inst);之前的5秒内可以收到上位机发过来的数据并显示

### 工程环境及编译：
```
Ubuntu 22.04.4 LTS

nRF Connect SDK v2.5.2 
NRF hal库版本：
- name: hal_nordic
  url: https://github.com/nrfconnect/sdk-hal_nordic
  revision: 9784731461018d3e983604698fbbed6af2bea801
  path: modules/hal/nordic


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
