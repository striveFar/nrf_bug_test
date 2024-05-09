#include <stdint.h>
#include <zephyr/kernel.h>
#include <nrfx_uarte.h>
#include <hal/nrf_gpio.h>
#include "uarte_split_trans.h"
#include <haly/nrfy_gpio.h>
//#include <nrfx_log.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uarte);
typedef struct split_msgq_trans
{
	/* transaction id */
	uint8_t id;
	/* note: only real data len! */
	uint8_t len;
	/* max data */
	uint8_t data[8];
	/* crc8: [id~data] */
	uint8_t crc8;

} __attribute__((packed)) split_msgq_trans_t;

#define RINGBUFF_SIZE 50

// #define NRFX_LOG_MODULE EXAMPLE
// #define NRFX_EXAMPLE_CONFIG_LOG_ENABLED 1
// #define NRFX_EXAMPLE_CONFIG_LOG_LEVEL 3

static void split_uart_init(nrfx_uarte_t *p_inst);
static void split_uarte_deinit(nrfx_uarte_t *p_inst);

/* Thread define */
//#define THREAD_STACK_SIZE 2048 
//K_THREAD_STACK_DEFINE(uarte_send_stack_area, THREAD_STACK_SIZE);
//static struct k_thread thread_uarte_send;
//#define THREAD_PRIORITY 4

static void uarte_recv_work_func(struct k_work *work);

K_WORK_DELAYABLE_DEFINE(uarte_work, uarte_recv_work_func);

/** @brief Symbol specifying UARTE instance to be used. */
#define UARTE_INST_IDX 1

/** @brief Symbol specifying TX pin number of UARTE. */
#define UARTE_TX_PIN NRF_GPIO_PIN_MAP(0, 27)

/** @brief Symbol specifying RX pin number of UARTE. */
#define UARTE_RX_PIN NRF_GPIO_PIN_MAP(0, 26)

static nrfx_uarte_t uarte_inst = NRFX_UARTE_INSTANCE(UARTE_INST_IDX);

static bool uart_status;

typedef struct {
	split_msgq_trans_t buff[RINGBUFF_SIZE];
	uint8_t w_pos;
	uint8_t r_pos;
} rx_buffers_t;

/** @brief Structure containing receive buffers. */
static rx_buffers_t m_rx_buffers = { 0 };
static bool bad_crc;

static void uarte_recv_work_func(struct k_work *work)
{
	uint8_t i = 0;
	uint8_t interval = 0;
	split_msgq_trans_t *ptr;
	nrfx_err_t status;

	// 处理是始终落后于接收的，所以当 w==r 时，必然是w已经写了一圈了，要处理所有

	interval = (m_rx_buffers.r_pos < m_rx_buffers.w_pos) ?
			   (m_rx_buffers.w_pos - m_rx_buffers.r_pos) :
			   (RINGBUFF_SIZE + m_rx_buffers.w_pos - m_rx_buffers.r_pos);

	for (i = 0; i < interval; i++) {
		ptr = m_rx_buffers.buff + ((m_rx_buffers.r_pos + i) % RINGBUFF_SIZE);
		LOG_HEXDUMP_INF(ptr->data, ptr->len, " ");
	}
	m_rx_buffers.r_pos = (m_rx_buffers.r_pos + interval) % RINGBUFF_SIZE;
	LOG_INF("w: %d, r: %d", m_rx_buffers.w_pos, m_rx_buffers.r_pos);
}

/**
 * @brief Function for handling UARTE driver events.
 *
 * @param[in] p_event   Pointer to event structure. Event is allocated on the stack so it is available
 *                      only within the context of the event handler.
 * @param[in] p_context Context passed to the interrupt handler, set on initialization. In this example
 *                      p_context is used to pass the address of the UARTE instance that calls this handler.
 */
static void uarte_handler(nrfx_uarte_event_t const *p_event, void *p_context)
{
	nrfx_err_t status;
	(void)status;
	uint8_t index;
	bool debug_after_err = false;

	nrfx_uarte_t *p_inst = p_context;

	switch (p_event->type) {
	case NRFX_UARTE_EVT_RX_DONE:
		/* 测试EVT_ERR之后是否还会进NRFX_UARTE_EVT_RX_DONE */
		if (debug_after_err) {
			debug_after_err = false;
			LOG_ERR("Enter RX_DONE after EVT_ERR\n");
		}
		
		index = (++m_rx_buffers.w_pos % RINGBUFF_SIZE);
		status = nrfx_uarte_rx(p_inst, (uint8_t *)(m_rx_buffers.buff + index),
				       sizeof(struct split_msgq_trans));
		NRFX_ASSERT(status == NRFX_SUCCESS);
		m_rx_buffers.w_pos = index;
		if (!bad_crc) {
			k_work_schedule(&uarte_work, K_NO_WAIT);
		} else {
			struct k_work_sync sync;
			k_work_cancel_delayable_sync(&uarte_work, &sync);
			split_uarte_deinit(p_inst);
			split_uart_init(p_inst);
			m_rx_buffers.r_pos = 0;
			m_rx_buffers.w_pos = 0;
			memset(m_rx_buffers.buff, 0, sizeof(m_rx_buffers.buff));
			LOG_ERR("crc err");
			bad_crc = 0;
		}

		break;
	case NRFX_UARTE_EVT_TX_DONE:
		LOG_INF("--> TX done");
		LOG_INF("--> Bytes transfered: %u", p_event->data.tx.bytes);

		break;

	case NRFX_UARTE_EVT_ERROR:
		uint32_t rxto = 0;
		uint32_t endrx = 0;
		/* 反复插拔RX引脚，一旦出现错误，就会疯狂进错误处理，清除不掉 */
		LOG_ERR("--> UARTE Error: %d", p_event->data.error.error_mask);
		LOG_ERR("--> Rx bytes: %d", p_event->data.error.rx.bytes);
		// LOG_ERR("e\n");
		struct k_work_sync sync;
		k_work_cancel_delayable_sync(&uarte_work, &sync);

		m_rx_buffers.r_pos = 0;
		m_rx_buffers.w_pos = 0;
		memset(m_rx_buffers.buff, 0, sizeof(m_rx_buffers.buff));

		*((volatile uint32_t *)((uint8_t *)p_inst->p_reg + (uint32_t)NRF_UARTE_TASK_FLUSHRX)) = 0x1UL;

		status = nrfx_uarte_rx(&uarte_inst, (uint8_t *)m_rx_buffers.buff,
				       sizeof(split_msgq_trans_t));
		NRFX_ASSERT(status == NRFX_SUCCESS);

		debug_after_err = true;

		/* The UARTE receiver is stopped by triggering the STOPRX task. 
		* An RXTO event is generated when the UARTE has stopped.
		*/
		//*((volatile uint32_t *)((uint8_t *)p_inst->p_reg + (uint32_t)NRF_UARTE_TASK_STOPRX)) = 0x1UL;
    		//uint8_t stop_rx = *((uint8_t *)p_inst->p_reg + (uint32_t)NRF_UARTE_TASK_STOPRX);
		/* 不可读，读出为0 */
		//LOG_ERR("stop_rx: %d\n", stop_rx);
		// while (rxto || !endrx)
		// {
		// 	LOG_ERR("Watting NRF_UARTE_EVENT_RXTO\n");
		// 	uint32_t *rxto_p = (uint32_t *)((uint8_t *)p_inst->p_reg + (uint32_t)NRF_UARTE_EVENT_RXTO);
		// 	uint32_t *endrx_p = (uint32_t *)((uint8_t *)p_inst->p_reg + (uint32_t)NRF_UARTE_EVENT_ENDRX);
		// 	rxto = *rxto_p;
		// 	endrx = *endrx_p;
		// 	LOG_ERR("EVENT_RXTO: %d, EVENT_ENDRX: %d\n", rxto, endrx);
		// }
		
		//*((volatile uint32_t *)((uint8_t *)p_inst->p_reg + (uint32_t)NRF_UARTE_TASK_STARTRX)) = 0x1UL;


		// /* 尝试清除errorsrc */
        	// while (nrfy_uarte_errorsrc_get_and_clear(p_inst->p_reg)) {
		// 	LOG_ERR("c\n");
		// }

		// /* 尝试清除所有EVENT寄存器 */
		// uint32_t mask =
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_CTS) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_NCTS) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXDRDY) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDRX) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXDRDY) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDTX) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXSTARTED) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTARTED) |
		// 	NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED);

		// while (!__nrfy_internal_uarte_events_process(p_inst->p_reg, mask, NULL)) {
		// 	LOG_ERR("mask = 0x%x", mask);
		// }
		//split_uarte_deinit(p_inst);
		//split_uart_init(p_inst);
		break;

	default:
		break;
	}
}
//
//void uarte_send_task(void *p1, void *p2, void *p3)
//{
//	static split_msgq_trans_t packet;
//
//	LOG_INF("uarte_send_task start!");
//
//	while (1) {
//		if (uart_status && !nrfx_uarte_tx_in_progress(&uarte_inst)) {
//			if (!split_data_msgq_get(&packet)) {
//				nrfx_uarte_tx(&uarte_inst, (uint8_t *)&packet,
//					      sizeof(split_msgq_trans_t), 0);
//				k_msleep(1);
//			}
//		} else {
//			k_msleep(1);
//		}
//	}
//}

//void thread_init_and_start(void)
//{
//	k_thread_create(&thread_uarte_send, uarte_send_stack_area,
//			K_THREAD_STACK_SIZEOF(uarte_send_stack_area), uarte_send_task, NULL, NULL,
//			NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
//	k_thread_name_set(&thread_uarte_send, "uarte_send_task");
//
//	k_thread_start(&thread_uarte_send);
//}

static void split_uart_init(nrfx_uarte_t *p_inst)
{
	nrfx_err_t status;
	uint32_t key;
	if (false == uart_status) {
		nrfx_uarte_config_t uarte_config =
			NRFX_UARTE_DEFAULT_CONFIG(UARTE_TX_PIN, UARTE_RX_PIN);
		uarte_config.baudrate = NRF_UARTE_BAUDRATE_1000000;
		uarte_config.p_context = p_inst;
		status = nrfx_uarte_init(p_inst, &uarte_config, uarte_handler);
		NRFX_ASSERT(status == NRFX_SUCCESS);
		/* note: 在nrfx_uarte_init之后设置上拉才生效，否则被 nrfx_uarte_init 的默认无拉覆盖 */
		nrfy_gpio_cfg_input(UARTE_RX_PIN,  NRF_GPIO_PIN_PULLUP);
		nrfy_gpio_cfg_input(UARTE_TX_PIN,  NRF_GPIO_PIN_PULLUP);
		// nrfy_gpio_cfg(UARTE_RX_PIN, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
		// 	      NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
		// nrfy_gpio_cfg(UARTE_TX_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT,
		// 	      NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
#if defined(__ZEPHYR__)
		IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_UARTE_INST_GET(UARTE_INST_IDX)),
				   IRQ_PRIO_LOWEST, NRFX_UARTE_INST_HANDLER_GET(UARTE_INST_IDX), 0);
#endif

		/* 接收完成32byte进中断，获得首次进入中断的机会 */
		status = nrfx_uarte_rx(&uarte_inst, (uint8_t *)m_rx_buffers.buff,
				       sizeof(split_msgq_trans_t));
		NRFX_ASSERT(status == NRFX_SUCCESS);
		key = irq_lock();
		uart_status = true;
		irq_unlock(key);
	}
}

static void split_uarte_deinit(nrfx_uarte_t *p_inst)
{
	if (uart_status) {
		uint32_t key = irq_lock();
		uart_status = false;
		nrfx_uarte_uninit(p_inst);
		m_rx_buffers.r_pos = 0;
		m_rx_buffers.w_pos = 0;
		memset(m_rx_buffers.buff, 0, sizeof(m_rx_buffers.buff));

		uint32_t mask =
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_CTS) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_NCTS) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXDRDY) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDRX) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXDRDY) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ENDTX) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_ERROR) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXTO) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_RXSTARTED) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTARTED) |
			NRFY_EVENT_TO_INT_BITMASK(NRF_UARTE_EVENT_TXSTOPPED);

		while (!__nrfy_internal_uarte_events_process(p_inst->p_reg, mask, NULL)) {
			LOG_ERR("mask = 0x%x", mask);
		}

		irq_unlock(key);
	}
}

/**
 * @brief Function for application main entry.
 *
 * @return Nothing.
 */
int kbd_split_uarte_init(void)
{
	split_uart_init(&uarte_inst);

	LOG_INF("uarte_sample start!");
	//thread_init_and_start();

	return 0;
}

/** @} */
