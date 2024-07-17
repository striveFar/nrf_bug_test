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
#define THREAD_STACK_SIZE 2048
K_THREAD_STACK_DEFINE(uarte_send_stack_area, THREAD_STACK_SIZE);
static struct k_thread thread_uarte_send;
#define THREAD_PRIORITY 4




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

/*victor add start*/
static K_SEM_DEFINE(sem_uarte_thread, 0, 1);

static uint8_t rx_buf0[16];
static uint8_t rx_buf1[16];
static uint8_t* p_rx_buf = NULL;

static inline uint8_t* uarte_get_next_rx_buf(uint8_t* pbuf)
{
	uint32_t key;
	key = irq_lock();
	if(pbuf == rx_buf0)
	{
		pbuf = rx_buf1;
	}
	else if(pbuf == rx_buf1)
	{
		pbuf = rx_buf0;
	}
	else
	{
		pbuf = rx_buf0;
	}

	irq_unlock(key);

	return pbuf;
}

struct uart_data_msg{
	size_t len;
    uint8_t buf[16];
};

K_MSGQ_DEFINE(uart_rx_msgq, sizeof(struct uart_data_msg), 4, 4);

#define UARTE_WORK_QUEUE_STACK_SIZE 1024
#define UARTE_WORK_QUEUE_PRIORITY 4

K_THREAD_STACK_DEFINE(uarte_work_queue_stack_area, UARTE_WORK_QUEUE_STACK_SIZE);

struct k_work_q uarte_cmd_work_q;
static void cmd_restart_work_func(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(cmd_restart_work, cmd_restart_work_func);

#define UARTE_RX_THREAD_STACK_SIZE 1024
#define UARTE_RX_THREAD_PRIORITY 5
void uarte_rx_thread_task_function(void *, void *, void *);
K_THREAD_DEFINE(uarte_rx_thread, UARTE_RX_THREAD_STACK_SIZE,
                uarte_rx_thread_task_function, NULL, NULL, NULL,
                UARTE_RX_THREAD_PRIORITY, 0, 0);



static void cmd_restart_work_func(struct k_work *work)
{
	nrf_gpio_pin_toggle(16);
	uint32_t error;
	nrfx_uarte_rx_abort(&uarte_inst, 0, 0);

	error = nrfx_uarte_errorsrc_get(&uarte_inst);
	printk("nrfx_uarte_errorsrc_get return: %d\n", error);

	k_msgq_purge(&uart_rx_msgq);

	p_rx_buf = uarte_get_next_rx_buf(p_rx_buf);
	nrfx_uarte_rx(&uarte_inst, p_rx_buf, sizeof(split_msgq_trans_t));
}


static void uarte_recv_work_func(struct uart_data_msg* p_msg)
{
	nrf_gpio_pin_toggle(13);
	nrfx_err_t error = 0;

	if(p_msg->len != sizeof(split_msgq_trans_t))
	{
		printk("p_msg->len = %d\n", p_msg->len);
		nrf_gpio_pin_toggle(14);
	}

	if(error)
	{
		k_work_schedule_for_queue(&uarte_cmd_work_q, &cmd_restart_work, K_USEC(1000));
	}
	LOG_HEXDUMP_INF(p_msg->buf, p_msg->len, " ");
}


void uarte_rx_thread_task_function(void *, void *, void *)
{
	struct uart_data_msg data;
	printk("uarte_thread_task_function\n");

	while(1)
	{
		//k_sem_take(&sem_uarte_thread, K_FOREVER);
		k_msgq_get(&uart_rx_msgq, &data, K_FOREVER);
		uarte_recv_work_func(&data);
	}
}
/*victor add end*/


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
	struct uart_data_msg data;

	nrfx_uarte_t *p_inst = p_context;


	switch (p_event->type) {
	case NRFX_UARTE_EVT_RX_DONE:
		p_rx_buf = uarte_get_next_rx_buf(p_event->data.rx.p_data);
		status = nrfx_uarte_rx(&uarte_inst, p_rx_buf, sizeof(split_msgq_trans_t));

		data.len = p_event->data.rx.bytes;
		memcpy(data.buf, p_event->data.rx.p_data, data.len);
        while (k_msgq_put(&uart_rx_msgq, &data, K_NO_WAIT) != 0) {
            /* message queue is full: purge old data & try again */
            k_msgq_purge(&uart_rx_msgq);
			k_msgq_put(&uart_rx_msgq, p_event->data.rx.p_data, K_NO_WAIT);
        }
		break;

	case NRFX_UARTE_EVT_TX_DONE:
		break;

	case NRFX_UARTE_EVT_ERROR:
		nrf_gpio_pin_toggle(15);
		k_work_schedule_for_queue(&uarte_cmd_work_q, &cmd_restart_work, K_USEC(1000));
		break;

	default:
		break;
	}
}

void uarte_send_task(void *p1, void *p2, void *p3)
{
	static split_msgq_trans_t packet;

	LOG_INF("uarte_send_task start!");
	static uint32_t cnt;
	static bool toggle_state;

	while (1) {
		if (++cnt < 500)
			k_msleep(10);
		else {
			toggle_state = !toggle_state;
			cnt = 0;
			if (toggle_state) {
				split_uarte_deinit(&uarte_inst);
				LOG_INF("deinit uarte");
			} else {
				split_uart_init(&uarte_inst);
				LOG_INF("init uarte");
			}

		}
	}
}

void thread_init_and_start(void)
{
	k_thread_create(&thread_uarte_send, uarte_send_stack_area,
			K_THREAD_STACK_SIZEOF(uarte_send_stack_area), uarte_send_task, NULL, NULL,
			NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&thread_uarte_send, "uarte_send_task");

	k_thread_start(&thread_uarte_send);
}

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
		IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_UARTE_INST_GET(UARTE_INST_IDX)),
				   IRQ_PRIO_LOWEST,
				   NRFX_UARTE_INST_HANDLER_GET(UARTE_INST_IDX),
				   NULL,
				   0);
#endif

		/* 接收完成32byte进中断，获得首次进入中断的机会 */
		p_rx_buf = uarte_get_next_rx_buf(p_rx_buf);
		status = nrfx_uarte_rx(&uarte_inst, p_rx_buf, sizeof(split_msgq_trans_t));
		NRFX_ASSERT(status == NRFX_SUCCESS);
		key = irq_lock();
		uart_status = true;
		irq_unlock(key);
	}
}

static void split_uarte_deinit(nrfx_uarte_t *p_inst)
{
	uint32_t key = irq_lock();
	nrfx_uarte_uninit(p_inst);
	irq_unlock(key);

}

/**
 * @brief Function for application main entry.
 *
 * @return Nothing.
 */
int kbd_split_uarte_init(void)
{
	k_work_queue_init(&uarte_cmd_work_q);

	k_work_queue_start(&uarte_cmd_work_q, uarte_work_queue_stack_area,
						K_THREAD_STACK_SIZEOF(uarte_work_queue_stack_area),
						UARTE_WORK_QUEUE_PRIORITY,
						NULL);

	split_uart_init(&uarte_inst);

	LOG_INF("uarte_sample start!");
	thread_init_and_start();

	return 0;
}

/** @} */
