#include <stdio.h>
#include <nrfx_spim.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include "MAX86178/max86178.h"
#include <zephyr/sys/printk.h>
#include "drivers/EASYQ/easyq.h"
#include "drivers/EASYQ/cmd_manager.h"
#include "utilities/mxc_errors.h"
#include <zephyr/irq.h>
#include <zephyr/devicetree.h>

#define GPIO0_NODE DT_NODELABEL(gpio0)
#define GPIO_PIN_8 8
#define GPIO_PIN_9 9

static const struct device *gpio0_dev;

#define SPI1_NODE DT_NODELABEL(max86178)

#define SPIOP SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_LINES_SINGLE

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(SPI1_NODE, SPIOP, 0);

/*SETUP semaforo*/
K_SEM_DEFINE(spi_sem, 0, 1); // Inizializza un semaforo con il valore iniziale 0 e il valore massimo 1

/*SETUP msgq*/
K_MSGQ_DEFINE(cmd_event_msgq, sizeof(struct cmd_item), CMD_EVENT_QUEUE_SIZE, 1);

/* 2200 msec = 2.2 sec */
#define PRODUCER_SLEEP_TIME_MS 2200
#define STACKSIZE 2048
#define PRODUCER_THREAD_PRIORITY 7
#define CONSUMER_THREAD_PRIORITY 7
#define MAX_DATA_SIZE 32
#define MIN_DATA_ITEMS 4
#define MAX_DATA_ITEMS 14
/*  Define the data type of the FIFO items*/
struct data_item_t
{
	void *fifo_reserved;
	uint8_t data[MAX_DATA_SIZE];
	uint8_t reg_addr;
	uint16_t len;
};

struct data_item
{
	void *fifo_reserved;
	uint8_t cmd1;
	uint8_t cmd2;
	uint8_t content1;
	uint8_t reg_addr;
	uint16_t len;
};

static easyq_t *m_easyq = NULL;
static sensor_t *biosensor;
static bool sensor_enabled = false;
static bool sensor_started = false;
static bool start_read = false;

int app = 0;

void timer_expiry_fn(struct k_timer *dummy)
{
	if (app == 0)
	{
		struct cmd_item evt = {
			.cmd = CMD_TYPE_INIT_MAX86178,
			.value = 1234,
		};
		cmd_event_manager_put(&evt, &cmd_event_msgq);
		app++;
	};
	if (app == 1)
	{
		struct cmd_item evt = {
			.cmd = CMD_TYPE_RUN_MAX86178,
			.value = 1234,
		};
		cmd_event_manager_put(&evt, &cmd_event_msgq);
		// app++;
	};
};
/*Define the timer*/
K_TIMER_DEFINE(timer, timer_expiry_fn, NULL);

/*  Define the FIFO */
K_FIFO_DEFINE(my_fifo);

#define STACKSIZE 1024
#define STACKSIZE2 2048
#define PRIORITY 7

void thread_FIFO_MAX86178(void *dummy1, void *dummy2, void *dummy3)
{
	while (1)
	{
		struct max86178_dev *max86178_driver = max86178_get_device_data();
		//max86178_sensor_enable(max86178_driver, 1, 1); // avvio sensore max86178

		max86178_fifo_irq_handler(max86178_driver); // gestione interrupt sensore max86178

		// if(max86178_get_irq_state(NULL) > 0) {
		if (max86178_irq_handler(max86178_driver) > 0)
		{
			max86178_irq_reset_ref_cnt();

			// printf("Queue sample count: %d\r\n", sd->queue.num_item);
		}
		//max86178_irq_reset_ref_cnt();

		k_sleep(K_SECONDS(5));
	}
}

K_THREAD_STACK_DEFINE(thread_stack_area, STACKSIZE2);
struct k_thread thread_data;

void gpio_callback_1(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
   // printk("GPIO interrupt on pin 8 triggered!\n");
    // Gestisci l'interrupt GPIO 8 qui
	if (sensor_started == true)
	{
		start_read = true;


	}
}

void gpio_callback_2(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
   // printk("GPIO interrupt on pin 9 triggered!\n");
    // Gestisci l'interrupt GPIO 9 qui
}

static struct gpio_callback gpio_cb_INT1;
static struct gpio_callback gpio_cb_2;

/**
 * @brief Function for application main entry.
 *
 * @return Nothing.
 */
int main(void)
{

	if (!device_is_ready(spispec.bus))
	{
		printk("SPI not ready");
		return -ENODEV;
	}

	gpio0_dev = device_get_binding(DT_LABEL(GPIO0_NODE));
    if (!gpio0_dev) {
        printk("Error: didn't find %s device\n", DT_LABEL(GPIO0_NODE));
        return;
    }

	// Configura GPIO 8 come input con interrupt su rising edge
    gpio_pin_configure(gpio0_dev, GPIO_PIN_8, GPIO_INPUT | GPIO_PULL_UP);
    gpio_pin_interrupt_configure(gpio0_dev, GPIO_PIN_8, GPIO_INT_EDGE_RISING);
	gpio_init_callback(&gpio_cb_INT1, gpio_callback_1, BIT(GPIO_PIN_8));
	//gpio_init_callback(&gpio_cb_INT1, max86178_irq_handler_fast, BIT(GPIO_PIN_8));
    gpio_add_callback(gpio0_dev, &gpio_cb_INT1);


    // Configura GPIO 9 come input con interrupt su rising edge
    gpio_pin_configure(gpio0_dev, GPIO_PIN_9, GPIO_INPUT | GPIO_PULL_UP);
	gpio_pin_interrupt_configure(gpio0_dev, GPIO_PIN_9, GPIO_INT_EDGE_RISING);
    gpio_init_callback(&gpio_cb_2, gpio_callback_2, BIT(GPIO_PIN_9));
    gpio_add_callback(gpio0_dev, &gpio_cb_2);
    

    // Abilita gli interrupt GPIO
    irq_enable(DT_IRQN(DT_NODELABEL(gpio0)));

	struct cmd_item cmd_evt;
	printk("Start!");
	k_timer_start(&timer, K_SECONDS(10), K_SECONDS(10));

	m_easyq = geteasyq();
	m_easyq->init();
	biosensor = m_easyq->getSensor(SH_MAX86178);

	while (1)
	{
		int ret_msg;
		ret_msg = cmd_event_manager_get(&cmd_evt, &cmd_event_msgq);

		if (start_read)
		{
				struct max86178_dev *max86178_driver = max86178_get_device_data();
	        //     max86178_fifo_irq_handler(max86178_driver); // gestione interrupt sensore max86178


				 //	if(max86178_get_irq_state(NULL) > 0) {
		if (max86178_irq_handler(max86178_driver) > 0)
			max86178_irq_reset_ref_cnt();

		//printf("Queue sample count: %d\r\n", sd->queue.num_item);
	//}
				 start_read = false;
		}

		if (ret_msg == E_NO_ERROR)
		{
			printk("Event received! type: %d \n", cmd_evt.cmd);
			switch (cmd_evt.cmd)
			{
			case CMD_TYPE_INIT_PMU:
				printk("Init PMU event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_RUN_PMU:
				printk("Run PMU event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_INIT_ACC:
				printk("Init ACC event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_RUN_ACC:
				printk("Run ACC event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_INIT_MAX86178:
				max86178_init(&spispec); // init sensore max86178
				sensor_started = true;
				break;
			case CMD_TYPE_RUN_MAX86178:
				 struct max86178_dev *max86178_driver = max86178_get_device_data();
				//  max86178_sensor_enable(max86178_driver, 1, 1); // avvio sensore max86178


				
				// if (sensor_enabled == false)
				// {
				// //struct max86178_dev *max86178_driver = max86178_get_device_data();
				//     max86178_sensor_enable(max86178_driver, 1, 1); // avvio sensore max86178
				// 	max86178_set_frame_ready_int(1);
					
				// 	//max86178_set_a_full_int(1);
				// 	max86178_init_fifo(max86178_driver, 0x0F);
				// 	sensor_enabled = true;

				// 									k_tid_t my_tid1 = k_thread_create(&thread_data, thread_stack_area,
				//                      K_THREAD_STACK_SIZEOF(thread_stack_area),
				//                      thread_FIFO_MAX86178,
				//                      NULL, NULL, NULL,
				//                      PRIORITY, 0, K_NO_WAIT);
				// }
				// //max86178_fifo_irq_handler(max86178_driver); // gestione interrupt sensore max86178

				if (sensor_enabled == false)
				{
									    max86178_sensor_enable(max86178_driver, 1, 1); // avvio sensore max86178
					//max86178_set_frame_ready_int(1);
					
					//max86178_set_a_full_int(1);
					//max86178_init_fifo(max86178_driver, 0x0F);
					sensor_enabled = true;
					// // Configura GPIO 8 come input con interrupt su rising edge
					// gpio_pin_configure(gpio0_dev, GPIO_PIN_8, GPIO_INPUT | GPIO_PULL_UP);
					// gpio_pin_interrupt_configure(gpio0_dev, GPIO_PIN_8, GPIO_INT_EDGE_RISING);
					// gpio_init_callback(&gpio_cb_INT1, max86178_irq_handler_fast, BIT(GPIO_PIN_8));
					// gpio_add_callback(gpio0_dev, &gpio_cb_INT1);


					// // Configura GPIO 9 come input con interrupt su rising edge
					// gpio_pin_configure(gpio0_dev, GPIO_PIN_9, GPIO_INPUT | GPIO_PULL_UP);
					// gpio_pin_interrupt_configure(gpio0_dev, GPIO_PIN_9, GPIO_INT_EDGE_RISING);
					// gpio_init_callback(&gpio_cb_2, gpio_callback_2, BIT(GPIO_PIN_9));
					// gpio_add_callback(gpio0_dev, &gpio_cb_2);
					

					// // Abilita gli interrupt GPIO
					// irq_enable(DT_IRQN(DT_NODELABEL(gpio0)));
				}


				break;
			case CMD_TYPE_RUN_ECG:
				printk("Run ECG event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_INIT_FLASH:
				printk("Init FLASH event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_RUN_FLASH:
				printk("Run FLASH event! code: %d", cmd_evt.cmd);
				break;
			case CMD_TYPE_STOP:
				printk("Stop event! code: %d", cmd_evt.cmd);
				// suspend thread my_tid
				//	k_thread_suspend(my_tid1);
				break;
			default:
				printk("Unknown cmd type: %d", cmd_evt.cmd);
				break;
			}
		}
		k_msleep(100);
	}

	// k_msleep(10);
	// printk("spi  ready");
	// max86178_init(&spispec);

	// k_msleep(10);
	// uint8_t reg_addr = 0x20;
	// uint8_t data1 = 0x21;
	// max86178_write_reg(&spispec, reg_addr, &data1, sizeof(data1));
	// k_msleep(10);
	// data1 = reg_addr;
	// max86178_read_reg(&spispec, &data1, sizeof(data1));
	// printk("max86178_read_reg  %d %d \n", reg_addr, data1);
	// k_msleep(10);

	// m_easyq = geteasyq();
	// m_easyq->init();

	// while (1)
	// {
	// 	int ret = 0;
	// 	ret = spi_rw_test(&spispec);
	// 	k_msleep(1000);

	// }

	// while (1)
	// {
	// 	producer();
	// 	k_msleep(20);
	// 	consumer();
	// 	k_msleep(1000);
	// }
}

// static void producer(void)
// {
// 		int bytes_written;

// 		for (uint8_t i = 32; i <= 36; i++) {
// 			/* Create a data item to send */
// 			if (i == 32 || i == 35 || i == 36)
// 			{
// 				struct data_item_t *buf = k_malloc(sizeof(struct data_item_t));
// 				if (buf == NULL) {
// 					/* Unable to locate memory from the heap */
// 					printk("Enable to allocate memory");
// 					return;
// 				}
// 				if (i == 32)
// 				{
// 					buf->data[0] = 0x21;
// 				}
// 				else if (i == 35)
// 				{
// 					buf->data[0] = 0x18;
// 				}
// 				else if (i == 36)
// 				{
// 					buf->data[0] = 0x3F;
// 				}
// 				max86178_write_reg(&spispec, i, &buf->data[0], sizeof(buf->data[0]));
// 				//buf->data[0] = i;
// 			// printk("Max86178_Write_Byte  %d %d\n",  i, i);
// 				//max86178_read_reg(&spispec, buf->data, sizeof(buf->data));
// 				//bytes_written = snprintf(buf->data, MAX_DATA_SIZE, "Max86178_Write_Byte  %d %d\n",  i, buf->data,
// 				//			 dataitem_count, 32);
// 				//buf->len = bytes_written;
// 				buf->reg_addr = i;
// 				k_fifo_put(&my_fifo, buf);
// 				//k_msleep(20);
// 		    }

// 		}
// }
// static void consumer(void)
// {

// 	while (1)
// 	{

// 		struct data_item_t *rec_item = k_fifo_get(&my_fifo, K_NO_WAIT);
// 		if (rec_item){
// 			uint8_t data[1];
// 			data[0] = rec_item->reg_addr;
// 			max86178_read_reg(&spispec, data, sizeof(data));
// 			//k_msleep(20);
// 			k_free(rec_item);
// 		}
// 		else
// 		{
// 			break;
// 		}
// 	}
// }

// thread disabled temporary

// static void producer_func(void *unused1, void *unused2, void *unused3)
// {
// 	ARG_UNUSED(unused1);
// 	ARG_UNUSED(unused2);
// 	ARG_UNUSED(unused3);
// 	static uint32_t dataitem_count = 0;
// 	/*STEP 5  - Complete the producer thread functionality  */
// 	while (1) {
// 		int bytes_written;

// 		for (uint8_t i = 32; i <= 36; i++) {
// 			/* Create a data item to send */
// 			if (i == 32 || i == 35 || i == 36)
// 			{
// 				struct data_item_t *buf = k_malloc(sizeof(struct data_item_t));
// 				if (buf == NULL) {
// 					/* Unable to locate memory from the heap */
// 					printk("Enable to allocate memory");
// 					return;
// 				}
// 				if (i == 32)
// 				{
// 					buf->data[0] = 0x21;
// 				}
// 				else if (i == 35)
// 				{
// 					buf->data[0] = 0x18;
// 				}
// 				else if (i == 36)
// 				{
// 					buf->data[0] = 0x3F;
// 				}
// 				max86178_write_reg(&spispec, i, &buf->data[0], sizeof(buf->data[0]));

// 				buf->reg_addr = i;
// 				dataitem_count++;
// 				k_fifo_put(&my_fifo, buf);
// 				//k_msleep(20);
// 		    }

// 		}
// 		k_sem_give(&spi_sem);  // Rilascia il semaforo dopo aver scritto su SPI
// 		//printk("Producer: Data Items Generated: %u", data_number);
// 		//k_msleep(PRODUCER_SLEEP_TIME_MS);
// 		//k_yield(); // per attivare altro thread
// 		k_msleep(1000);
// 	}

// }

// static void consumer_func(void *unused1, void *unused2, void *unused3)
// {
//  		k_sem_take(&spi_sem, K_FOREVER);  // Prende il semaforo. Se il semaforo è 0, il thread si blocca fino a quando il semaforo non viene rilasciato

// 	ARG_UNUSED(unused1);
// 	ARG_UNUSED(unused2);
// 	ARG_UNUSED(unused3);
// 	/*STEP 6 - Complete the consumer thread functionality */
// 	while (1) {

// 		// struct data_item_t *rec_item;
//         // rec_item = k_fifo_get(&my_fifo, K_FOREVER);
// 		// uint8_t data[1];
// 		// data[0] = rec_item->reg_addr;
// 		// max86178_read_reg(&spispec, data, sizeof(data));
// 		// //printk("max86178_read_reg  %d %d \n", rec_item->reg_addr, data[1]);
// 		// k_free(rec_item);

// 		struct data_item_t *rec_item = k_fifo_get(&my_fifo, K_FOREVER);
// 		if (rec_item){
// 			uint8_t data[1];
// 			data[0] = rec_item->reg_addr;
// 			max86178_read_reg(&spispec, data, sizeof(data));
// 			//k_msleep(20);
// 			// uint8_t app1=data[0];
// 			// uint8_t app2=rec_item->data[0];
// 			if (data[0] == rec_item->data[0])
// 			{
// 				printk("read ok\n");
// 			}
// 			k_free(rec_item);
// 		}

// 	}

// }

// K_THREAD_DEFINE(producer, STACKSIZE, producer_func, NULL, NULL, NULL, PRODUCER_THREAD_PRIORITY, 0, 0);
// K_THREAD_DEFINE(consumer, STACKSIZE, consumer_func, NULL, NULL, NULL, CONSUMER_THREAD_PRIORITY, 0, 0);
