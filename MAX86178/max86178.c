#include "max86178.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "../../utilities/Peripherals.h"
#include "../../utilities/device.h"

// #define MAX86178_PART_ID_REG 0xFE
#define MAX86178_PART_ID 0x43
#define SPI_READ_CMD 0x80
#define SPI_WRITE_CMD 0x00
static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);
static max86178_fifo_read_cb fifo_read_cb;
char device_info[sizeof(struct max86178_dev)];

extern uint8_t gUseAcc;

static void *max86178_device_mem(void *dev_ptr)
{
	static void *dev_data_ptr;

	if (dev_ptr)
		dev_data_ptr = dev_ptr;
	return dev_data_ptr;
}

void *max86178_get_device_data(void)
{
	return max86178_device_mem(NULL);
}

static void *max86178_set_device_data(void *dev_ptr)
{
	return max86178_device_mem(dev_ptr);
}

int max86178_read_reg(const struct spi_dt_spec *spi_dev, uint8_t *data, uint16_t len);

int max86178_init(const struct spi_dt_spec *spi_dev)
{
	int ret = 0;
	uint8_t part_id, rev_id, num_pd;
	printk("max86178_init\n");
	struct max86178_dev *handle;
	ret =max86178_get_part_info(spi_dev,  &part_id, &rev_id, &num_pd);
		if (ret < 0) {
		printk("MAX86178 is not detected. Part_id: 0x%X, Rev_Id: 0x%X\n", part_id, rev_id);
		goto fail;
	}
	printk("MAX86178 detected. Part_id: 0x%X, Rev_Id: 0x%X\n", part_id, rev_id);

	handle = (void *)device_info;
		if (handle == NULL) {
		printk("%s:%d - Out of memory\n", __func__, __LINE__);
		ret = -2;
		goto fail;
	}

	memset(handle, 0, sizeof(struct max86178_dev));
	handle->part_id = part_id;
	handle->rev_id = rev_id;
	handle->num_pd = num_pd;
	max86178_set_device_data(handle);
	handle->spi_dev = spi_dev;
	handle->dac_calib_status = 0x00;
	handle->operating_cfg.dac_calib = 0;
	handle->operating_cfg.firmware_default = 1;
	ret = max86178_startup_init(handle);
	if (ret < 0) {
		ret = -3;
		goto fail;
	}
	return ret;
fail:
	printk("Init failed %s:%d\n", __func__, __LINE__, ret);
	return ret;
}

int max86178_write_reg(const struct spi_dt_spec *spi_dev, uint8_t reg_addr, uint8_t *data, uint32_t len)
{
	uint8_t tx_buffer[2 + len];
	tx_buffer[0] = reg_addr;
	tx_buffer[1] = SPI_WRITE_CMD;
	memcpy(&tx_buffer[2], data, len);

	struct spi_buf tx_spi_bufs[] = {
		{.buf = tx_buffer, .len = sizeof(tx_buffer)},

	};

	struct spi_buf_set spi_tx_buffer_set = {
		.buffers = tx_spi_bufs,
		.count = 1,
	};

	int ret;

	ret = spi_write(spi_dev->bus, (const struct spi_config *)&spi_dev->config, &spi_tx_buffer_set);

	if (ret != 0)
	{
		printk("spi_write error: %i\n", ret);
	}
	else
	{
		for (int i = 0; i < len; i++)
		{
			printk("max86178_write_reg  %d %d \n", reg_addr, data[i]);
			reg_addr++;
		}
	}

	return ret;
}

// int max86178_write_reg(const struct spi_dt_spec *spi_dev, uint8_t reg_addr,uint8_t *data, uint32_t len)
// {
//     uint8_t tx_buffer[2+len];
//     tx_buffer[0] = reg_addr;
//     tx_buffer[1] = SPI_WRITE_CMD;
//     memcpy(&tx_buffer[2], data, len);
//     printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[2]);
//     uint8_t rx_buf[2+len];
//     struct spi_buf tx_spi_bufs[] = {
//         {.buf = tx_buffer, .len = sizeof(tx_buffer)},

//     };

//     struct spi_buf_set spi_tx_buffer_set = {
//         .buffers = tx_spi_bufs,
//         .count = 1,
//     };

// 	    struct spi_buf rx_spi_bufs[]={
//       { .buf = rx_buf, .len = sizeof(rx_buf)},
//     };

//     struct spi_buf_set spi_rx_buffer_set = {
//       .buffers = rx_spi_bufs,
//       .count = 1,
//     };

//     int ret = 0;

//     ret = spi_transceive(spi_dev->bus, (const struct spi_config *) &spi_dev->config, &spi_tx_buffer_set, &spi_rx_buffer_set);
// 	memcpy(data, &rx_buf[2], len);
// 	printk("SPI RX: 0x%.2x, 0x%.2x\n", tx_buffer[1], rx_buf[2]);
//     return ret;
// }

// int max86178_write_reg(const struct spi_dt_spec *spi_dev, uint8_t reg_addr,uint8_t *data, uint32_t len)
// {
//     uint8_t tx_buffer[2+len];
// 	uint8_t rx_buffer[2+len];

//     tx_buffer[0] = reg_addr;
//     tx_buffer[1] = SPI_WRITE_CMD;
//     memcpy(&tx_buffer[2], data, len);
//    // printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[2]);

// 	const struct spi_buf tx_buf = {
// 		.buf = tx_buffer,
// 		.len = sizeof(tx_buffer)
// 	};
// 	const struct spi_buf_set tx = {
// 		.buffers = &tx_buf,
// 		.count = 1
// 	};

// 	struct spi_buf rx_buf = {
// 		.buf = rx_buffer,
// 		.len = sizeof(rx_buffer),
// 	};
// 	const struct spi_buf_set rx = {
// 		.buffers = &rx_buf,
// 		.count = 1
// 	};

//    	// Reset signal
// 	k_poll_signal_reset(&spi_done_sig);

// 	// Start transaction
// 	int error = spi_transceive_signal(spi_dev->bus,  (const struct spi_config *) &spi_dev->config, &tx, &rx, &spi_done_sig);
// 	if(error != 0){
// 		printk("SPI transceive error: %i\n", error);
// 		return error;
// 	}
// 	// Wait for the done signal to be raised and log the rx buffer
// 	int spi_signaled, spi_result;
// 	do{
// 		k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
// 	} while(spi_signaled == 0);
// 	memcpy(data, &rx_buffer[2], len);
// 	//printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[1], rx_buffer[2]);

// 			for (int i = 0; i < len; i++)
// 		{
// 			printk("max86178_write_reg  %d %d \n", reg_addr, data[i]);
// 			reg_addr++;
// 		}
// 	return 0;
// }

// ok funzionante
// int max86178_read_reg(const struct spi_dt_spec *spi_dev, uint8_t *data, uint16_t len)
// {
// 	uint8_t tx_buffer[2 + len];
// 	//uint8_t *tx_buffer = malloc(2 + len);
// 	uint8_t init_reg = data[0];
// 	tx_buffer[0] = data[0];
// 	tx_buffer[1] = SPI_READ_CMD;
// 	for (int i = 0; i < len; i++)
// 	{
// 		tx_buffer[2 + i] = 0;
// 	}

// 	uint8_t rx_buf[2 + len];
// 	//uint8_t *rx_buf = malloc(2 + len);

// 	struct spi_buf tx_spi_bufs[] = {
// 		{.buf = tx_buffer, .len = sizeof(tx_buffer)},
// 	};

// 	struct spi_buf_set spi_tx_buffer_set = {
// 		.buffers = tx_spi_bufs,
// 		.count = 1,
// 	};

// 	struct spi_buf rx_spi_bufs[] = {
// 		{.buf = rx_buf, .len = sizeof(rx_buf)},
// 	};

// 	struct spi_buf_set spi_rx_buffer_set = {
// 		.buffers = rx_spi_bufs,
// 		.count = 1,
// 	};
// 	int ret;

// 	ret = spi_transceive(spi_dev->bus, (const struct spi_config *)&spi_dev->config, &spi_tx_buffer_set, &spi_rx_buffer_set);

// 	memcpy(data, &rx_buf[2], len);

// 	//free(tx_buffer);
//     //free(rx_buf);

// 	if (ret != 0)
// 	{
// 		printk("spi_read error: %i\n", ret);
// 	}
// 	else
// 	{
// 		for (int i = 0; i < len; i++)
// 		{
// 			printk("max86178_read_reg  %d %d \n", init_reg, data[i]);
// 			init_reg++;
// 		}
// 	}

// 	// 	printk("Max86140_read Reg %d Data %d\n", reg, data);
// 	// printk("max86178_read_reg\n");
// 	return ret;
// }


int max86178_read_reg(const struct spi_dt_spec *spi_dev, uint8_t *data, uint16_t len)
{

    // Allocate buffers on the heap
    uint8_t *tx_buffer = k_malloc(2 + len);
    uint8_t *rx_buf = k_malloc(2 + len);

    uint8_t init_reg = data[0];
    tx_buffer[0] = data[0];
    tx_buffer[1] = SPI_READ_CMD;

    // for (int i = 2; i < len; i++)
    // {
    //     tx_buffer[i] = 0;
    // }

    struct spi_buf tx_spi_bufs[] = {
        {.buf = tx_buffer, .len = 2 + len},
    };

    struct spi_buf_set spi_tx_buffer_set = {
        .buffers = tx_spi_bufs,
        .count = 1,
    };

    struct spi_buf rx_spi_bufs[] = {
        {.buf = rx_buf, .len = 2 + len},
    };

    struct spi_buf_set spi_rx_buffer_set = {
        .buffers = rx_spi_bufs,
        .count = 1,
    };
    int ret;

    ret = spi_transceive(spi_dev->bus, (const struct spi_config *)&spi_dev->config, &spi_tx_buffer_set, &spi_rx_buffer_set);

    // Copy the data, skipping the first 2 bytes
    memcpy(data, &rx_buf[2], len);

    if (ret != 0)
    {
        printk("spi_read error: %i\n", ret);
    }
    // else
    // {
	// 	for (int i = 0; i < len; i++)
	// 	{
	// 		printk("max86178_read_reg  %d %d \n", init_reg, data[i]);
	// 		init_reg++;
	// 	}
    // }

    // Free the allocated memory
    k_free(tx_buffer);
    k_free(rx_buf);

    return ret;
}

static const uint8_t max86178_part_info[][2] = {
	// check for MRD106 conf
	{MAX86178_PART_ID_VAL, 1},
};
int max86178_block_write(const struct spi_dt_spec *spi_dev,  struct regmap reg_block[], int size)
{
	int i;
	int ret = 0;

	for (i = 0; i < size; i++) {
		ret = max86178_write_reg(spi_dev, reg_block[i].addr, &reg_block[i].val,sizeof(reg_block[i].val));
		if (ret < 0)
		{
			printk("max86178_write_reg returned %d\n", ret);
			return ret;
		}
	}

	return ret;
}



int max86178_get_part_info(const struct spi_dt_spec *spi_dev,
						   uint8_t *part_id, uint8_t *rev_id, uint8_t *num_pd) // OK check
{
	int i;
	int ret;
	uint8_t buf[2];

	buf[0] = MAX86178_REV_ID_REG;
	// buf[0] = MAX86178_PART_ID_REG;
	ret = max86178_read_reg(spi_dev, buf, 2);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(max86178_part_info); i++)
	{
		if (buf[1] == max86178_part_info[i][0])
		{
			*num_pd = max86178_part_info[i][1];
			*rev_id = buf[0];
			*part_id = buf[1];
			return 0;
		}
	}

	*part_id = 0x39; // MYG check to fake Wilson GUI for test pusposes
	// Unsupported part
	return -1;
}

int max86178_set_sample_rate(struct max86178_dev *sd, uint16_t rate) // OK , be sure internal oscillator is seleceted as frame clock
{
	int ret = -1;
	uint16_t afe_clk, clk_div;
	uint8_t reg;

	/* Return if sample rate is zero */
	if (rate == 0)
		return ret;

	reg = MAX86178_PLL_CFG6_REG;
	ret = max86178_read_reg(sd->spi_dev, &reg, 1);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	/* Identify selected AFE clock in Hz */
	if (reg & MAX86178_PLL_CLK_FREQ_SEL_MASK)
		afe_clk = 32768;
	else
		afe_clk = 32000;

	/* Calculate AFE clock divider */
	clk_div = afe_clk / rate;

	/* Set AFE clock division into the MAX86178 register */
	uint8_t data;
	data = clk_div & MAX86178_FR_CLK_DIV_L_MASK;
	ret = max86178_write_reg(sd->spi_dev, MAX86178_PPG_FRM_RATE_LSB_REG, &data, 1);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}
    data= (clk_div >> 8) & MAX86178_FR_CLK_DIV_H_MASK;
	max86178_write_reg(sd->spi_dev, MAX86178_PPG_FRM_RATE_MSB_REG, &data, 1);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	return 0;
}
int max86178_get_sample_rate(struct max86178_dev *sd) // OK be sure to set internal oscillator as ppg frame clock
{
	int ret = -1;
	uint8_t reg;
	uint16_t afe_clk, clk_div, sample_rate;

	reg = MAX86178_PLL_CFG6_REG;
	ret = max86178_read_reg(sd->spi_dev, &reg, 1);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	/* Identify selected AFE clock in Hz */
	if (reg & MAX86178_PLL_CLK_FREQ_SEL_MASK)
		afe_clk = 32768;
	else
		afe_clk = 32000;

	/* Read AFE clock division  */
	reg = MAX86178_PPG_FRM_RATE_MSB_REG;
	ret = max86178_read_reg(sd->spi_dev, &reg, 1);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	/* MSB of clock division */
	clk_div = reg & MAX86178_FR_CLK_DIV_H_MASK;
	clk_div <<= 8;

	reg = MAX86178_PPG_FRM_RATE_LSB_REG;
	ret = max86178_read_reg(sd->spi_dev, &reg, 1);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}
	/* LSB of clock division */
	clk_div |= reg;

	/* Calculate sample rate */
	sample_rate = afe_clk / clk_div;

	return sample_rate;
}

int max86178_get_meas_num(struct max86178_dev *sd) // OK
{
	int ret = -1;
	uint8_t reg, i;
	uint8_t meas_num = 0;

	reg = MAX86178_PPG_CFG1_REG;

	ret = max86178_read_reg(sd->spi_dev, &reg, sizeof(reg));
	// ret = max86178_read_reg(const struct spi_dt_spec *spi_dev,  reg, uint8_t *data, 1)
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	for (i = 0; i < 6; i++)
	{
		if (reg & 0x1)
			meas_num++;

		reg >>= 1;
	}

	return meas_num;
}

static int update_bits(uint8_t reg, uint8_t mask, uint8_t val)
{
	int ret_val = MAX86178_SUCCESS;
	uint8_t reg_val;

	struct max86178_dev *sd = max86178_get_device_data();

	if (val > mask)
	{
		ret_val = MAX86178_FAILURE;
	}
	else
	{
		// place val to masked position
		int i = 0;

		for (i = 0; i < sizeof(mask) * 8; i++)
			if (mask & 1 << i)
				break;

		val = val << i;
	}

	if (ret_val == MAX86178_SUCCESS)
	{
		ret_val = max86178_read_reg(sd->spi_dev, &reg_val, 1);
		if (ret_val == MAX86178_SUCCESS)
		{
			uint8_t tmp = 0;

			tmp = reg_val & ~mask;
			tmp |= val & mask;
			ret_val = max86178_write_reg(sd->spi_dev, reg, &tmp, sizeof(tmp));
		}
	}

	return ret_val;
}

int max86178_dump_regs(const struct spi_dt_spec *spi_dev, uint8_t *buf,
					   uint8_t start_addr, uint8_t end_addr) // OK check for result register val order
{
	int i;
	int32_t ret;
	uint8_t tmp[2];

	for (i = start_addr; i <= end_addr; i++)
	{
		tmp[0] = i;
		ret = max86178_read_reg(spi_dev, tmp, sizeof(tmp));
		if (ret != 0)
			return ret;
		buf[i - start_addr] = tmp[0];
	}

	return 0;
}

int max86178_bioz_enable(max86178_bioz_meas_t meas)
{
    int ret_val = MAX86178_SUCCESS;

    if (meas >= MAX86178_BIOZ_MEAS_INVALID)
        ret_val = MAX86178_FAILURE;

    if (ret_val == MAX86178_SUCCESS)
        ret_val = update_bits(MAX86178_BIOZ_CFG1_REG, MAX86178_BIOZ_ENABLE_MASK, (uint8_t)meas);

    return ret_val;
}

int max86178_sensor_enable(struct max86178_dev *sd, int agc_enable, int enable) // OK
{
	int ret = 0;
	uint8_t a_full;

	// queue_reset(&sd->queue); // todo queuue zephyr
	if (enable)
	{

		ret = max86178_poweron(sd);
		if (ret < 0)
			goto fail;

		max86178_irq_clr_to_zero(sd);

		/*INTx is enabled and gets cleared automatically after 244us  */
		// ret = max86178_write_reg(NULL, MAX86178_PIN_FUNC_CFG_REG, 0x19); // check whether needed! INT1 is the INT line in MAX88678 Wrist demo schematic
		uint8_t data=0x1F;
		ret = max86178_write_reg(sd->spi_dev, MAX86178_SYSTEM_PIN_FUNC_REG, &data, 1); //
		if (ret < 0)
			goto fail;
        data= MAX86178_A_FULL_EN_MASK | MAX86178_FRAME_RDY_EN_MASK | MAX86178_FIFO_DATA_RDY_EN2_MASK;
		ret = max86178_write_reg(sd->spi_dev, MAX86178_INT1_ENABLE1_REG, &data , 1); // EN INT1  PPG
		//ret = max86178_write_reg(sd->spi_dev, MAX86178_INT1_ENABLE5_REG, &data , 1);  // EN INT1 BIOZ
		data=MAX86178_PPG_FRAME_RDY_EN2_MASK;
		ret = max86178_write_reg(sd->spi_dev, MAX86178_INT2_ENABLE1_REG, &data , 1); // MAX86178_A_FULL_EN_MASK | MAX86178_FRAME_RDY_EN_MASK);

		// Flush fifo
		uint8_t fifo_cfg2_value = MAX86178_FIFO_CFG2_REG;

		ret = max86178_read_reg(sd->spi_dev, &fifo_cfg2_value, 1);


        data=fifo_cfg2_value | MAX86178_FLUSH_FIFO_MASK | MAX86178_FIFO_STAT_CLR_MASK | MAX86178_FIFO_RO_MASK; // MAX86178_FIFO_STAT_CLR_MASK add 231024
		ret = max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG2_REG, &data , 1); // MAX86178_A_FULL_EN_MASK | MAX86178_FRAME_RDY_EN_MASK);

        sd->curr_state=1;
		if (ret < 0)
			goto fail;
	}
	else
	{
		ret = max86178_poweroff(sd);
		if (ret < 0)
			goto fail;
	}
	return 0;

fail:
	printk("%s failed. ret: %d\n", __func__, ret);
	return ret;
}
int max86178_regulator_onoff(struct max86178_dev *sd, char enable)
{
	sd->regulator_state = enable;
	return 0;
}

int max86178_poweron(struct max86178_dev *sd)
{ // OK
	int ret = 0;
	uint8_t buf;

	buf = MAX86178_SYSTEM_CFG1_REG;
	ret = max86178_read_reg(sd->spi_dev, &buf, 1);
	buf &= ~MAX86178_SYSTEM_SHDN_MASK;
	ret = max86178_write_reg(sd->spi_dev, MAX86178_SYSTEM_CFG1_REG, &buf, sizeof(buf));
	//if (sd->regulator_state) todo
	
	ret |= max86178_regulator_onoff(sd, PWR_ON);
	if (ret < 0)
	{
		printk("Unable to turn off the regulator. %s:%d, ret: %d\n",
				__func__, __LINE__, ret);
	}
	//sd->regulator_state = 1;
			// buf = MAX86178_SYSTEM_CFG1_REG;
			// ret = max86178_read_reg(sd->spi_dev, &buf, 1);
			// buf &= ~MAX86178_DISABLE_I2C_MASK;
			// buf &= ~MAX86178_SYSTEM_ECG_BIOZ_TIMING_DATA_MASK;
			// ret = max86178_write_reg(sd->spi_dev, MAX86178_SYSTEM_CFG1_REG, &buf, sizeof(buf));
	
	return ret;
}
int max86178_poweroff(struct max86178_dev *sd) // OK
{
	int ret = 0;
	uint8_t buf;
	

	buf = MAX86178_SYSTEM_CFG1_REG;
	ret = max86178_read_reg(sd->spi_dev, &buf, sizeof(buf));
	buf |= MAX86178_SYSTEM_SHDN_MASK;
	ret = max86178_write_reg(sd->spi_dev, MAX86178_SYSTEM_CFG1_REG,
							 &buf, 1);
	if (sd->regulator_state)
	{
		ret |= max86178_regulator_onoff(sd, PWR_OFF);
		if (ret < 0)
		{
			printk("Unable to turn off the regulator. %s:%d, ret: %d\n",
				   __func__, __LINE__, ret);
		}
	}
	return ret;
}

/* MAX86178 IRQ Functions */
static volatile uint16_t max86178_irq_cnt = 0;

void max86178_irq_clr_to_zero(struct max86178_dev *sd)
{ // OK
	uint8_t read_buf[3];

	read_buf[0] = MAX86178_STATUS1_REG;
	/* Read three status registers to clear interrupts */
	max86178_read_reg(sd->spi_dev, read_buf, sizeof(read_buf));
	enter_critical_section();
	//read_buf[0] = MAX86178_STATUS1_REG;
	max86178_irq_cnt = 0;
	//max86178_read_reg(sd->spi_dev, read_buf, sizeof(read_buf));
	exit_critical_section();
}


int max86178_reset(struct max86178_dev *sd) // OK
{
	int ret = 0;

	if (!sd->regulator_state)
	{
		ret = max86178_regulator_onoff(sd, PWR_ON);
		if (ret < 0)
		{
			printk("Unable to turn on the regulator. %s:%d, ret: %d\n",
				   __func__, __LINE__, ret);
		}
	}
    uint8_t data;
	data=MAX86178_SYSTEM_RESET_MASK;
	ret = max86178_write_reg(sd->spi_dev, MAX86178_SYSTEM_CFG1_REG, &data, 1);
	sd->die_temp.frac = 0;
	sd->die_temp.tint = 0;

	return ret;
}

int max86178_irq_handler(void *arg) // OK check for overlapping read of status reg2
{
	struct max86178_dev *sd = arg;
	int ret = 0;
	union int_status status;

	status.val[0] = MAX86178_STATUS1_REG;
	ret = max86178_read_reg(sd->spi_dev, status.val, 3);
	printk("Status reg: %X %X %X\n", status.val[0], status.val[1], status.val[2]);
	k_msleep(10);
	if (ret < 0)
	{
		printk("Comm failed. err: %d. %s:%d\n",
			   ret, __func__, __LINE__);
		return -1;
	}

	if (status.a_full || status.fifo_data_rdy || status.frame_rdy)
	{
		return max86178_fifo_irq_handler(sd);
	}

	if (status.thresh1_hilo)
		printk("thresh1_hilo interrupt was triggered.\n");

	if (status.thresh2_hilo)
		printk("thresh2_hilo interrupt was triggered.\n");

	if (status.pwr_rdy)
		printk("Pwr_rdy interrupt was triggered.\n");

	if (status.alc_ovf)
		printk("alc_ovf interrupt was triggered.\n");

	if (status.exp_ovf)
		printk("exp_ovf interrupt was triggered.\n");

	status.val[0] = MAX86178_STATUS2_REG;
	ret = max86178_read_reg(sd->spi_dev, status.val, 2);
	// printk("Status reg: %X %X\n", status_s.val[0], status_s.val[1]);
	if (ret < 0)
	{
		printk("Comm failed. err: %d. %s:%d\n",
			   ret, __func__, __LINE__);
		return -1;
	}

	if (status.invalid_cfg)
		printk("INVALID AFE CFG!\n");

	/*if (status.vdd_oor) {
		sd->vdd_oor_cnt++;
		printk("VDD Out of range cnt: %d\n", sd->vdd_oor_cnt);
	}*/

	return 0;
}

/* MAX86178 IRQ Functions */
// static volatile uint16_t max86178_irq_cnt = 0;
int max86178_get_irq_state(void *data) // OK
{
	int ret;
	enter_critical_section();
	ret = max86178_irq_cnt;
	exit_critical_section();
	return ret;
}

int max86178_is_ecg_enabled(uint8_t *p_ecg_enabled) // OK
{
	int ret = 0;

	uint8_t reg = MAX86178_EGC_CONFIG_1_REG;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->spi_dev, &reg, sizeof(reg));

	if (0 == ret)
	{
		if ((reg & MAX86178_ECG_ENABLE_MASK))
		{
			*p_ecg_enabled = 1;
		}
		else
		{
			*p_ecg_enabled = 0;
		}
	}

	return ret;
}

int max86178_init_fifo(struct max86178_dev *sd, uint8_t a_full_val) // OK
{
	int ret;
    uint8_t data;
	data=MAX86178_FIFO_A_FULL_MASK & a_full_val;
	ret = max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG1_REG, &data
							 , sizeof(data));
    data=MAX86178_FIFO_RO_MASK | MAX86178_A_FULL_ONCE | MAX86178_FIFO_STAT_CLR_MASK | MAX86178_FLUSH_FIFO_MASK;
	ret |= max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG2_REG,
							  &data,
							  sizeof(data));


	// ret = max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG1_REG, MAX86178_FIFO_A_FULL_MASK & a_full_val,1);

	// ret |= max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG2_REG,
	// 		MAX86178_FIFO_RO_MASK |
	// 		MAX86178_A_FULL_ONCE |
	// 		MAX86178_FIFO_STAT_CLR_MASK |
	// 		MAX86178_FLUSH_FIFO_MASK,1);							  

	return ret;
}

void max86178_irq_handler_fast(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("GPIO interrupt on pin 8 triggered!\n");
    max86178_irq_cnt++;

	struct max86178_dev *max86178_driver = max86178_get_device_data();
	if (max86178_driver->curr_state == 1)
 	  max86178_fifo_irq_handler(max86178_driver); // gestione interrupt sensore max86178
	

}

static inline int max86178_read_fifo_data(struct max86178_dev *sd, uint8_t *fifo_data, int num_samples) // OK
{
	uint16_t num_bytes = num_samples * MAX86178_DATA_WORD_SIZE;
	int ret = 0;

	fifo_data[0] = MAX86178_FIFO_DATA_REG;
	ret = max86178_read_reg(sd->spi_dev, fifo_data, num_bytes);
	if (ret < 0)
	{
		printk("%s failed. ret: %d\n", __func__, ret);
		return ret;
	}

	return ret;
}
static inline int max86178_get_num_samples_in_fifo(struct max86178_dev *sd) // OK
{

	int ret_val = MAX86178_SUCCESS;
	uint8_t reg_val[2] = {0};

	reg_val[0] = MAX86178_OVF_CNT_REG;

	if (ret_val == MAX86178_SUCCESS)
	{
		// ret_val = read_reg(MAX86178_FIFO_CNT1_REG, reg_val, 2);
		ret_val = max86178_read_reg(sd->spi_dev, reg_val, 2);

		if (ret_val == MAX86178_SUCCESS)
		{
			if ((reg_val[0] & ~MAX86178_FIFO_DATA_COUNT_MSB_MASK) > 0)
			{
				// Data overflow
				ret_val = MAX86178_MAX_FIFO_DEPTH;
			}
			else
			{
				ret_val = (reg_val[0] & MAX86178_FIFO_DATA_COUNT_MSB_MASK) ? MAX86178_MAX_FIFO_DEPTH : reg_val[1];
			}
		}
	}

	return ret_val;
}
int max86178_get_num_of_channel(uint8_t *p_num_ch) // OK
{
	int ret = 0;
	uint8_t index = 0;
	uint8_t num_of_ch = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	uint8_t reg[1];

	reg[0] = MAX86178_PPG_CFG1_REG;
	ret = max86178_read_reg(sd->spi_dev, reg, 1);
	printk("Read Channel PPG Enabled %d\n", reg[0]);
	k_msleep(10);
	if (0 == ret)
	{
		for (index = 0; index < MAX86178_MEAS_CHMAX; index++)
		{
			num_of_ch += ((reg[0] >> index) & 1);
		}
	}

	*p_num_ch = num_of_ch;
	printk("Num Channel %d - %d\n", num_of_ch, *p_num_ch);

	return ret;
}
int max86178_is_ppg_enabled(uint8_t *p_ppg_enabled) // OK
{
	int ret = 0;
	uint8_t nm_ch = 0;

	ret = max86178_get_num_of_channel(&nm_ch);

	if (0 == ret)
	{
		if (nm_ch > 0)
		{
			*p_ppg_enabled = 1;
		}
		else
		{
			*p_ppg_enabled = 0;
		}
	}

	return ret;
}

int max86178_is_acc_enabled(uint8_t *p_acc_enabled)
{
//	*p_acc_enabled = gUseAcc;  todo verificare
	*p_acc_enabled = NULL;
	return 0;
}

int max86178_is_iq_enabled(uint8_t *p_iq_enabled) // OK
{
	int ret = 0;

	uint8_t reg = MAX86178_BIOZ_CFG1_REG;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->spi_dev, &reg, 1);

	if (0 == ret)
	{
		if ((reg & MAX86178_BIOZ_ENABLE_MASK))
		{
			*p_iq_enabled = 1;
		}
		else
		{
			*p_iq_enabled = 0;
		}
	}

	return ret;
}


// Initial AFE configuration with no timing data enabled
// struct regmap ppg_init_cfg[] = {
// 	// MYG fill with MEAS1-2 config & ECG default for Test purposes - check ECG default from MRD104
// 	{MAX86178_SYSTEM_CFG1_REG, MAX86178_SYSTEM_SHDN_MASK}, // CSB/I2C_SEL pin selects interface , disable all timing data
// 	/* PLL */
// 	///////////////////////////////////////////////////////////////////////////////{MAX86178_PIN_FUNC_CFG_REG, 0x1E},
// 	{MAX86178_PLL_CFG1_REG, 0x01}, // PLL EN MDIV = 256 PLL_CLK = 32768*256
// 	{MAX86178_PLL_CFG2_REG, 0xFF}, // PLL EN M =  MDIV+1 = 256 PLL_CLK = 32768*256
// 	// BiOZ Clock
// 	{MAX86178_PLL_CFG3_REG, 0x00}, // BioZ NDIV = 256, BIOZ KDIV = 1
// 								   // ECG ADC Clock
// 	{MAX86178_PLL_CFG4_REG, 0x03}, // ECG_FDIV = 4
// 	{MAX86178_PLL_CFG5_REG, 0x40}, // ECG_NDIV = 64
// 								   // 32768 internal osc as input to PLL
// 	{MAX86178_PLL_CFG6_REG, 0x20}, // internal 32678Hz ref clock for PLL
// 								   // PPG frame clock
// 	{MAX86178_PPG_FRM_RATE_MSB_REG, 0x05},
// 	{MAX86178_PPG_FRM_RATE_LSB_REG, 0x1B}, // PPG to 25 hz with ref_clk 32768 division of 1307 - 0x51B
 
// 	{MAX86178_PPG_CFG1_REG, 0x03},	   // En chan1-2-3   def 0x03  todo
// 									   // MEAS1 settings: 
// 	{MAX86178_MEAS1_SELECT_REG, 0x00}, // Both to LED1_DRV
// 	{MAX86178_MEAS1_CFG1_REG, 0x5A},
// 	{MAX86178_MEAS1_CFG2_REG, 0x33},
// 	{MAX86178_MEAS1_CFG3_REG, 0x00},
// 	{MAX86178_MEAS1_CFG4_REG, 0x47},
// 	{MAX86178_MEAS1_CFG5_REG, 0x0E},
// 	{MAX86178_MEAS1_LED_A_REG, 0x1E},
// 	{MAX86178_MEAS1_LED_B_REG, 0x00},
// 	// MEAS2 settings:
// 	{MAX86178_MEAS2_SELECT_REG, 0x09}, // Both to LED2_DRV
// 	{MAX86178_MEAS2_CFG1_REG, 0x5A},
// 	{MAX86178_MEAS2_CFG2_REG, 0x33},
// 	{MAX86178_MEAS2_CFG3_REG, 0x00},
// 	{MAX86178_MEAS2_CFG4_REG, 0x47},
// 	{MAX86178_MEAS2_CFG5_REG, 0x0E},
// 	{MAX86178_MEAS2_LED_A_REG, 0x1F},
// 	{MAX86178_MEAS2_LED_B_REG, 0x00},
// 	// ECG settings
// 	{MAX86178_ECG_CFG1_REG, 0x04}, //0x04 DISABLED ECG  -  DEC= 64 => ECG SR = 512 with 32768 ECG_ADC clock, ECG DISABLED IN THIS CONFIG !! TO ENABLE WRITE : 0x05  todo
// 	{MAX86178_ECG_CFG2_REG, 0x81}, // PGA gain is 1V/V , INA gain is 20V/V
// 	{MAX86178_ECG_CFG3_REG, 0x01}, // Automatic INA recovery disabled , Enable the ECG MUX and assigned to ECGP = ECG_EL1 , ECGN= ECG_EL2, RLD = ECG_EL3
// 	{MAX86178_ECG_CFG4_REG, 0x3D}, // Automatic fast recovery normal mode , Fast recovery threshold set to 95% of full scale
// 	{MAX86178_CAL_CFG_1_REG, 0x00},
// 	{MAX86178_CAL_CFG_2_REG, 0x00},
// 	{MAX86178_CAL_CFG_3_REG, 0x00},
// 	{MAX86178_LEAD_DETECT_CFG_1_REG, 0x00},
// 	{MAX86178_LEAD_DETECT_CFG_2_REG, 0x00},
// 	{MAX86178_LEAD_BIAS_CFG_1_REG, 0x04},
// 	{MAX86178_RLD_CFG_1_REG, 0xCF},
// 	{MAX86178_RLD_CFG_2_REG, 0x40},

// 	// BiOZ Respiration setting
// 	{MAX86178_BIOZ_CFG1_REG, 0xF7}, // ADC_OSR = 1024 , DAC_OSR = 256, ECG and BioZ bandgap bias enabled   0xf4 original 0xf7 iq enabled  todo
// 	{MAX86178_BIOZ_CFG2_REG, 0x00}, // Set the digital HPF to bypass, Set the digital LPF to bypass,
// 	{MAX86178_BIOZ_CFG3_REG, 0x28},
// 	{MAX86178_BIOZ_CFG4_REG, 0x00},
// 	{MAX86178_BIOZ_CFG5_REG, 0x00},
// 	{MAX86178_BIOZ_CFG6_REG, 0x33}, // Set the analog HPF to 1kHz
// 	{MAX86178_BIOZ_CFG7_REG, 0x8A},
// 	{MAX86178_BIOZ_CFG8_REG, 0x02},
// 	{MAX86178_BIOZ_LOW_THRESH_REG, 0x00},
// 	{MAX86178_BIOZ_HIGH_THRESH_REG, 0xFF},
// 	{MAX86178_BIOZ_MUX_CFG1_REG, 0x42}, // Enable the MUX
// 	{MAX86178_BIOZ_MUX_CFG2_REG, 0x02},
// 	{MAX86178_BIOZ_MUX_CFG3_REG, 0xA0},
// 	{MAX86178_BIOZ_MUX_CFG4_REG, 0x1B},
// 	{MAX86178_BIOZ_LEAD_DETCT_CFG1_REG, 0x00},
// 	{MAX86178_BIOZ_LOFF_THRESH_REG, 0x00},
// 	{MAX86178_BIOZ_LEAD_BIAS_CFG1_REG, 0x07},
// 	{MAX86178_RESPIRATION_CFG1_REG, 0x00},

// };


// CONFIGURATION FOR BIOZ
// // Initial AFE configuration with no timing data enabled
// struct regmap ppg_init_cfg[] = {
// 	// MYG fill with MEAS1-2 config & ECG default for Test purposes - check ECG default from MRD104
// 	{MAX86178_SYSTEM_CFG1_REG, MAX86178_SYSTEM_SHDN_MASK}, // CSB/I2C_SEL pin selects interface , disable all timing data
// 	/* PLL */
// 	///////////////////////////////////////////////////////////////////////////////{MAX86178_PIN_FUNC_CFG_REG, 0x1E},
// 	{MAX86178_PLL_CFG1_REG, 0x00}, // PLL EN MDIV = 256 PLL_CLK = 32768*256
// 	{MAX86178_PLL_CFG2_REG, 0xFF}, // PLL EN M =  MDIV+1 = 256 PLL_CLK = 32768*256
// 	// BiOZ Clock
// 	{MAX86178_PLL_CFG3_REG, 0x05}, // BioZ NDIV = 256, BIOZ KDIV = 1
// 								   // ECG ADC Clock
// 	{MAX86178_PLL_CFG4_REG, 0x03}, // ECG_FDIV = 4
// 	{MAX86178_PLL_CFG5_REG, 0x40}, // ECG_NDIV = 64
// 								   // 32768 internal osc as input to PLL
// 	{MAX86178_PLL_CFG6_REG, 0x20}, // internal 32678Hz ref clock for PLL
// 								   // PPG frame clock
// 	{MAX86178_PPG_FRM_RATE_MSB_REG, 0x01},
// 	{MAX86178_PPG_FRM_RATE_LSB_REG, 0x00}, // PPG to 25 hz with ref_clk 32768 division of 1307 - 0x51B
 
// 	{MAX86178_PPG_CFG1_REG, 0x00},	   // En chan1-2-3   def 0x03  todo
// 									   // MEAS1 settings: 
// 	{MAX86178_MEAS1_SELECT_REG, 0x00}, // Both to LED1_DRV
// 	{MAX86178_MEAS1_CFG1_REG, 0x58},
// 	{MAX86178_MEAS1_CFG2_REG, 0x33},
// 	{MAX86178_MEAS1_CFG3_REG, 0x00},
// 	{MAX86178_MEAS1_CFG4_REG, 0x47},
// 	{MAX86178_MEAS1_CFG5_REG, 0x0E},
// 	{MAX86178_MEAS1_LED_A_REG, 0x1E},
// 	{MAX86178_MEAS1_LED_B_REG, 0x00},
// 	// MEAS2 settings:
// 	{MAX86178_MEAS2_SELECT_REG, 0x09}, // Both to LED2_DRV
// 	{MAX86178_MEAS2_CFG1_REG, 0x5A},
// 	{MAX86178_MEAS2_CFG2_REG, 0x33},
// 	{MAX86178_MEAS2_CFG3_REG, 0x00},
// 	{MAX86178_MEAS2_CFG4_REG, 0x47},
// 	{MAX86178_MEAS2_CFG5_REG, 0x0E},
// 	{MAX86178_MEAS2_LED_A_REG, 0x0F},
// 	{MAX86178_MEAS2_LED_B_REG, 0x00},
// 	// ECG settings
// 	{MAX86178_ECG_CFG1_REG, 0x08}, //0x04 DISABLED ECG  -  DEC= 64 => ECG SR = 512 with 32768 ECG_ADC clock, ECG DISABLED IN THIS CONFIG !! TO ENABLE WRITE : 0x05  todo
// 	{MAX86178_ECG_CFG2_REG, 0x01}, // PGA gain is 1V/V , INA gain is 20V/V
// 	{MAX86178_ECG_CFG3_REG, 0x01}, // Automatic INA recovery disabled , Enable the ECG MUX and assigned to ECGP = ECG_EL1 , ECGN= ECG_EL2, RLD = ECG_EL3
// 	{MAX86178_ECG_CFG4_REG, 0x3F}, // Automatic fast recovery normal mode , Fast recovery threshold set to 95% of full scale
// 	{MAX86178_CAL_CFG_1_REG, 0x00},
// 	{MAX86178_CAL_CFG_2_REG, 0x00},
// 	{MAX86178_CAL_CFG_3_REG, 0x00},
// 	{MAX86178_LEAD_DETECT_CFG_1_REG, 0x00},
// 	{MAX86178_LEAD_DETECT_CFG_2_REG, 0x00},
// 	{MAX86178_LEAD_BIAS_CFG_1_REG, 0x04},
// 	{MAX86178_RLD_CFG_1_REG, 0x4E},
// 	{MAX86178_RLD_CFG_2_REG, 0x40},

// 	// BiOZ Respiration setting
// 	{MAX86178_BIOZ_CFG1_REG, 0xF5}, // ADC_OSR = 1024 , DAC_OSR = 256, ECG and BioZ bandgap bias enabled   0xf4 original 0xf7 iq enabled  todo
// 	{MAX86178_BIOZ_CFG2_REG, 0x00}, // Set the digital HPF to bypass, Set the digital LPF to bypass,
// 	{MAX86178_BIOZ_CFG3_REG, 0x38},
// 	//{MAX86178_BIOZ_CFG3_REG, 0x28},
// 	{MAX86178_BIOZ_CFG4_REG, 0x00},
// 	{MAX86178_BIOZ_CFG5_REG, 0x00},
// 	{MAX86178_BIOZ_CFG6_REG, 0x33}, // Set the analog HPF to 1kHz
// 	{MAX86178_BIOZ_CFG7_REG, 0x8A},
// 	{MAX86178_BIOZ_CFG8_REG, 0x00},
// 	{MAX86178_BIOZ_LOW_THRESH_REG, 0x00},
// 	{MAX86178_BIOZ_HIGH_THRESH_REG, 0xFF},
// 	{MAX86178_BIOZ_MUX_CFG1_REG, 0x02}, // Enable the MUX
// 	{MAX86178_BIOZ_MUX_CFG2_REG, 0x00},
// 	{MAX86178_BIOZ_MUX_CFG3_REG, 0xA0},
// 	{MAX86178_BIOZ_MUX_CFG4_REG, 0x1C},
// 	{MAX86178_BIOZ_LEAD_DETCT_CFG1_REG, 0x10},
// 	{MAX86178_BIOZ_LOFF_THRESH_REG, 0xB0},
// 	{MAX86178_BIOZ_LEAD_BIAS_CFG1_REG, 0x0B},
// 	{MAX86178_RESPIRATION_CFG1_REG, 0x00},
	

// };
// Initial AFE configuration with no timing data enabled
struct regmap ppg_init_cfg[] = {
	// MYG fill with MEAS1-2 config & ECG default for Test purposes - check ECG default from MRD104
	{MAX86178_SYSTEM_CFG1_REG, MAX86178_SYSTEM_SHDN_MASK}, // CSB/I2C_SEL pin selects interface , disable all timing data
	/* PLL */
	///////////////////////////////////////////////////////////////////////////////{MAX86178_PIN_FUNC_CFG_REG, 0x1E},
	{MAX86178_PLL_CFG1_REG, 0x01}, // PLL EN MDIV = 256 PLL_CLK = 32768*256
	{MAX86178_PLL_CFG2_REG, 0xFF}, // PLL EN M =  MDIV+1 = 256 PLL_CLK = 32768*256
	// BiOZ Clock
	{MAX86178_PLL_CFG3_REG, 0x00}, // BioZ NDIV = 256, BIOZ KDIV = 1
								   // ECG ADC Clock
	{MAX86178_PLL_CFG4_REG, 0x03}, // ECG_FDIV = 4
	{MAX86178_PLL_CFG5_REG, 0x40}, // ECG_NDIV = 64
								   // 32768 internal osc as input to PLL
	{MAX86178_PLL_CFG6_REG, 0x20}, // internal 32678Hz ref clock for PLL
	//{MAX86178_PLL_CFG6_REG, 0x60}, // external 32678Hz ref clock for PLL 
								   // PPG frame clock
	{MAX86178_PPG_FRM_RATE_MSB_REG, 0x01},
	{MAX86178_PPG_FRM_RATE_LSB_REG, 0x00}, // PPG to 25 hz with ref_clk 32768 division of 1307 - 0x51B
 
	{MAX86178_PPG_CFG1_REG, 0x00},	   // En chan1-2-3   def 0x03  todo

	//{MAX86178_PPG_CFG3_REG, 0x02},	   // PROVA

//	{MAX86178_FIFO_CFG2_REG	, 0x00}, // PROVA2
									   // MEAS1 settings: 
	{MAX86178_MEAS1_SELECT_REG, 0x00}, // Both to LED1_DRV
	{MAX86178_MEAS1_CFG1_REG, 0x58},
	{MAX86178_MEAS1_CFG2_REG, 0x33},
	{MAX86178_MEAS1_CFG3_REG, 0x00},
	{MAX86178_MEAS1_CFG4_REG, 0x47},
	{MAX86178_MEAS1_CFG5_REG, 0x0E},
	{MAX86178_MEAS1_LED_A_REG, 0x1E},
	{MAX86178_MEAS1_LED_B_REG, 0x00},
	// MEAS2 settings:
	{MAX86178_MEAS2_SELECT_REG, 0x09}, // Both to LED2_DRV
	{MAX86178_MEAS2_CFG1_REG, 0x5A},
	{MAX86178_MEAS2_CFG2_REG, 0x33},
	{MAX86178_MEAS2_CFG3_REG, 0x00},
	{MAX86178_MEAS2_CFG4_REG, 0x47},
	{MAX86178_MEAS2_CFG5_REG, 0x0E},
	{MAX86178_MEAS2_LED_A_REG, 0x0F},
	{MAX86178_MEAS2_LED_B_REG, 0x00},
	// ECG settings
	{MAX86178_ECG_CFG1_REG, 0x03}, //0x04 DISABLED ECG  -  DEC= 64 => ECG SR = 512 with 32768 ECG_ADC clock, ECG DISABLED IN THIS CONFIG !! TO ENABLE WRITE : 0x05  todo
	{MAX86178_ECG_CFG2_REG, 0x01}, // PGA gain is 1V/V , INA gain is 20V/V
	{MAX86178_ECG_CFG3_REG, 0x01}, // Automatic INA recovery disabled , Enable the ECG MUX and assigned to ECGP = ECG_EL1 , ECGN= ECG_EL2, RLD = ECG_EL3
	{MAX86178_ECG_CFG4_REG, 0x3F}, // Automatic fast recovery normal mode , Fast recovery threshold set to 95% of full scale
	{MAX86178_CAL_CFG_1_REG, 0x00},
	{MAX86178_CAL_CFG_2_REG, 0x00},
	{MAX86178_CAL_CFG_3_REG, 0x00},
	{MAX86178_LEAD_DETECT_CFG_1_REG, 0x00},
	{MAX86178_LEAD_DETECT_CFG_2_REG, 0x00},
	{MAX86178_LEAD_BIAS_CFG_1_REG, 0x04},
	{MAX86178_RLD_CFG_1_REG, 0x4E},
	{MAX86178_RLD_CFG_2_REG, 0x40},

	// BiOZ Respiration setting
	{MAX86178_BIOZ_CFG1_REG, 0xF5}, // ADC_OSR = 1024 , DAC_OSR = 256, ECG and BioZ bandgap bias enabled   0xf4 original 0xf7 iq enabled  todo
	{MAX86178_BIOZ_CFG2_REG, 0x00}, // Set the digital HPF to bypass, Set the digital LPF to bypass,
	{MAX86178_BIOZ_CFG3_REG, 0x2B},
	{MAX86178_BIOZ_CFG4_REG, 0x00},
	{MAX86178_BIOZ_CFG5_REG, 0x00},
	{MAX86178_BIOZ_CFG6_REG, 0x33}, // Set the analog HPF to 1kHz
	{MAX86178_BIOZ_CFG7_REG, 0x8A},
	{MAX86178_BIOZ_CFG8_REG, 0x00},
	{MAX86178_BIOZ_LOW_THRESH_REG, 0x00},
	{MAX86178_BIOZ_HIGH_THRESH_REG, 0xFF},
	{MAX86178_BIOZ_MUX_CFG1_REG, 0x02}, // Enable the MUX
	{MAX86178_BIOZ_MUX_CFG2_REG, 0x00},
	{MAX86178_BIOZ_MUX_CFG3_REG, 0x00},
	{MAX86178_BIOZ_MUX_CFG4_REG, 0x1C},
	{MAX86178_BIOZ_LEAD_DETCT_CFG1_REG, 0x00},
	{MAX86178_BIOZ_LOFF_THRESH_REG, 0x00},
	{MAX86178_BIOZ_LEAD_BIAS_CFG1_REG, 0x00},
	{MAX86178_RESPIRATION_CFG1_REG, 0x00},
	

};
int max86178_startup_init(struct max86178_dev *sd) // OK
{
	int ret = 0;
	uint8_t a_full;

	ret = max86178_reset(sd);
	if (ret < 0)
		goto fail;

	ret |= max86178_block_write(sd->spi_dev, ppg_init_cfg, ARRAY_SIZE(ppg_init_cfg));
	k_msleep(200);
	a_full = MAX86178_FIFO_AFULL;

	ret |= max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG1_REG, &a_full, 1);

	max86178_irq_clr_to_zero(sd);
	ret |= max86178_poweroff(sd);
	printk("%s done\n", __func__);
	return ret;
fail:
	printk("%s failed. ret: %d\n", __func__, ret);
	return ret;
}

int max86178_is_enabled(uint8_t *p_is_enabled) // OK
{
	int ret = 0;

	uint8_t is_iq_enabled = 0;
	uint8_t is_ecq_enabled = 0;
	uint8_t is_ppg_enabled = 0;

	ret = max86178_is_iq_enabled(&is_iq_enabled);

	if (0 == ret)
	{
		ret = max86178_is_ecg_enabled(&is_ecq_enabled);
	}

	if (0 == ret)
	{
		ret = max86178_is_ppg_enabled(&is_ppg_enabled);
	}

	if (0 == ret)
	{
		*p_is_enabled = is_ppg_enabled | is_ecq_enabled | is_iq_enabled;
	}
	else
	{
		*p_is_enabled = 0;
	}

	return ret;
}

int max86178_is_ppg1_enabled(uint8_t *p_is_enabled) // OK
{
	int ret = 0;
	uint8_t reg_addr = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if (p_is_enabled == NULL)
	{
		ret = -1;
	}

	if (0 == ret)
	{
		reg_addr = MAX86178_PPG_CFG2_REG;
		ret = max86178_read_reg(sd->spi_dev, &reg_addr, 1);
	}

	if (0 == ret)
	{
		if (reg_addr & 0b00000100)
		{
			*p_is_enabled = 0;
		}
		else
		{
			*p_is_enabled = 1;
		}
	}

	return ret;
}

int max86178_is_ppg2_enabled(uint8_t *p_is_enabled) // OK
{
	int ret = 0;
	uint8_t reg_addr = 0;
	struct max86178_dev *sd = max86178_get_device_data();

	if (p_is_enabled == NULL)
	{
		ret = -1;
	}

	if (0 == ret)
	{
		reg_addr = MAX86178_PPG_CFG2_REG;
		ret = max86178_read_reg(sd->spi_dev, &reg_addr, 1);
	}

	if (0 == ret)
	{
		if (reg_addr & 0b00001000)
		{
			*p_is_enabled = 0;
		}
		else
		{
			*p_is_enabled = 1;
		}
	}

	return ret;
}

int32_t convert_18bit_to_17bit(uint32_t input) {
    // Estrarre il bit di segno (MSB)
    int32_t sign_bit = (input >> 17) & 0x1;

    // Estrarre i 17 bit di dati
    int32_t data_bits = input & 0x1FFFF;

    // Se il bit di segno è 1, convertire i dati in complemento a due
    if (sign_bit == 1) {
        data_bits = -((~data_bits + 1) & 0x1FFFF);
    }

    return data_bits;
}

int max86178_fifo_irq_handler(struct max86178_dev *sd) // OK check for ECG-ACC interaction is required.
{
	uint8_t fifo_buf[MAX86178_MAX_FIFO_DEPTH * MAX86178_DATA_WORD_SIZE];
	int ret;
	int num_samples;
	int ppg_discard = 0;
	fifo_data_t fifo_data;
	uint16_t idx;
	int i;
	static unsigned int ppg_data_number = 0;

	num_samples = max86178_get_num_samples_in_fifo(sd);

	//	printf("num_sample %d count %d\n", num_samples, counter);
	//	counter++;

	if (num_samples <= 0)
	{
		printk("%s:%d failed. err: %d\n", __func__, __LINE__, num_samples);
		return 0;
	}
	printk("Numero Pacchetti letti: %d\n",num_samples);
	ret = max86178_read_fifo_data(sd, fifo_buf, num_samples);
	if (ret < 0)
	{
		printk("%s:%d failed. ret: %d\n", __func__, __LINE__, ret);
		return -1;
	}

	if (fifo_read_cb)
	{
		int ret = fifo_read_cb();
		if (ret == 1)
			ppg_discard = 0;
	}

	/// DEBUG------------------------------------
#ifdef ENABLE_ACCEL_DEBUG
	ppgTimeStamp = (platform_get_time_ms());
#endif
	/// ------------------------------------

	for (i = 0; i < num_samples; i++)
	{
		idx = MAX86178_DATA_WORD_SIZE * i;
		fifo_data.raw = fifo_buf[idx + 0] << 16 | fifo_buf[idx + 1] << 8 | fifo_buf[idx + 2];
		//if (fifo_data.type ==9 || fifo_data.type == 10)  //BIOZ
		if (fifo_data.type ==11 ) // ECG
		//if (fifo_data.type ==9 || fifo_data.type == 10 || fifo_data.type ==11 || fifo_data.type == 14) // ECG
          printk("FIFO TYPE: %d DATA: %d \n ",fifo_data.type, convert_18bit_to_17bit(fifo_data.val) );
		k_msleep(10);
		if (fifo_data.type < DATA_TYPE_PF && ppg_discard)
		{
			continue;
		}

		if (fifo_data.type < DATA_TYPE_PF)
		{ // algo queue for PPG algo processing
			fifo_data_t algo_fifo_data = fifo_data;

			if (algo_fifo_data.val > MAX86178_MAX_PPG_VALUE)
				algo_fifo_data.val = 0;

			// enqueue(&queue_algo, &algo_fifo_data);  todo
			ppg_data_number++;
		}
		

		// ret = enqueue(&sd->queue, &fifo_data);  todo
		if (ret < 0)
		{
			printk("Enqueue data is failed. ret: %d, thrown: %d\n",
				   ret, num_samples - i);
			return -1;
		}
#ifdef ENABLE_ACCEL_DEBUG
		{
			static int pe = 0;
			printf("p:%d\n", pe++);
		}
#endif
	}
     printk("=====================================\n");
	 //k_msleep(10);
	/*
		Please note that this method can only work with single sample reading per interrupt
		from OS58, otherwise it breaks Accel and OS58 sync.
	*/

	uint8_t ppg_enabled, ecg_enabled, ch_enabled, iq_enabled, acc_enabled;
	uint8_t acc_read_count = 0, ppg1 = 0, ppg2 = 0;

	max86178_is_ppg_enabled(&ppg_enabled);
	max86178_is_ecg_enabled(&ecg_enabled);
	max86178_is_acc_enabled(&acc_enabled);
	max86178_is_iq_enabled(&iq_enabled);

	max86178_is_ppg1_enabled(&ppg1);
	max86178_is_ppg2_enabled(&ppg2);
	k_msleep(10);
	printk("Measures Enable: PPG: %d PPG1: %d PPG2: %d ECG: %d ACC: %d IQ: %d\n", ppg_enabled, ppg1, ppg2, ecg_enabled, acc_enabled, iq_enabled);
	k_msleep(10);

	if (0 == ppg1 && 0 == ppg2)
		ppg_enabled = 0;

	if (ppg_enabled)
	{
		ppg1 = ppg1 + ppg2;
		max86178_get_num_of_channel(&ch_enabled);

		if (acc_enabled)
		{
			if (ppg1)
				acc_read_count = ppg_data_number / (ch_enabled * ppg1);
			else
				acc_read_count = 0;
			for (i = 0; i < acc_read_count; i++)
			{
				// lis2dh12_poll_data_buffer(NULL);
				// adxl367_accel.execute_once(NULL);   // todo insert new accel driver icm20948
				ppg_data_number -= (ch_enabled * ppg1);
			}
		}
	}

	return num_samples;
}

int max86178_set_led_current(struct max86178_dev *sd, uint8_t led_num, int led_mA_val) // OK
{
	return 0;
}

int max86178_set_frame_ready_int(uint8_t enable) // OK Check for MRD106 max86178 interrupt line
{

#define FIFO_INT_CTRL_REQ_MRD106 (MAX86178_INT2_ENABLE1_REG)

	int ret = 0;

	uint8_t reg = FIFO_INT_CTRL_REQ_MRD106;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->spi_dev, &reg, 1);

	if (enable)
	{
		reg |= 0x40;
	}
	else
	{
		reg &= 0xBF;
	}

	if (0 == ret)
	{
		ret = max86178_write_reg(sd->spi_dev, FIFO_INT_CTRL_REQ_MRD106, &reg, 1);
	}

	return ret;
}

int max86178_set_a_full_int(uint8_t enable) // OK Check for MRD106 max86178 interrupt line
{

#define FIFO_INT_CTRL_REQ_MRD106 (MAX86178_INT2_ENABLE1_REG)

	int ret = 0;

	uint8_t reg = FIFO_INT_CTRL_REQ_MRD106;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->spi_dev, &reg, 1);

	if (enable)
	{
		reg |= 0x80;
	}
	else
	{
		reg &= 0x7F;
	}

	if (0 == ret)
	{
		ret = max86178_write_reg(sd->spi_dev, FIFO_INT_CTRL_REQ_MRD106, &reg, 1);
	}

	return ret;
}

int max86178_set_fifo_a_full(uint8_t level) // OK
{
	int ret = 0;

	struct max86178_dev *sd = max86178_get_device_data();
    uint8_t data;
	data = MAX86178_MAX_FIFO_DEPTH - level;
	ret = max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG1_REG, &data, 1);

	return ret;
}

void max86178_register_fifo_read_callback(max86178_fifo_read_cb func)
{
	fifo_read_cb = func;
}

void max86178_unregister_fifo_read_callback()
{
	fifo_read_cb = NULL;
}

void max86178_irq_clr()
{
	uint8_t read_buf[3];

	if (max86178_get_irq_state(NULL) > 0)
	{ // OK
		read_buf[0] = MAX86178_STATUS1_REG;
		/* Read three status registers to clear interrupts */
		max86178_read_reg(NULL, read_buf, 3);
		enter_critical_section();
		max86178_irq_cnt--;
		exit_critical_section();
	}
}

void max86178_irq_reset_ref_cnt()
{ // OK
	enter_critical_section();
	max86178_irq_cnt = 0;
	exit_critical_section();
}

int max86178_is_ecg_faster_than_ppg(uint8_t *p_is_true) // OK check for MRD106 conf for PLL/Clock selection - get form ME15 ECG config
{
	int ret = 0;

	uint8_t regs[2] = {0};

	uint32_t ppgFrameRateDiv;
	uint32_t ecgDecRateDiv;

	struct max86178_dev *sd = max86178_get_device_data();

	/*Reading PPG Frame Rate*/
	regs[0] = MAX86178_FR_CLK_DIV_MSB_REG;
	ret = max86178_read_reg(sd->spi_dev, regs, 2);
	if (0 == ret)
	{
		regs[0] &= 0x7F;
		ppgFrameRateDiv = ((uint32_t)regs[0] << 8) | ((uint32_t)regs[1]);

		/*ECG Frame Rate*/
		regs[0] = MAX86178_EGC_CONFIG_1_REG;
		ret = max86178_read_reg(sd->spi_dev, regs, 1);
	}

	if (0 == ret)
	{
		ecgDecRateDiv = (uint32_t)(regs[0] >> 1);
		ecgDecRateDiv &= 0x07;
		ecgDecRateDiv = (1 << ecgDecRateDiv) * 16;

		if (ecgDecRateDiv < ppgFrameRateDiv)
		{
			*p_is_true = 1;
		}
		else
		{
			*p_is_true = 0;
		}
	}

	return ret;
}

int max86178_clear_fifo() //OK
{
	int ret = 0;
	unsigned char byt = 0;
	struct max86178_dev *sd;

	/* Getting sensor descriptor */
	sd = max86178_get_device_data();

	if(NULL == sd)
		ret = -1;

	if(0 == ret)
	{
		byt = MAX86178_FIFO_CFG2_REG;
		ret = max86178_read_reg(sd->spi_dev, &byt, 1);
	}

	if(0 == ret)
	{
		byt |= MAX86178_FLUSH_FIFO_MASK;
		ret = max86178_write_reg(sd->spi_dev, MAX86178_FIFO_CFG2_REG, &byt, sizeof(byt));
	}

	return ret;
}

int max86178_get_num_photo_diodes(uint8_t *p_num_diode) // OK
{
	int ret = 0;

	uint8_t reg = MAX86178_PPG_CFG2_REG;
	struct max86178_dev *sd = max86178_get_device_data();

	ret = max86178_read_reg(sd->spi_dev, &reg, 1);

	reg &= (MAX86178_PPG1_PWRDN_MASK | MAX86178_PPG2_PWRDN_MASK);
	reg >>= 2;

	if (0 == ret)
	{
		if (3 == reg)
			*p_num_diode = 0;
		if (1 == reg || 2 == reg)
			*p_num_diode = 1;
		if (0 == reg)
			*p_num_diode = 2;
	}

	return ret;
}

int spi_rw_test(const struct spi_dt_spec *spi_dev)
{
	int ret;
	uint8_t reg_addr = 0x20;
	uint8_t reg_val = 0x21;

	uint8_t read_byte;

	ret = max86178_write_reg(spi_dev, reg_addr, &reg_val, sizeof(reg_addr));
	// printk("Max86140_SS_Write_Byte %d %d %d\n", ret, reg_addr, reg_val);
	k_msleep(10);

	read_byte = reg_addr;
	ret = max86178_read_reg(spi_dev, &read_byte, sizeof(read_byte));
	// printk("max86178_read_reg %d %d %d\n", ret, reg_addr, read_byte);
	k_msleep(20);

	reg_addr = 0x23;
	reg_val = 0x18;

	ret = max86178_write_reg(spi_dev, reg_addr, &reg_val, sizeof(reg_val));
	// printk("Max86140_SS_Write_Byte %d %d %d\n", ret, reg_addr, reg_val);
	k_msleep(10);
	read_byte = reg_addr;
	ret = max86178_read_reg(spi_dev, &read_byte, sizeof(read_byte));

	// printk("max86178_read_reg %d %d %d\n", ret, reg_addr, read_byte);
	k_msleep(20);

	reg_addr = 0x24;
	reg_val = 0x3F;
	ret = max86178_write_reg(spi_dev, reg_addr, &reg_val, sizeof(reg_val));
	// printk("max86178_read_reg %d %d %d\n", ret, reg_addr, reg_val);
	read_byte = reg_addr;
	ret = max86178_read_reg(spi_dev, &read_byte, sizeof(read_byte));

	k_msleep(20);
	reg_addr = 0x23;
	// read_reg_addr = 0x23;
	read_byte = reg_addr;
	ret = max86178_read_reg(spi_dev, &read_byte, sizeof(read_byte));
	k_msleep(10);

	reg_addr = 0x23;
	reg_val = 0x10;

	ret = max86178_write_reg(spi_dev, reg_addr, &reg_val, sizeof(reg_val));
	// printk("Max86140_SS_Write_Byte %d %d %d\n", ret, reg_addr, reg_val);
	read_byte = reg_addr;
	ret = max86178_read_reg(spi_dev, &read_byte, sizeof(read_byte));

	// printk("max86178_read_reg %d %d %d \n", ret, reg_addr, read_byte);
	k_msleep(10);

	reg_addr = 0x24;
	reg_val = 0x12;
	ret = max86178_write_reg(spi_dev, reg_addr, &reg_val, sizeof(reg_val));
	// printk("Max86140_SS_Write_Byte %d %d %d \n", ret, reg_addr, reg_val);
	read_byte = reg_addr;
	ret = max86178_read_reg(spi_dev, &read_byte, sizeof(read_byte));
	// printk("max86178_read_reg %d %d %d\n", ret, reg_addr, read_byte);
	k_msleep(20);

	return ret;
}