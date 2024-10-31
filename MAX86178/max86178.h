
#ifndef MAX86178_H
#define MAX86178_H
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include "../../utilities/device.h"

/*	MAX86178 Registers	*/


#define MAX86178_SUCCESS    0
#define MAX86178_FAILURE    -1

/* Status registers */
#define MAX86178_STATUS1_REG				0x00
#define MAX86178_PWR_RDY_MASK				(0x01)
#define MAX86178_THRESH1_HILO_MASK			(0x01 << 1)
#define MAX86178_THRESH2_HILO_MASK			(0x01 << 2)
#define MAX86178_EXP_OVF_MASK				(0x01 << 3)
#define MAX86178_ALC_OVF_MASK				(0x01 << 4)
#define MAX86178_FIFO_DATA_RDY_MASK			(0x01 << 5)
#define MAX86178_FRAME_RDY_MASK				(0x01 << 6)
#define MAX86178_A_FULL_MASK				(0x01 << 7)


#define MAX86178_STATUS2_REG				0x01
#define MAX86178_LED1_COMPB_MASK			(0x01)
#define MAX86178_LED2_COMPB_MASK			(0x01 << 1)
#define MAX86178_LED3_COMPB_MASK			(0x01 << 2)
#define MAX86178_LED4_COMPB_MASK			(0x01 << 3)
#define MAX86178_LED5_COMPB_MASK			(0x01 << 4)
#define MAX86178_LED6_COMPB_MASK			(0x01 << 5)
#define MAX86178_INVALID_CFG_MASK			(0x01 << 7)


#define MAX86178_STATUS3_REG				0x02
#define MAX86178_PHASE_LOCK_MASK         	(0x01 << 1)
#define MAX86178_PHASE_UNLOCK_MASK       	(0x01 << 2)
#define MAX86178_FREQ_LOCK_MASK          	(0x01 << 3)
#define MAX86178_FREQ_UNLOCK_MASK        	(0x01 << 4)


#define MAX86178_STATUS4_REG				0x03
#define MAX86178_ECG_LOFF_NL_MASK        	(0x01 << 0)
#define MAX86178_ECG_LOFF_NH_MASK        	(0x01 << 1)
#define MAX86178_ECG_LOFF_PL_MASK        	(0x01 << 2)
#define MAX86178_ECG_LOFF_PH_MASK        	(0x01 << 3)
#define MAX86178_ECG_RLD_OOR_MASK        	(0x01 << 4)
#define MAX86178_ECG_FAST_REC_MASK       	(0x01 << 5)
#define MAX86178_ECG_LEADS_ON_MASK       	(0x01 << 7)


#define MAX86178_STATUS5_REG				0x04
#define MAX86178_BIOZ_LOFF_NL_MASK       	(0x01 << 0)
#define MAX86178_BIOZ_LOFF_NH_MASK       	(0x01 << 1)
#define MAX86178_BIOZ_LOFF_PL_MASK       	(0x01 << 2)
#define MAX86178_BIOZ_LOFF_PH_MASK       	(0x01 << 3)
#define MAX86178_BIOZ_DRV_OOR_MASK       	(0x01 << 4)
#define MAX86178_BIOZ_UNDR_MASK          	(0x01 << 5)
#define MAX86178_BIOZ_OVER_MASK          	(0x01 << 6)
#define MAX86178_BIOZ_LON_MASK           	(0x01 << 7)

/* FIFO registers */
#define MAX86178_FIFO_WR_PTR_REG			0x08
#define MAX86178_FIFO_WR_PTR_MASK			(0xFF)

#define MAX86178_FIFO_RD_PTR_REG			0x09
#define MAX86178_FIFO_RD_PTR_MASK			(0xFF)

#define MAX86178_OVF_CNT_REG				0x0A
#define MAX86178_OVF_CNT_MASK				(0x7F)
#define MAX86178_FIFO_DATA_COUNT_MSB_MASK	(0x01 << 7)

#define MAX86178_FIFO_DATA_CNT_REG			0x0B
#define MAX86178_FIFO_DATA_REG				0x0C

#define MAX86178_FIFO_CFG1_REG				0x0D
#define MAX86178_FIFO_A_FULL_MASK			(0xFF)

#define MAX86178_FIFO_CFG2_REG				0x0E
#define MAX86178_FIFO_RO_MASK				(0x01 << 1)
#define MAX86178_A_FULL_TYPE_MASK			(0x01 << 2)
#define MAX86178_FIFO_STAT_CLR_MASK			(0x01 << 3)
#define MAX86178_FLUSH_FIFO_MASK			(0x01 << 4)
#define MAX86178_FIFO_MARK_MASK				(0x01 << 5)


#define MAX86178_FIFO_RO_STOP				(0x00 << 1)
#define MAX86178_FIFO_RO_PUSH				(0x01 << 1)
#define MAX86178_A_FULL_RPT					(0x00 << 2)
#define MAX86178_A_FULL_ONCE				(0x01 << 2)
#define MAX86178_FIFO_STAT_RD_DATA_NOCLR	(0x00 << 3)
#define MAX86178_FIFO_STAT_RD_DATA_CLR		(0x01 << 3)

#define MAX86178_DATA_MASK					0x000FFFFF
#define MAX86178_TAG_MASK					0x00F00000


/* System Control registers */

#define MAX86178_SYSTEM_SYNC_REG		0x10
#define MAX86178_SYSTEM_TIMING_RESET_MASK	  (0x01 << 7)

#define MAX86178_SYSTEM_CFG1_REG					0x11
#define MAX86178_SYSTEM_RESET_MASK					(0x01)
#define MAX86178_SYSTEM_SHDN_MASK					(0x01 << 1)
#define MAX86178_SYSTEM_ECG_BIOZ_TIMING_DATA_MASK	(0x01 << 3)
#define MAX86178_SYSTEM_BIOZ_PPG_TIMING_DATA_MASK	(0x01 << 4)
#define MAX86178_SYSTEM_ECG_PPG_TIMING_DATA_MASK	(0x01 << 5)
#define MAX86178_SYSTEM_ALL_TIMING_DATA_MASK		(0x07 << 3)
#define MAX86178_DISABLE_I2C_MASK					(0x01 << 6)

#define MAX86178_SYSTEM_CFG2_REG		0x12
#define MAX86178_SYSTEM_ECG_SAMP_SYNC_FREQ_MASK (0x1F)
#define MAX86178_SYSTEM_BYP_DLY_MASK            (1<<7)

/* OLD
#define MAX86178_SYSTEM_CFG2_REG			0x12
#define MAX86178_MEAS1_EN_MASK				(0x01)
#define MAX86178_MEAS2_EN_MASK				(0x01 << 1)
#define MAX86178_MEAS3_EN_MASK				(0x01 << 2)
#define MAX86178_MEAS4_EN_MASK				(0x01 << 3)
#define MAX86178_MEAS5_EN_MASK				(0x01 << 4)
#define MAX86178_MEAS6_EN_MASK				(0x01 << 5)
#define MAX86178_MEAS7_EN_MASK				(0x01 << 6)
#define MAX86178_MEAS8_EN_MASK				(0x01 << 7)
#define MAX86178_MEAS_ALL_EN_MASK			(0xFF)
*/


#define MAX86178_SYSTEM_PIN_FUNC_REG	0x13  // OLD MAX86178_SYSTEM_CFG3_REG
#define MAX86178_SYSTEM_INT1_FCFG_MASK	(0x03 << 0)
#define MAX86178_SYSTEM_INT2_FCFG_MASK	(0x03 << 2)
#define MAX86178_SYSTEM_TRIG_ICFG_MASK	(0x01 << 4)
#define MAX86178_SYSTEM_TRIG_FCFG_MASK	(0x07 << 5)

#define MAX86178_SYSTEM_OUT_PIN_CFG_REG	0x14  // OLD MAX86178_PHOTO_DIODE_BIAS_REG
#define MAX86178_SYSTEM_INT1_OCFG_MASK	(0x03 << 0)
#define MAX86178_SYSTEM_INT2_OCFG_MASK	(0x03 << 2)
#define MAX86178_SYSTEM_TRIG_OCFG_MASK	(0x03 << 6)

#define MAX86178_I2C_BCAST_ADDR_REG		0x15
#define MAX86178_I2C_BCAST_EN_MASK		(0x01)
#define MAX86178_I2C_BCAST_ADDR_MASK	(0xFE)

/* PLL registers */
#define MAX86178_PLL_CFG1_REG	0x18
#define MAX86178_PLL_ENABLE_POS       	  (0)
#define MAX86178_PLL_LOCK_WNDW_POS    	  (1)
#define MAX86178_PLL_MDIV_HIGH_BITS_POS   (6)
#define MAX86178_PLL_ENABLE_MASK       	  (0x01 << MAX86178_PLL_ENABLE_POS)
#define MAX86178_PLL_LOCK_WNDW_MASK    	  (0x01 << MAX86178_PLL_LOCK_WNDW_POS)
#define MAX86178_PLL_MDIV_HIGH_BITS_MASK  (0x03 << MAX86178_PLL_MDIV_HIGH_BITS_POS)


#define MAX86178_PLL_CFG2_REG	0x19
#define MAX86178_PLL_MDIV_LOW_BITS_MASK   (0xFF)


#define MAX86178_PLL_CFG3_REG	0x1A
#define MAX86178_PLL_BIOZ_KDIV_POS      (0)
#define MAX86178_PLL_BIOZ_NDIV_POS      (6)
#define MAX86178_PLL_BIOZ_KDIV_MASK      (0x0F << MAX86178_PLL_BIOZ_KDIV_POS)
#define MAX86178_PLL_BIOZ_NDIV_MASK      (0x03 << MAX86178_PLL_BIOZ_NDIV_POS)


#define MAX86178_PLL_CFG4_REG	0x1B
#define MAX86178_PLL_ECG_FDIV_POS            (0)
#define MAX86178_PLL_ECG_NDIV_HIGH_BITS_POS  (5)
#define MAX86178_PLL_ECG_FDIV_MASK            (0x07 << MAX86178_PLL_ECG_FDIV_POS)
#define MAX86178_PLL_ECG_NDIV_HIGH_BITS_MASK  (0x07 << MAX86178_PLL_ECG_NDIV_HIGH_BITS_POS)


#define MAX86178_PLL_CFG5_REG	0x1C
#define MAX86178_PLL_ECG_NDIV_LOW_BITS_MASK   (0xFF)


#define MAX86178_PLL_CFG6_REG	0x1D
#define MAX86178_PLL_CLK_FINE_TUNE_MASK	(0x1F)
#define MAX86178_PLL_CLK_FREQ_SEL_MASK	(0x01 << 5)
#define MAX86178_PLL_REF_CLK_SEL_MASK	(0x01 << 6)

#define MAX86178_SLOW_CLK_HI	32768u
#define MAX86178_SLOW_CLK_LO	32000u


/* PPG configuration registers */
#define MAX86178_PPG_CFG1_REG			0x20 // OLD  MAX86178_SYSTEM_CFG2_REG
#define MAX86178_MEAS1_EN_MASK			(0x01)
#define MAX86178_MEAS2_EN_MASK			(0x01 << 1)
#define MAX86178_MEAS3_EN_MASK			(0x01 << 2)
#define MAX86178_MEAS4_EN_MASK			(0x01 << 3)
#define MAX86178_MEAS5_EN_MASK			(0x01 << 4)
#define MAX86178_MEAS6_EN_MASK			(0x01 << 5)
#define MAX86178_MEAS_ALL_EN_MASK		(0x3F)

#define MAX86178_PPG_CFG2_REG			0x21
#define MAX86178_PPG1_PWRDN_MASK			(0x01 << 2)
#define MAX86178_PPG2_PWRDN_MASK			(0x01 << 3)
#define MAX86178_SYNC_MODE_MASK				(0x03 << 5)
/*
#define MAX86178_PPG1_CHAN_EN_POS		(2)
#define MAX86178_PPG2_CHAN_EN_POS		(3)
#define MAX86178_PPG1_CHAN_EN_MASK		(0x01 << MAX86178_PPG1_CHAN_EN_POS)
#define MAX86178_PPG2_CHAN_EN_MASK		(0x01 << MAX86178_PPG2_CHAN_EN_POS)
#define MAX86178_PPG_SYNCH_MODE_MASK	(0x01 << 5)
*/

#define MAX86178_SYSTEM_CFG3_REG 0x22
#define MAX86178_PPG_CFG3_REG	 0x22
#define MAX86178_MEAS1_CONFIG_SEL_MASK		(0x01)
#define MAX86178_PPG_MEAS1_CFG_FOR_ALL_MASK (0x01 << 0)
#define MAX86178_PPG_COLLECT_RAW_DATA_MASK	(0x01 << 1)
#define MAX86178_PPG_MWBA_EN_MASK	    	(0x01 << 2)
#define MAX86178_PPG_ALC_DIS_MASK	    	(0x01 << 3)
#define MAX86178_PPG_SAMP_AVERAGE_MASK		(0x07 << 4)
#define MAX86178_PPG_SWAP_DIS_MASK	    	(0x01 << 7)


#define MAX86178_PPG_CFG4_REG	0x23
#define MAX86178_PPG_PROX_AUTO_MASK    (0x01 << 3)
#define MAX86178_PPG_PROX_DATA_EN_MASK (0x01 << 4)


#define MAX86178_PHOTO_DIODE_BIAS_REG   0x24
#define MAX86178_PPG_PD_BIAS_REG	    0x24
#define MAX86178_PD1_BIAS_POS			(0)
#define MAX86178_PD2_BIAS_POS			(2)
#define MAX86178_PD3_BIAS_POS			(4)
#define MAX86178_PD4_BIAS_POS			(6)
#define MAX86178_PPG_PD1_BIAS_MASK    	(0x03 << MAX86178_PD1_BIAS_POS)
#define MAX86178_PPG_PD2_BIAS_MASK    	(0x03 << MAX86178_PD2_BIAS_POS)
#define MAX86178_PPG_PD3_BIAS_MASK    	(0x03 << MAX86178_PD3_BIAS_POS)
#define MAX86178_PPG_PD4_BIAS_MASK    	(0x03 << MAX86178_PD4_BIAS_POS)
#define MAX86178_PD1_BIAS_MASK			(0x03)
#define MAX86178_PD2_BIAS_MASK			(0x03 << 2)
#define MAX86178_PD3_BIAS_MASK			(0x03 << 4)
#define MAX86178_PD4_BIAS_MASK			(0x03 << 6)
#define MAX86178_PD_BIAS_MAX			(0x03)

/* Frame Rate Clock registers */
#define MAX86178_FR_CLK_DIV_MSB_REG     0x28
#define MAX86178_PPG_FRM_RATE_MSB_REG	0x28
#define MAX86178_PPG_FRM_CLK_DIV_MSB_BITS_MASK  (0x7F)
#define MAX86178_FR_CLK_DIV_H_MASK				(0x7F)
#define MAX86178_FR_CLK_32768_DIV_MSB_400HZ		0x00	// MSB=0x01, divider = d82 (0x52)
#define MAX86178_FR_CLK_32000_DIV_MSB_400HZ		0x00	// MSB=0x01, divider = d80 (0x50)
#define MAX86178_FR_CLK_32768_DIV_MSB_200HZ		0x00	// MSB=0x01, divider = d164 (0xA4)
#define MAX86178_FR_CLK_32000_DIV_MSB_200HZ		0x00	// MSB=0x01, divider = d160 (0xA0)
#define MAX86178_FR_CLK_32768_DIV_MSB_100HZ		0x01	// MSB=0x01, divider = d328 (0x148)
#define MAX86178_FR_CLK_32000_DIV_MSB_100HZ		0x01	// MSB=0x01, divider = d320 (0x140)
#define MAX86178_FR_CLK_32768_DIV_MSB_50HZ		0x02	// MSB=0x05, divider = d655 (0x28F)
#define MAX86178_FR_CLK_32000_DIV_MSB_50HZ		0x02	// MSB=0x05, divider = d640 (0x280)
#define MAX86178_FR_CLK_32768_DIV_MSB_25HZ		0x05	// MSB=0x05, divider = d1311 (0x51F)
#define MAX86178_FR_CLK_32000_DIV_MSB_25HZ		0x05	// MSB=0x05, divider = d1280 (0x500)



#define MAX86178_FR_CLK_DIV_LSB_REG     0x29
#define MAX86178_PPG_FRM_RATE_LSB_REG	0x29
#define MAX86178_PPG_FRM_CLK_DIV_LSB_BITS_MASK (0xFF)
#define MAX86178_FR_CLK_DIV_L_MASK			(0xFF)
#define MAX86178_FR_CLK_32768_DIV_LSB_400HZ		0x52	// MSB=0x01, divider = d82 (0x52)
#define MAX86178_FR_CLK_32000_DIV_LSB_400HZ		0x50	// MSB=0x01, divider = d80 (0x50)
#define MAX86178_FR_CLK_32768_DIV_LSB_200HZ		0xA4	// MSB=0x01, divider = d164 (0xA4)
#define MAX86178_FR_CLK_32000_DIV_LSB_200HZ		0xA0	// MSB=0x01, divider = d160 (0xA0)
#define MAX86178_FR_CLK_32768_DIV_LSB_100HZ		0x48	// LSB=0x48, divider = d328 (0x148)
#define MAX86178_FR_CLK_32000_DIV_LSB_100HZ		0x40	// LSB=0x40, divider = d320 (0x140)
#define MAX86178_FR_CLK_32768_DIV_LSB_50HZ		0x8F	// MSB=0x05, divider = d655 (0x28F)
#define MAX86178_FR_CLK_32000_DIV_LSB_50HZ		0x80	// MSB=0x05, divider = d640 (0x280)
#define MAX86178_FR_CLK_32768_DIV_LSB_25HZ		0x1F	// LSB=0x1F, divider = d1311 (0x51F)
#define MAX86178_FR_CLK_32000_DIV_LSB_25HZ		0x00	// LSB=0x00, divider = d1280 (0x500)



/* MEAS1 Setup registers */
#define MAX86178_MEAS1_SELECT_REG		0x30
#define MAX86178_MEAS1_CFG1_REG			0x31
#define MAX86178_MEAS1_CFG2_REG			0x32
#define MAX86178_MEAS1_CFG3_REG			0x33
#define MAX86178_MEAS1_CFG4_REG			0x34
#define MAX86178_MEAS1_CFG5_REG			0x35
#define MAX86178_MEAS1_LED_A_REG		0x36
#define MAX86178_MEAS1_LED_B_REG		0x37

/* MEAS2 Setup registers */
#define MAX86178_MEAS2_SELECT_REG		0x38
#define MAX86178_MEAS2_CFG1_REG			0x39
#define MAX86178_MEAS2_CFG2_REG			0x3A
#define MAX86178_MEAS2_CFG3_REG			0x3B
#define MAX86178_MEAS2_CFG4_REG			0x3C
#define MAX86178_MEAS2_CFG5_REG			0x3D
#define MAX86178_MEAS2_LED_A_REG		0x3E
#define MAX86178_MEAS2_LED_B_REG		0x3F

/* MEAS3 Setup registers */
#define MAX86178_MEAS3_SELECT_REG		0x40
#define MAX86178_MEAS3_CFG1_REG			0x41
#define MAX86178_MEAS3_CFG2_REG			0x42
#define MAX86178_MEAS3_CFG3_REG			0x43
#define MAX86178_MEAS3_CFG4_REG			0x44
#define MAX86178_MEAS3_CFG5_REG			0x45
#define MAX86178_MEAS3_LED_A_REG		0x46
#define MAX86178_MEAS3_LED_B_REG		0x47


/* MEAS4 Setup registers */
#define MAX86178_MEAS4_SELECT_REG		0x48
#define MAX86178_MEAS4_CFG1_REG			0x49
#define MAX86178_MEAS4_CFG2_REG			0x4A
#define MAX86178_MEAS4_CFG3_REG			0x4B
#define MAX86178_MEAS4_CFG4_REG			0x4C
#define MAX86178_MEAS4_CFG5_REG			0x4D
#define MAX86178_MEAS4_LED_A_REG		0x4E
#define MAX86178_MEAS4_LED_B_REG		0x4F


/* MEAS5 Setup registers */
#define MAX86178_MEAS5_SELECT_REG		0x50
#define MAX86178_MEAS5_CFG1_REG			0x51
#define MAX86178_MEAS5_CFG2_REG			0x52
#define MAX86178_MEAS5_CFG3_REG			0x53
#define MAX86178_MEAS5_CFG4_REG			0x54
#define MAX86178_MEAS5_CFG5_REG			0x55
#define MAX86178_MEAS5_LED_A_REG		0x56
#define MAX86178_MEAS5_LED_B_REG		0x57


/* MEAS6 Setup registers */
#define MAX86178_MEAS6_SELECT_REG		0x58
#define MAX86178_MEAS6_CFG1_REG			0x59
#define MAX86178_MEAS6_CFG2_REG			0x5A
#define MAX86178_MEAS6_CFG3_REG			0x5B
#define MAX86178_MEAS6_CFG4_REG			0x5C
#define MAX86178_MEAS6_CFG5_REG			0x5D
#define MAX86178_MEAS6_LED_A_REG		0x5E
#define MAX86178_MEAS6_LED_B_REG		0x5F


/* MEASxSetup masks */
#define MAX86178_MEAS_DRV_PA_MASK			(0xFF)
//MEASx Selects
#define MAX86178_MEAS_DRVA_MASK				(0x07)
#define MAX86178_MEAS_DRVB_MASK				(0x07 << 3)
#define MAX86178_MEAS_AMB_MASK				(0x01 << 6)

//MEASx Config1
#define MAX86178_MEAS_AVER_MASK				(0x07)
#define MAX86178_MEAS_TINT_MASK				(0x03 << 3)

//MEASx Config2
#define MAX86178_MEAS_PPG1_ADC_RGE_MASK		(0x03)
#define MAX86178_MEAS_PPG2_ADC_RGE_MASK		(0x03 << 2)
#define MAX86178_MEAS_LED_RGE_MASK			(0x03 << 4)
#define MAX86178_MEAS_FILT_SEL_MASK			(0x01 << 6)
#define MAX86178_MEAS_SINC3_SEL_MASK		(0x01 << 7)

//MEASx Config3
#define MAX86178_MEAS_PPG1_DAC_OFF_MASK		(0x0F)
#define MAX86178_MEAS_PPG2_DAC_OFF_MASK		(0x0F << 4)
#define MAX86178_MEAS_LED_SETLNG_MASK		(0x03 << 4)
#define MAX86178_MEAS_PD_SETLNG_MASK		(0x03 << 6)

//MEASx Config4
#define MAX86178_MEAS_PPG1_PDSEL_MASK		(0x03)
#define MAX86178_MEAS_PPG2_PDSEL_MASK		(0x03 << 2)
#define MAX86178_MEAS_PPG_GAIN_MASK			(0x03 << 4)
#define MAX86178_MEAS_BUFCHAN_MASK			(0x01 << 6)

/* LED Driver Values */
#define MAX86178_LED_DRV_A_PIN_1_MASK		(0x00)
#define MAX86178_LED_DRV_A_PIN_2_MASK		(0x01)
#define MAX86178_LED_DRV_A_PIN_3_MASK		(0x02)
#define MAX86178_LED_DRV_A_PIN_4_MASK		(0x03)
#define MAX86178_LED_DRV_A_PIN_5_MASK		(0x04)
#define MAX86178_LED_DRV_A_PIN_6_MASK		(0x05)

#define MAX86178_LED_DRV_B_PIN_1_MASK		(0x00 << 3)
#define MAX86178_LED_DRV_B_PIN_2_MASK		(0x01 << 3)
#define MAX86178_LED_DRV_B_PIN_3_MASK		(0x02 << 3)
#define MAX86178_LED_DRV_B_PIN_4_MASK		(0x03 << 3)
#define MAX86178_LED_DRV_B_PIN_5_MASK		(0x04 << 3)
#define MAX86178_LED_DRV_B_PIN_6_MASK		(0x05 << 3)

/* TINT  Values */
#define MAX86178_TINT_14_US_MASK			(0 << 3)
#define MAX86178_TINT_29_US_MASK			(1 << 3)
#define MAX86178_TINT_59_US_MASK			(2 << 3)
#define MAX86178_TINT_117_US_MASK			(3 << 3)

/* PPG PD Select values */
#define MAX86178_PPG1_PD1_MASK				(0x00)
#define MAX86178_PPG1_PD2_MASK				(0x01)
#define MAX86178_PPG1_PD3_MASK				(0x02)
#define MAX86178_PPG1_PD4_MASK				(0x03)
#define MAX86178_PPG2_PD1_MASK				(0x00 << 2)
#define MAX86178_PPG2_PD2_MASK				(0x01 << 2)
#define MAX86178_PPG2_PD3_MASK				(0x02 << 2)
#define MAX86178_PPG2_PD4_MASK				(0x03 << 2)

#define MAX86178_PPG_GAIN_1_MASK			(0x00 << 4)
#define MAX86178_PPG_GAIN_2_MASK			(0x01 << 4)
#define MAX86178_PPG_GAIN_4_MASK			(0x02 << 4)

#define MAX86178_PPG_BUFFCHAN_ADC1_MASK		(0x00 << 6)
#define MAX86178_PPG_BUFFCHAN_ADC2_MASK		(0x01 << 6)

/* ADC RGE Values */
#define MAX86178_PPG1_ADC_RGE_4uA_MASK		(0x00)
#define MAX86178_PPG1_ADC_RGE_8uA_MASK		(0x01)
#define MAX86178_PPG1_ADC_RGE_16uA_MASK		(0x02)
#define MAX86178_PPG1_ADC_RGE_32uA_MASK		(0x03)

#define MAX86178_PPG2_ADC_RGE_4uA_MASK		(0x00 << 2)
#define MAX86178_PPG2_ADC_RGE_8uA_MASK		(0x01 << 2)
#define MAX86178_PPG2_ADC_RGE_16uA_MASK		(0x02 << 2)
#define MAX86178_PPG2_ADC_RGE_32uA_MASK		(0x03 << 2)

/* LED RGE Values */
#define MAX86178_LED_RGE_32mA_MASK			(0x00 << 4)
#define MAX86178_LED_RGE_64mA_MASK			(0x01 << 4)
#define MAX86178_LED_RGE_96mA_MASK			(0x02 << 4)
#define MAX86178_LED_RGE_128mA_MASK			(0x03 << 4)

/* Average Values */
#define MAX86178_AVERAGE_1_MASK				(0)
#define MAX86178_AVERAGE_2_MASK				(1)
#define MAX86178_AVERAGE_4_MASK				(2)
#define MAX86178_AVERAGE_8_MASK				(3)
#define MAX86178_AVERAGE_16_MASK			(4)
#define MAX86178_AVERAGE_32_MASK			(5)
#define MAX86178_AVERAGE_64_MASK			(6)
#define MAX86178_AVERAGE_128_MASK			(7)


/* Threshold interrupt registers */
#define MAX86178_THR_MEAS_SEL_REG			0x70
	#define MAX86178_THR1_MEAS_SEL_MASK			(0xF)
	#define MAX86178_THR2_MEAS_SEL_MASK			(0xF << 4)

#define MAX86178_THR_HYST_REG				0x71
	#define MAX86178_LEVEL_HYST_MASK			(0x7)
	#define MAX86178_TIME_HYST_MASK				(0x3 << 3)
	#define MAX86178_THRESH1_PPG_SEL_MASK		(0x1 << 6)
	#define MAX86178_THRESH2_PPG_SEL_MASK		(0x1 << 7)

	#define MAX86178_PPG_UPPER_THR1_REG			0x72
	#define MAX86178_PPG_LOWER_THR1_REG			0x73
	#define MAX86178_PPG_UPPER_THR2_REG			0x74
	#define MAX86178_PPG_LOWER_THR2_REG			0x75


/* Picket Fence registers */
/*
#define MAX86178_PICKET_FENCE_SEL_REG		0x70
#define MAX86178_PPG1_PF_MEAS_SEL_MASK		(0xF)
#define MAX86178_PPG2_PF_MEAS_SEL_MASK		(0xF << 4)

#define MAX86178_PICKET_FENCE_CFG_REG		0x71
#define MAX86178_THRESH_SIGMA_MULT_MASK		(0x3 << 0)
#define MAX86178_IIR_INIT_VAL_MASK			(0x3 << 2)
#define MAX86178_IIR_TC_MASK				(0x3 << 4)
#define MAX86178_PF_ORDER_MASK				(0x1 << 6)*/

/* Interrupts enable registers */
#define MAX86178_INT1_ENABLE1_REG		0xC0
	#define MAX86178_THRESH1_HILO_EN_MASK	(0x01 << 1)
	#define MAX86178_THRESH2_HILO_EN_MASK	(0x01 << 2)
	#define MAX86178_EXP_OVF_EN_MASK		(0x01 << 3)
	#define MAX86178_ALC_OVF_EN_MASK		(0x01 << 4)
	#define MAX86178_FIFO_DATA_RDY_EN_MASK	(0x01 << 5)
	#define MAX86178_FRAME_RDY_EN_MASK		(0x01 << 6)
	#define MAX86178_A_FULL_EN_MASK			(0x01 << 7)

#define MAX86178_INT1_ENABLE2_REG		0xC1
	#define MAX86178_LED1_COMPB_EN_MASK		(0x01)
	#define MAX86178_LED2_COMPB_EN_MASK		(0x01 << 1)
	#define MAX86178_LED3_COMPB_EN_MASK		(0x01 << 2)
	#define MAX86178_LED4_COMPB_EN_MASK		(0x01 << 3)
	#define MAX86178_LED5_COMPB_EN_MASK		(0x01 << 4)
	#define MAX86178_LED6_COMPB_EN_MASK		(0x01 << 5)
	#define MAX86178_INVALID_CFG_EN_MASK	(0x01 << 7)

/*INT1 PLL*/
#define MAX86178_INT1_ENABLE3_REG		0xC2
	#define MAX86178_PHASE_LOCK_EN_MASK      (0x01 << 1)
	#define MAX86178_PHASE_UNLOCK_EN_MASK    (0x01 << 2)
	#define MAX86178_FREQ_LOCK_EN_MASK       (0x01 << 3)
	#define MAX86178_FREQ_UNLOCK_EN_MASK     (0x01 << 4)

/*INT1 ECG*/
#define MAX86178_INT1_ENABLE4_REG		0xC3
	#define MAX86178_ECG_LOFF_NL_EN_MASK     (0x01 << 0)
	#define MAX86178_ECG_LOFF_NH_EN_MASK     (0x01 << 1)
	#define MAX86178_ECG_LOFF_PL_EN_MASK     (0x01 << 2)
	#define MAX86178_ECG_LOFF_PH_EN_MASK     (0x01 << 3)
	#define MAX86178_ECG_RLD_OOR_EN_MASK     (0x01 << 4)
	#define MAX86178_ECG_FAST_REC_EN_MASK    (0x01 << 5)
	#define MAX86178_ECG_LEADS_ON_EN_MASK    (0x01 << 7)

/*INT1 BIOZ*/
#define MAX86178_INT1_ENABLE5_REG		0xC4
	#define MAX86178_BIOZ_LOFF_NL_EN_MASK     (0x01 << 0)
	#define MAX86178_BIOZ_LOFF_NH_EN_MASK     (0x01 << 1)
	#define MAX86178_BIOZ_LOFF_PL_EN_MASK     (0x01 << 2)
	#define MAX86178_BIOZ_LOFF_PH_EN_MASK     (0x01 << 3)
	#define MAX86178_BIOZ_DRVP_OFF_EN_MASK    (0x01 << 4)
	#define MAX86178_BIOZ_UNDR_EN_MASK        (0x01 << 5)
	#define MAX86178_BIOZ_OVER_EN_MASK        (0x01 << 6)
	#define MAX86178_BIOZ_LEADS_ON_EN_MASK    (0x01 << 7)

/*INT2 registers*/
#define MAX86178_INT2_ENABLE1_REG		0xC5
	#define MAX86178_THRESH1_HILO_EN2_MASK	  (0x01 << 1)
	#define MAX86178_THRESH2_HILO_EN2_MASK	  (0x01 << 2)
	#define MAX86178_EXP_OVF_EN2_MASK		  (0x01 << 3)
	#define MAX86178_ALC_OVF_EN2_MASK		  (0x01 << 4)
	#define MAX86178_FIFO_DATA_RDY_EN2_MASK	  (0x01 << 5)
	#define MAX86178_PPG_FRAME_RDY_EN2_MASK   (0x01 << 6)
	#define MAX86178_A_FULL_EN2_MASK		  (0x01 << 7)


#define MAX86178_INT2_ENABLE2_REG		0xC6
	#define MAX86178_LED1_COMPB_EN2_MASK	  (0x01)
	#define MAX86178_LED2_COMPB_EN2_MASK	  (0x01 << 1)
	#define MAX86178_LED3_COMPB_EN2_MASK	  (0x01 << 2)
	#define MAX86178_LED4_COMPB_EN2_MASK	  (0x01 << 3)
	#define MAX86178_LED5_COMPB_EN2_MASK	  (0x01 << 4)
	#define MAX86178_LED6_COMPB_EN2_MASK	  (0x01 << 5)
	#define MAX86178_INVALID_CFG_EN2_MASK	  (0x01 << 7)


/*INT2 PLL*/
#define MAX86178_INT2_ENABLE3_REG		0xC7
	#define MAX86178_PHASE_LOCK_EN2_MASK      (0x01 << 1)
	#define MAX86178_PHASE_UNLOCK_EN2_MASK    (0x01 << 2)
	#define MAX86178_FREQ_LOCK_EN2_MASK       (0x01 << 3)
	#define MAX86178_FREQ_UNLOCK_EN2_MASK     (0x01 << 4)

/*INT2 ECG*/
#define MAX86178_INT2_ENABLE4_REG		0xC8
	#define MAX86178_ECG_LOFF_NL_EN2_MASK     (0x01 << 0)
	#define MAX86178_ECG_LOFF_NH_EN2_MASK     (0x01 << 1)
	#define MAX86178_ECG_LOFF_PL_EN2_MASK     (0x01 << 2)
	#define MAX86178_ECG_LOFF_PH_EN2_MASK     (0x01 << 3)
	#define MAX86178_ECG_RLD_OOR_EN2_MASK     (0x01 << 4)
	#define MAX86178_ECG_FAST_REC_EN2_MASK    (0x01 << 5)
	#define MAX86178_ECG_LEADS_ON_EN2_MASK    (0x01 << 7)


/*INT2 BIOZ*/
#define MAX86178_INT2_ENABLE5_REG		0xC9
	#define MAX86178_BIOZ_LOFF_NL_EN2_MASK     (0x01 << 0)
	#define MAX86178_BIOZ_LOFF_NH_EN2_MASK     (0x01 << 1)
	#define MAX86178_BIOZ_LOFF_PL_EN2_MASK     (0x01 << 2)
	#define MAX86178_BIOZ_LOFF_PH_EN2_MASK     (0x01 << 3)
	#define MAX86178_BIOZ_DRVP_OFF_EN2_MASK    (0x01 << 4)
	#define MAX86178_BIOZ_UNDR_EN2_MASK        (0x01 << 5)
	#define MAX86178_BIOZ_OVER_EN2_MASK        (0x01 << 6)
	#define MAX86178_BIOZ_LEADS_ON_EN2_MASK    (0x01 << 7)


#define MAX86178_TIMING_SYS_RESET_EN_MASK	(0x01 << 7) // ??????????
#define MAX86178_VDD_OOR_EN_MASK			(0x01 << 6) // ??????????





/* EGC Configuration registers */
#define MAX86178_ECG_CFG1_REG				0x80
#define MAX86178_EGC_CONFIG_1_REG			0x80
	#define MAX86178_ECG_ENABLE_POS		(0)
	#define MAX86178_ECG_DEC_RATE_POS	(1)
	#define MAX86178_ECG_ENABLE_MASK	(0x01 << MAX86178_ECG_ENABLE_POS)
	#define MAX86178_ECG_DEC_RATE_MASK	(0x07 << MAX86178_ECG_DEC_RATE_POS)

	#define MAX86178_ECG_DEC_RATE_16   0x00
	#define MAX86178_ECG_DEC_RATE_32   0x01
	#define MAX86178_ECG_DEC_RATE_64   0x02
	#define MAX86178_ECG_DEC_RATE_128  0x03
	#define MAX86178_ECG_DEC_RATE_256  0x04
	#define MAX86178_ECG_DEC_RATE_512  0x05


#define MAX86178_ECG_CFG2_REG				0x81
#define MAX86178_EGC_CONFIG_2_REG			0x81
	#define MAX86178_ECG_INA_GAIN_MASK	(0x03 << 0)
	#define MAX86178_ECG_INA_RGE_MASK	(0x03 << 2)  /*?*/
	#define MAX86178_ECG_PGA_GAIN_MASK	(0x07 << 4)
	#define MAX86178_ECG_IPOL_MASK	    (0x01 << 7)

	#define MAX86178_ECG_PGA_GAIN_1    (0x00)
	#define MAX86178_ECG_PGA_GAIN_2    (0x01)
	#define MAX86178_ECG_PGA_GAIN_4    (0x02)
	#define MAX86178_ECG_PGA_GAIN_8    (0x03)
	#define MAX86178_ECG_PGA_GAIN_16   (0x07)

	#define MAX86178_ECG_INA_GAIN_20    (0x00)
	#define MAX86178_ECG_INA_GAIN_30    (0x01)
	#define MAX86178_ECG_INA_GAIN_40    (0x02)
	#define MAX86178_ECG_INA_GAIN_60    (0x03)


#define MAX86178_ECG_CFG3_REG				0x82
#define MAX86178_EGC_CONFIG_3_REG			0x82
	#define MAX86178_ECG_MUX_SEL_MASK	(0x03 << 0)
	#define MAX86178_ECG_AUTO_REC_MASK	(0x01 << 2)
	#define MAX86178_ECG_IMP_HI_MASK	(0x01 << 3)


#define MAX86178_ECG_CFG4_REG				0x83
	#define MAX86178_ECG_FAST_REC_THRESH_MASK	(0x1F << 0)
	#define MAX86178_ECG_EN_FAST_REC_MASK	    (0x03 << 6)

	#define MAX86178_ECG_FAST_REC_EN_NORMAL_MODE   (0x00)
	#define MAX86178_ECG_FAST_REC_EN_MANUAL_MODE   (0x01)
	#define MAX86178_ECG_FAST_REC_EN_AUTO_MODE     (0x02)


/* ECG Calibration registers*/

#define MAX86178_CAL_CFG_1_REG         0x84
#define MAX86178_ECG_CAL_CFG1_REG	   0x84
	#define MAX86178_ECG_CAL_ENABLE_MASK  (0x01 << 0)
	#define MAX86178_ECG_CAL_DUTY_MASK    (0x01 << 1)
	#define MAX86178_ECG_CAL_FREQ_MASK    (0x07 << 2)
	#define MAX86178_ECG_CAL_HIGH_MSB_MASK     (0x07 << 5)

	#define MAX86178_ECG_CAL_FREQ_256       (0x00)
	#define MAX86178_ECG_CAL_FREQ_64        (0x01)
	#define MAX86178_ECG_CAL_FREQ_16        (0x02)
	#define MAX86178_ECG_CAL_FREQ_4         (0x03)
	#define MAX86178_ECG_CAL_FREQ_1         (0x04)
	#define MAX86178_ECG_CAL_FREQ_1_DIV_4   (0x05)
	#define MAX86178_ECG_CAL_FREQ_1_DIV_16  (0x06)
	#define MAX86178_ECG_CAL_FREQ_1_DIV_64  (0x07)


#define MAX86178_CAL_CFG_2_REG		0x85
#define MAX86178_ECG_CAL_CFG2_REG	0x85
	#define MAX86178_ECG_CAL_HIGH_LSB_MASK     (0xFF)


#define MAX86178_CAL_CFG_3_REG      0x86
#define MAX86178_ECG_CAL_CFG3_REG	0x86
	#define MAX86178_ECG_CAL_N_SEL_MASK  (0x03 << 0)
	#define MAX86178_ECG_CAL_P_SEL_MASK  (0x03 << 2)
	#define MAX86178_ECG_CAL_MAG_MASK    (0x01 << 4)
	#define MAX86178_ECG_CAL_MODE_MASK   (0x01 << 5)
	#define MAX86178_ECG_OPEN_N_MASK     (0x01 << 6)
	#define MAX86178_ECG_OPEN_P_MASK     (0x01 << 7)



/* ECG Lead detect registers*/
#define MAX86178_LEAD_DETECT_CFG_1_REG		0x88
#define MAX86178_ECG_LEAD_DETECT_CFG1_REG	0x88
#define MAX86178_ECG_LOFF_FREQ_MASK          (0x07 << 0)
#define MAX86178_ECG_LOFF_MODE_MASK          (0x01 << 3)
#define MAX86178_ECG_LOFF_ENABLE_MASK        (0x01 << 6)
#define MAX86178_ECG_LON_ENABLE_MASK         (0x01 << 7)



#define MAX86178_LEAD_DETECT_CFG_2_REG		0x89
#define MAX86178_ECG_LEAD_DETECT_CFG2_REG	0x89
#define MAX86178_ECG_LOFF_THRESH_MASK         (0x0F << 0)
#define MAX86178_ECG_LOFF_IMAG_MASK           (0x07 << 4)
#define MAX86178_ECG_LOFF_IPOL_MASK           (0x01 << 7)


/* ECG Lead bias registers*/
#define MAX86178_LEAD_BIAS_CFG_1_REG		0x90 // same as old
#define MAX86178_ECG_LEAD_BIAS_CFG_REG		0x90
	#define MAX86178_ECG_RBIAS_N_ENABLE_MASK   (0x01 << 0)
	#define MAX86178_ECG_RBIAS_P_ENABLE_MASK   (0x01 << 1)
	#define MAX86178_ECG_RBIAS_VALUE_MASK      (0x03 << 2)


/*ECG ECG RLD and CM Amps registers*/
#define MAX86178_RLD_CFG_1_REG		0x92 // same as old
#define MAX86178_ECG_RLD_CFG1_REG	0x92
	#define MAX86178_ECG_RLD_GAIN_MASK       (0x03 << 0)
	#define MAX86178_ECG_RLD_ACTV_CM_N_MASK  (0x01 << 2)
	#define MAX86178_ECG_RLD_ACTV_CM_P_MASK  (0x01 << 3)
	#define MAX86178_ECG_RLD_OOR_EN_MASK     (0x01 << 4)
	#define MAX86178_ECG_RLD_RBIAS_MASK      (0x01 << 5)
	#define MAX86178_ECG_RLD_MODE_MASK       (0x01 << 6)
	#define MAX86178_ECG_RLD_ENABLE_MASK     (0x01 << 7)

#define MAX86178_RLD_CFG_2_REG		0x93 // same as old
#define MAX86178_ECG_RLD_CFG2_REG	0x93
	#define MAX86178_ECG_BODY_BIAS_DAC_MASK  (0x0F << 0)
	#define MAX86178_ECG_RLD_BW_MASK         (0x03 << 4)
	#define MAX86178_ECG_RLD_ECG_SEL_MASK    (0x01 << 6)
	#define MAX86178_ECG_RLD_EXT_RES_MASK    (0x01 << 7)





/*********BIOZ*********/


#define MAX86178_BIOZ_CFG1_REG	0xA0
#define MAX86178_BIOZ_ENABLE_POS      (0)
#define MAX86178_BIOZ_ECG_BG_EN_POS   (2)
#define MAX86178_BIOZ_ADC_OSR_POS     (3)
#define MAX86178_BIOZ_DAC_OSR_POS     (6)
#define MAX86178_BIOZ_ENABLE_MASK     (0x03 << MAX86178_BIOZ_ENABLE_POS)
#define MAX86178_BIOZ_ECG_BG_EN_MASK  (0x01 << MAX86178_BIOZ_ECG_BG_EN_POS)
#define MAX86178_BIOZ_ADC_OSR_MASK    (0x07 << MAX86178_BIOZ_ADC_OSR_POS)
#define MAX86178_BIOZ_DAC_OSR_MASK    (0x03 << MAX86178_BIOZ_DAC_OSR_POS)


#define MAX86178_BIOZ_CFG2_REG	0xA1
#define MAX86178_BIOZ_THRESH_EN_MASK  (0x01 << 0)
#define MAX86178_BIOZ_DLPF_MASK       (0x07 << 3)
#define MAX86178_BIOZ_DHPF_MASK       (0x03 << 6)


#define MAX86178_BIOZ_CFG3_REG	0xA2
#define MAX86178_BIOZ_DRV_MODE_MASK    (0x03 << 0)
#define MAX86178_BIOZ_IDRV_RGE_MASK    (0x03 << 2)
#define MAX86178_BIOZ_VDRV_MAG_MASK    (0x03 << 4)
#define MAX86178_BIOZ_EXT_RES_EN_MASK  (0x01 << 7)

#define MAX86178_BIOZ_IDRV_RGE_552K5    (0x00) /* VDRV magnitude reduced by 4x*/
#define MAX86178_BIOZ_IDRV_RGE_110K5    (0x01)
#define MAX86178_BIOZ_IDRV_RGE_5K525    (0x02)
#define MAX86178_BIOZ_IDRV_RGE_276K25   (0x03)



#define MAX86178_BIOZ_CFG4_REG	0xA3
#define MAX86178_BIOZ_UTIL_MODE_EN_MASK    (0x01 << 0)


#define MAX86178_BIOZ_CFG5_REG	0xA4
#define MAX86178_BIOZ_DC_CODE_SEL_MASK  (0x01 << 0)
#define MAX86178_BIOZ_DC_DAC_CODE_MASK  (0xFE << 0)


#define MAX86178_BIOZ_CFG6_REG	0xA5
#define MAX86178_BIOZ_GAIN_MASK        (0x03 << 0)
#define MAX86178_BIOZ_DM_DISABLE_MASK  (0x01 << 2)
#define MAX86178_BIOZ_INA_MODE_MASK    (0x01 << 3)
#define MAX86178_BIOZ_AHPF_MASK        (0x0F << 4)




#define MAX86178_BIOZ_CFG7_REG	0xA6
#define MAX86178_BIOZ_AMP_BW_MASK             (0x03 << 0)
#define MAX86178_BIOZ_AMP_RGE_MASK            (0x03 << 2)
#define MAX86178_BIOZ_DAC_RESET_MASK          (0x01 << 4)
#define MAX86178_BIOZ_DRV_RESET_MASK          (0x01 << 5)
#define MAX86178_BIOZ_DC_RESTORE_SWITCH_MASK  (0x01 << 6)
#define MAX86178_BIOZ_EXT_CAP_SEL_MASK        (0x01 << 7)


#define MAX86178_BIOZ_CFG8_REG	0xA7
#define MAX86178_BIOZ_CH_FSEL_MASK        (0x01 << 0)
#define MAX86178_BIOZ_INA_CHOP_EN_MASK    (0x01 << 1)
#define MAX86178_BIOZ_FAST_MASK           (0x01 << 2)
#define MAX86178_BIOZ_IPOL_MASK           (0x01 << 3)
#define MAX86178_BIOZ_STBYON_MASK         (0x01 << 4)
#define MAX86178_BIOZ_CMRES_DIS_MASK      (0x01 << 5)
#define MAX86178_BIOZ_RDL_DRV_MASK        (0x01 << 6)
#define MAX86178_BIOZ_RDL_SEL_MASK        (0x01 << 7)


#define MAX86178_BIOZ_LOW_THRESH_REG	0xA8
#define MAX86178_BIOZ_LOW_THRESH_MASK        (0xFF)


#define MAX86178_BIOZ_HIGH_THRESH_REG	0xA9
#define MAX86178_BIOZ_HIGH_THRESH_MASK        (0xFF)


#define MAX86178_BIOZ_MUX_CFG1_REG		0xAA
#define MAX86178_BIOZ_CAL_ENABLE_MASK	(0x1 << 0)
#define MAX86178_BIOZ_MUX_ENABLE_MASK	(0x1 << 1)
#define MAX86178_BIOZ_BMUX_BIST_EN_MASK	(0x1 << 5)
#define MAX86178_BIOZ_BMUX_RSEL_MASK	(0x3 << 6)


#define MAX86178_BIOZ_MUX_CFG2_REG		0xAB
#define MAX86178_BIOZ_INT_INLOAD_EN_MASK	(0x1 << 0)
#define MAX86178_BIOZ_EXT_INLOAD_EN_MASK	(0x1 << 1)
#define MAX86178_BIOZ_GSR_LOAD_EN_MASK	    (0x1 << 5)
#define MAX86178_BIOZ_BMUX_GSR_RSEL_MASK	(0x3 << 6)


#define MAX86178_BIOZ_MUX_CFG3_REG		0xAC
#define MAX86178_BIOZ_DRVN_ASSIGN_MASK	(0x3 << 0)
#define MAX86178_BIOZ_DRVP_ASSIGN_MASK	(0x3 << 2)
#define MAX86178_BIOZ_BIN_ASSIGN_MASK	(0x3 << 4)
#define MAX86178_BIOZ_BIP_ASSIGN_MASK	(0x3 << 6)


#define MAX86178_BIOZ_MUX_CFG4_REG		0xAD
#define MAX86178_BIOZ_BIST_R_ERR_MASK   (0xFF)


#define MAX86178_BIOZ_LEAD_DETCT_CFG1_REG  0xB0
#define MAX86178_BIOZ_LOFF_IMAG_MASK		(0x7 << 0)
#define MAX86178_BIOZ_LOFF_IPOL_MASK		(0x1 << 3)
#define MAX86178_BIOZ_DRV_OOR_EN_MASK		(0x1 << 4)
#define MAX86178_BIOZ_EXT_LOFF_EN_MASK		(0x1 << 5)
#define MAX86178_BIOZ_LOFF_EN_MASK	    	(0x1 << 6)
#define MAX86178_BIOZ_LON_EN_MASK	    	(0x1 << 7)


#define MAX86178_BIOZ_LOFF_THRESH_REG  0xB1
#define MAX86178_BIOZ_LOFF_THRESH_MASK       (0x0F << 0)
#define MAX86178_BIOZ_RESP_CG_MAG_MASK       (0x07 << 4)
#define MAX86178_BIOZ_RESP_CG_MAX_4X_MASK	 (0x01 << 7)

#define MAX86178_BIOZ_RESP_CG_MAG_0    (0x00)
#define MAX86178_BIOZ_RESP_CG_MAG_32   (0x01)
#define MAX86178_BIOZ_RESP_CG_MAG_64   (0x02)
#define MAX86178_BIOZ_RESP_CG_MAG_128  (0x03)
#define MAX86178_BIOZ_RESP_CG_MAG_192  (0x04)
#define MAX86178_BIOZ_RESP_CG_MAG_256  (0x05)
#define MAX86178_BIOZ_RESP_CG_MAG_320  (0x06)
#define MAX86178_BIOZ_RESP_CG_MAG_384  (0x07)



#define MAX86178_BIOZ_LEAD_BIAS_CFG1_REG  0xB4
#define MAX86178_BIOZ_RBIAS_N_EN_MASK		(0x01 << 0)
#define MAX86178_BIOZ_RBIAS_P_EN_MASK		(0x01 << 1)
#define MAX86178_BIOZ_RBIAS_VALUE_MASK		(0x03 << 2)



#define MAX86178_RESPIRATION_CFG1_REG  0xB6
#define MAX86178_RESP_ENABLE_MASK       (0x01 << 0)
#define MAX86178_RESP_CG_MODE_MASK      (0x03 << 1)
#define MAX86178_RESP_CG_CHOP_CLK_MASK  (0x03 << 3)
#define MAX86178_RESP_CG_LPF_DUTY_MASK  (0x07 << 5)


#define MAX86178_RESP_CG_LPF_DUTY_0_DOT_98	(0x00)
#define MAX86178_RESP_CG_LPF_DUTY_1_DOT_95	(0x01)
#define MAX86178_RESP_CG_LPF_DUTY_3_DOT_90	(0x02)
#define MAX86178_RESP_CG_LPF_DUTY_7_DOT_79	(0x03)
#define MAX86178_RESP_CG_LPF_DUTY_15_DOT_54	(0x04)
#define MAX86178_RESP_CG_LPF_DUTY_30_DOT_89	(0x05)
#define MAX86178_RESP_CG_LPF_DUTY_61_DOT_08	(0x06)



/* Lead Detect Confiuration registers */


/* old do not have in new chip
#define MAX86178_AC_LEAD_DETECT_WAVE_REG	0x95
#define MAX86178_DC_LEAD_DETECT_DAC_REG		0x96
#define MAX86178_DC_LEAD_OFF_THR_REG		0x97
#define MAX86178_AC_LEAD_OFF_THR_1_REG		0x98
#define MAX86178_AC_LEAD_OFF_THR_2_REG		0x99
#define MAX86178_AC_LEAD_OFF_PGA_HPF_REG	0x9A
#define MAX86178_AC_LEAD_OFF_CAL_RES_REG	0x9B
#define MAX86178_LEAD_BIAS_CFG_1_REG		0x9E
#define MAX86178_UTIL_ADC_CFG_1_REG			0xE0
#define MAX86178_UTIL_ADC_DATA_HIGH_REG		0xE1
#define MAX86178_UTIL_ADC_DATA_LOW_REG		0xE2
*/


/* Interrupts enable registers */
#define MAX86178_INT1_ENABLE1_REG		0xC0
#define MAX86178_THRESH1_HILO_EN_MASK	(0x01 << 1)
#define MAX86178_THRESH2_HILO_EN_MASK	(0x01 << 2)
#define MAX86178_EXP_OVF_EN_MASK		(0x01 << 3)
#define MAX86178_ALC_OVF_EN_MASK		(0x01 << 4)
#define MAX86178_FIFO_DATA_RDY_EN_MASK	(0x01 << 5)
#define MAX86178_FRAME_RDY_EN_MASK		(0x01 << 6)
#define MAX86178_A_FULL_EN_MASK			(0x01 << 7)






/*appication related definitions*/

#define MAX86178_MAX_PPG_VALUE				(524287)

/* Rev and Par Id registers */
#define MAX86178_REV_ID_REG					0xFE
#define MAX86178_PART_ID_REG				0xFF

#define MAX86178_DATA_WORD_SIZE		3
#define MAX86178_MAX_FIFO_DEPTH		256
#define MAX86178_DATA_TYPE_MASK		(0xF << 19)
#define MAX86178_FIFO_REM			(MAX86178_MAX_FIFO_DEPTH - (25*3))
#define MAX86178_DRIVER_FIFO_SZ		(MAX86178_MAX_FIFO_DEPTH * 8)

#define MAX86140_PART_ID_VAL	0x24 // Single PD with SPI serial Interface
#define MAX86141_PART_ID_VAL	0x25 // Dual PD with SPI serial Interface
#define MAX86142_PART_ID_VAL	0x26 // Single PD with I2C serial Interface
#define MAX86143_PART_ID_VAL	0x27 // Dual PD with I2C serial Interface
#define MAX86161_PART_ID_VAL	0x36 // green/IR/red (LED1/2/3) module; single PD with I2C serial interface
#define MAX86171_PART_ID_VAL	0x2C // MAX86171
#define MAX86178_PART_ID_VAL	0x43 // MAX86178

#define OS64L_I2C_ADDRESS 0x66//(0x64)
#define OS64H_I2C_ADDRESS 0x67//(0x65)


#define PWR_ON		true
#define PWR_OFF		false

/* Configuration */
#define NUM_SAMPLES_PER_CHANNEL 	2	// 2 PD

#define MAX86178_LED_NUM   			9 	//for OS61

//#define MAX86178_FIFO_AFULL   (MAX86178_MAX_FIFO_DEPTH - (MAX86178_LED_NUM * 25))     //DO NOT EXCEED 256 !!!!!
#define MAX86178_FIFO_AFULL	 (MAX86178_MAX_FIFO_DEPTH - 6)

#define THREE_MEASEREMENT_MASK  ((1 << DATA_TYPE_MEAS1) | \
						 	 	(1 << DATA_TYPE_MEAS2) | \
						 	 	(1 << DATA_TYPE_MEAS3))


#define ALL_MEASEREMENT_MASK   ((1 << DATA_TYPE_MEAS1) | \
						 	 	(1 << DATA_TYPE_MEAS2) | \
						 	 	(1 << DATA_TYPE_MEAS3) | \
								(1 << DATA_TYPE_MEAS4) | \
								(1 << DATA_TYPE_MEAS5) | \
								(1 << DATA_TYPE_MEAS6) | \
								(1 << DATA_TYPE_MEAS7) | \
								(1 << DATA_TYPE_MEAS8) | \
								(1 << DATA_TYPE_MEAS9))

#define MAX86178_MEAS_TINT_POS				(3)
#define MAX86178_MEAS_AVG_POS				(0)

#define MAX86178_MEAS_PPG1_DAC_OFF_POS		(0)
#define MAX86178_MEAS_PPG2_DAC_OFF_POS		(4)

typedef enum {
	MAX86178_PPG1 = 0,
	MAX86178_PPG2,
}max86178_ppg_num_t;

/* OS64's LEDS */
typedef enum {
	MAX86178_LED_GREEN = 0u,
	MAX86178_LED_RED,
	MAX86178_LED_IR,

	MAX86178_LED_MAX
} max86178_leds_t;

typedef enum max86178_channels {
	DATA_TYPE_MEAS1   = 0x00,
	DATA_TYPE_MEAS2   = 0x01,
	DATA_TYPE_MEAS3   = 0x02,
	DATA_TYPE_MEAS4   = 0x03,
	DATA_TYPE_MEAS5   = 0x04,
	DATA_TYPE_MEAS6   = 0x05,
	DATA_TYPE_DARK    = 0x06,
	DATA_TYPE_ALC_OVF = 0x07,
	DATA_TYPE_EXP_OVF = 0x08,
	DATA_TYPE_PF      = 0x09,
	DATA_TYPE_INVALID = 0x0F,
	//DATA_TYPE_RESERVED = 0x0F,
} max86178_channels_t;


typedef union {
	struct {
		uint32_t val:20;
		uint32_t type:4;
		uint32_t:8;
	};
	uint32_t raw;
} fifo_data_t;

typedef union {
	uint16_t val;
	struct {
		uint8_t tint;
		uint8_t frac:4;
		uint8_t:4;
	};
} die_temp_t;

union int_status {
	struct {
		struct {
			unsigned char pwr_rdy:1;
			unsigned char thresh1_hilo:1; 
			unsigned char thresh2_hilo:1; 
			unsigned char exp_ovf:1; 
			unsigned char alc_ovf:1; 
			unsigned char fifo_data_rdy:1; 
			unsigned char frame_rdy:1; 
			unsigned char a_full:1;
		};
		struct {
			unsigned char led1_compb:1;
			unsigned char led2_compb:1; 
			unsigned char led3_compb:1; 
			unsigned char led4_compb:1; 
			unsigned char led5_compb:1; 
			unsigned char led6_compb:1; 
			unsigned char reserved_1:1;
			unsigned char invalid_cfg:1;
		};

		struct {
			unsigned char reserved_2:1;
			unsigned char phase_lock:1;
			unsigned char phase_unlock:1;
			unsigned char freq_lock:1;
			unsigned char freq_unlock:1;
			unsigned char reserved_3:2;
			unsigned char timing_sys_reset:1;
		};

	};
	uint8_t val[3];
};


struct max86178_dev {
	//dev_comm_t *comm;
    const struct spi_dt_spec *spi_dev;
	//uint8_t *data;
	//queue_t queue;
    int regulator_state;
    int curr_state;
    int int_gpio;
    int vdd_oor_cnt;
    int irq;
	die_temp_t die_temp;
	uint8_t part_id;
	uint8_t rev_id;
	uint8_t num_pd;
	union {
		struct {
			/*
				Choose what settings will be used; user settings/firmware settings.
				Deault value is 1, which is firmware default
			*/
			uint8_t firmware_default:1;
			/*
				If DAC Calibration procedure will be run before settings register.
				Default value is 1, which is enabled.
			*/
			uint8_t dac_calib:1;
			uint8_t rfu:6;
		};
		uint8_t val;
	} operating_cfg;

	union {
		struct {
			uint8_t ppg_tint:2;
			uint8_t ppg1_adc_rge:2;
			uint8_t ppg2_adc_rge:2;
			uint8_t add_offset:1;
			uint8_t alc_dis:1;
		};
		uint8_t val;
	} ppg_cfg1;

	uint8_t dac_calib_status;
};

typedef struct max86178_dev max86178_dev_t;



typedef enum {
	MAX86178_MEAS_CH1 = 0,
	MAX86178_MEAS_CH2,
	MAX86178_MEAS_CH3,
	MAX86178_MEAS_CH4,
	MAX86178_MEAS_CH5,
	MAX86178_MEAS_CH6,
	/*MAX86178_MEAS_CH7,
	MAX86178_MEAS_CH8,
	MAX86178_MEAS_CH9,*/
	MAX86178_MEAS_CHMAX
} max86178_meas_ch_t;

typedef enum {
	MAX86178_LED_DRIVER_A = 0,
	MAX86178_LED_DRIVER_B,
	MAX86178_LED_DRIVER_MAX
} max86178_led_type;
/************************************************************************************
 *                                                                                  *
 *                              BIOZ API                                            *
 *                                                                                  *
 ************************************************************************************/
typedef enum _max86178_bioz_meas_t {
    MAX86178_BIOZ_MEAS_DISABLE = 0,
    MAX86178_BIOZ_MEAS_I,
    MAX86178_BIOZ_MEAS_Q,
    MAX86178_BIOZ_MEAS_INVALID,
} max86178_bioz_meas_t;

typedef enum _max86178_drv_mode_t {
    MAX86178_DRV_MODE_CURRENT = 0, //default
    MAX86178_DRV_MODE_VOLTAGE,
    MAX86178_DRV_MODE_HBRIDGE,
    MAX86178_DRV_MODE_STANDBY,
    MAX86178_DRV_MODE_INVALID
} max86178_drv_mode_t;

typedef enum _max86178_vdrv_mag_t {
    MAX86178_VDRV_MAG_35_4 = 0, //default
    MAX86178_VDRV_MAG_70_7,
    MAX86178_VDRV_MAG_177,
    MAX86178_VDRV_MAG_354,
    MAX86178_VDRV_MAG_INVALID
} max86178_vdrv_mag_t;

typedef enum _max86178_amp_bw_t {
    MAX86178_AMP_BW_LOW = 0, //default
    MAX86178_AMP_BW_MID_LOW,
    MAX86178_AMP_BW_MID_HIGH,
    MAX86178_AMP_BW_HIGH,
    MAX86178_AMP_BW_INVALID,
} max86178_amp_bw_t;

typedef enum _max86178_amp_rge_t {
    MAX86178_AMP_RGE_LOW = 0, //default
    MAX86178_AMP_RGE_MID_LOW,
    MAX86178_AMP_RGE_MID_HIGH,
    MAX86178_AMP_RGE_HIGH,
    MAX86178_AMP_RGE_INVALID,
} max86178_amp_rge_t;

typedef enum _max86178_rld_drv_t {
    MAX86178_RLD_DRV_VMID_TX = 0, //default
    MAX86178_RLD_DRV_VRLD,
    MAX86178_RLD_DRV_INVALID
} max86178_rld_drv_t;

typedef enum _max86178_resp_mode_t {
    MAX86178_RESP_MODE_LPF = 0,
    MAX86178_RESP_MODE_DYN,
    MAX86178_RESP_MODE_DYN_LPF,
    MAX86178_RESP_MODE_DYN_RES,
    MAX86178_RESP_MODE_INVALID
} max86178_resp_mode_t;

typedef enum _max86178_lpf_bw_t {
    MAX86178_LPF_BW_0_98Hz = 0, //default
    MAX86178_LPF_BW_1_95Hz,
    MAX86178_LPF_BW_3_9HZ,
    MAX86178_LPF_BW_7_79Hz,
    MAX86178_LPF_BW_15_54Hz,
    MAX86178_LPF_BW_30_89Hz,
    MAX86178_LPF_BW_61_08Hz,
    MAX86178_LPF_BW_NA,
    MAX86178_LPF_BW_INVALID
} max86178_lpf_bw_t;

typedef enum _max86178_chop_clk_t {
    MAX86178_CHOP_CLK_256 = 0, //default
    MAX86178_CHOP_CLK_512,
    MAX86178_CHOP_CLK_1024,
    MAX86178_CHOP_CLK_2048,
    MAX86178_CHOP_CLK_INVALID
} max86178_chop_clk_t;

typedef enum _max86178_drv_current_t {
    MAX86178_DRV_CURR_0 = 0, //default
    MAX86178_DRV_CURR_8,
    MAX86178_DRV_CURR_16,
    MAX86178_DRV_CURR_32,
    MAX86178_DRV_CURR_48,
    MAX86178_DRV_CURR_64,
    MAX86178_DRV_CURR_80,
    MAX86178_DRV_CURR_96,
    MAX86178_DRV_CURR_INVALID
} max86178_drv_current_t;

typedef enum _max86178_ina_mode_t {
    MAX86178_INA_MODE_LOW_NOISE = 0, //default
    MAX86178_INA_MODE_LOW_POWER,
    MAX86178_INA_MODE_INVALID
} max86178_ina_mode_t;

typedef enum _max86178_ahpf_t {
    MAX86178_AHPF_100Hz = 0,
    MAX86178_AHPF_200Hz,
    MAX86178_AHPF_1kHz,
    MAX86178_AHPF_2kHz,
    MAX86178_AHPF_5kHz,
    MAX86178_AHPF_10kHz,
    MAX86178_AHPF_BYPASS,
    MAX86178_AHPF_2Hz,
    MAX86178_AHPF_4Hz,
    MAX86178_AHPF_10Hz,
    MAX86178_AHPF_19Hz,
    MAX86178_AHPF_36Hz,
    MAX86178_AHPF_94Hz,
    MAX86178_AHPF_INVALID
} max86178_ahpf_t;

typedef enum _max86178_ipol_t {
    MAX86178_IPOL_NON_INVERTED = 0,
    MAX86178_IPOL_INVERTED,
    MAX86178_IPOL_INVALID
} max86178_ipol_t;

typedef enum _max86178_dhpf_t {
    MAX86178_DHPF_BYPASS = 0,
    MAX86178_DHPF_SRx00025,
    MAX86178_DHPF_SRx002,
    MAX86178_DHPF_INVALID
} max86178_dhpf_t;

typedef enum _max86178_dlpf_t {
    MAX86178_DLPF_BYPASS = 0,
    MAX86178_DLPF_SRx005,
    MAX86178_DLPF_SRx02,
    MAX86178_DLPF_SRx08,
    MAX86178_DLPF_SRx25,
    MAX86178_DLPF_INVALID,
} max86178_dlpf_t;

typedef enum _max86178_gain_t {
    MAX86178_GAIN_1 = 0,
    MAX86178_GAIN_2,
    MAX86178_GAIN_5,
    MAX86178_GAIN_10,
    MAX86178_GAIN_INVALID
} max86178_gain_t;

typedef enum _max86178_dac_osr_t {
    MAX86178_DAC_OSR_32 = 0,
    MAX86178_DAC_OSR_64,
    MAX86178_DAC_OSR_128,
    MAX86178_DAC_OSR_256,
    MAX86178_DAC_OSR_INVALID
} max86178_dac_osr_t;

typedef enum _max86178_adc_osr_t {
    MAX86178_ADC_OSR_8 = 0,
    MAX86178_ADC_OSR_16,
    MAX86178_ADC_OSR_32,
    MAX86178_ADC_OSR_64,
    MAX86178_ADC_OSR_128,
    MAX86178_ADC_OSR_256,
    MAX86178_ADC_OSR_512,
    MAX86178_ADC_OSR_1024,
    MAX86178_ADC_OSR_INVALID
} max86178_adc_osr_t;

typedef int (*max86178_fifo_read_cb)(void);

void *max86178_get_device_data(void);
int max86178_init(const struct spi_dt_spec *spi_dev);
int max86178_startup_init(struct max86178_dev *sd);
int max86178_write_reg(const struct spi_dt_spec *spi_dev, uint8_t reg_addr,uint8_t *data, uint32_t len);
int max86178_read_reg(const struct spi_dt_spec *spi_dev, uint8_t *data, uint16_t len);
int max86178_block_write(const struct spi_dt_spec *spi_dev,  struct regmap reg_block[], int size);
int max86178_dump_regs(const struct spi_dt_spec *spi_dev, uint8_t *buf, uint8_t start_addr, uint8_t end_addr);


int max86178_sensor_enable(struct max86178_dev *sd, int agc_enable, int enable);
int max86178_is_ecg_enabled(uint8_t *p_ecg_enabled);
int max86178_init_fifo(struct max86178_dev *sd, uint8_t a_full_val) ;
int max86178_get_meas_num(struct max86178_dev *sd);
int max86178_get_part_info(const struct spi_dt_spec *spi_dev, uint8_t *part_id, uint8_t *rev_id, uint8_t *num_pd);
int max86178_set_sample_rate(struct max86178_dev *sd, uint16_t rate);
int max86178_get_sample_rate(struct max86178_dev *sd);
int max86178_fifo_irq_handler(struct max86178_dev *sd);
int max86178_irq_handler(void *arg);
int max86178_get_irq_state(void *data);
void max86178_irq_clr();
void max86178_irq_clr_to_zero(struct max86178_dev *sd);
void max86178_irq_reset_ref_cnt();
void max86178_irq_handler_fast(const struct device *dev, struct gpio_callback *cb, uint32_t pins);

int max86178_get_meas_num(struct max86178_dev *sd);

int spi_rw_test(const struct spi_dt_spec *spi_dev);


// sensor on off reset
int max86178_reset (struct max86178_dev *sd);
int max86178_poweron (struct max86178_dev *sd);
int max86178_poweroff (struct max86178_dev *sd);

int max86178_get_num_of_channel(uint8_t * p_num_ch);
int max86178_is_ppg_enabled(uint8_t * p_ppg_enabled);
int max86178_is_ecg_enabled(uint8_t *p_ecg_enabled);
int max86178_is_iq_enabled(uint8_t * p_iq_enabled);
int max86178_is_acc_enabled(uint8_t * p_acc_enabled);


int max86178_is_enabled(uint8_t * p_is_enabled);
int max86178_set_frame_ready_int(uint8_t enable);
int max86178_set_a_full_int(uint8_t enable);
int max86178_set_fifo_a_full(uint8_t level);

int max86178_is_ecg_faster_than_ppg(uint8_t *p_is_true);
int max86178_get_num_photo_diodes(uint8_t *p_num_diode);

void max86178_register_fifo_read_callback(max86178_fifo_read_cb func);
void max86178_unregister_fifo_read_callback();

int max86178_clear_fifo();

int max86178_bioz_enable(max86178_bioz_meas_t meas);


#endif