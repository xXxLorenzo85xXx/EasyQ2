#include <nrfx_spim.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>                                                                                                                                                     
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>
#include "spi_api.h"

#define MY_GPIO1 DT_NODELABEL(gpio1)
#define GPIO_1_CS 7
const struct device *gpio1_dev = DEVICE_DT_GET(MY_GPIO1);

#define SPI1_NODE DT_NODELABEL(max86178)

#define SPIOP SPI_WORD_SET(8)   |SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB |  SPI_LINES_SINGLE

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(SPI1_NODE, SPIOP, 0);

int spi_init(void)
{
  return 0;  
  gpio_pin_configure(gpio1_dev, GPIO_1_CS, GPIO_OUTPUT);
  gpio_pin_set(gpio1_dev, GPIO_1_CS,1);
  if (!device_is_ready(spispec.bus))
   {
      printk("spi not ready");
      return -ENODEV;
   }
  printk("spi ready"); 
}   