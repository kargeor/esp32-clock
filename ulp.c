#ifdef _ULPCC_ // Do not add anything above this def

#include <ulp_c.h>
#include <soc_ulp_c.h>

// Find reg names:
// https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/rtc_io_reg.h
// https://github.com/espressif/esp-idf/blob/master/components/soc/soc/esp32/include/soc/rtc_io_reg.h

// MUST CHANGE "CONFIG_ULP_COPROC_RESERVE_MEM"
// in Library/Arduino15/packages/esp32/hardware/esp32/1.0.4/tools/sdk/include/config/sdkconfig.h

//
#define gpio_data_unlock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD2_REG, RTC_IO_TOUCH_PAD2_HOLD_S, 1, 0)
#define gpio_data_high() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + 12, 1, 1)
#define gpio_data_low() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 12, 1, 1)
#define gpio_data_lock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD2_REG,RTC_IO_TOUCH_PAD2_HOLD_S, 1, 1)
//
#define gpio_clock_unlock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD4_REG, RTC_IO_TOUCH_PAD4_HOLD_S, 1, 0)
#define gpio_clock_high() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + 14, 1, 1)
#define gpio_clock_low() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 14, 1, 1)
#define gpio_clock_lock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD4_REG,RTC_IO_TOUCH_PAD4_HOLD_S, 1, 1)
//
#define gpio_latch_unlock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD7_REG, RTC_IO_TOUCH_PAD7_HOLD_S, 1, 0)
#define gpio_latch_high() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + 17, 1, 1)
#define gpio_latch_low() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 17, 1, 1)
#define gpio_latch_lock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD7_REG,RTC_IO_TOUCH_PAD7_HOLD_S, 1, 1)
//
#define gpio_lcd_com_unlock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD0_REG, RTC_IO_TOUCH_PAD0_HOLD_S, 1, 0)
#define gpio_lcd_com_high() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TS_REG, RTC_GPIO_OUT_DATA_W1TS_S + 10, 1, 1)
#define gpio_lcd_com_low() \
  WRITE_RTC_REG(RTC_GPIO_OUT_W1TC_REG, RTC_GPIO_OUT_DATA_W1TC_S + 10, 1, 1)
#define gpio_lcd_com_lock() \
  WRITE_RTC_REG(RTC_IO_TOUCH_PAD0_REG,RTC_IO_TOUCH_PAD0_HOLD_S, 1, 1)
//

// this should be around 0.1 ms
#define short_delay() wait( 800 )

// Shared with main
unsigned count = 0;
unsigned LCDA = 0;
unsigned LCDB = 0;

// helpers
unsigned dA = 0;
unsigned dB = 0;
unsigned i = 0;
unsigned d = 0;

void entry() {
  gpio_data_unlock();
  gpio_clock_unlock();
  gpio_latch_unlock();
  gpio_lcd_com_unlock();

  short_delay();
  if (count & 1) {
    gpio_lcd_com_low();
    dA = LCDA;
    dB = LCDB;
  } else {
    gpio_lcd_com_high();
    dA = ~LCDA;
    dB = ~LCDB;
  }
  gpio_latch_low();
  short_delay();

  // output
  i = 0;
  d = dA;
  while (i < 32) {
    if (i == 16) {
      d = dB;
    }

    gpio_clock_low();
    
    if (d & 1) {
      gpio_data_high();
    } else {
      gpio_data_low();
    }

    short_delay();
    
    gpio_clock_high();
    short_delay();

    d = d >> 1;
    i++;
  }

  short_delay();
  gpio_latch_high();

  gpio_data_lock();
  gpio_clock_lock();
  gpio_latch_lock();
  gpio_lcd_com_lock();

  count++;
  if ((count & 3) == 0) {
    wake_when_ready();
  }
}

#endif // do not add anything after here
