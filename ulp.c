#ifdef _ULPCC_ // Do not add anything above this def

#include <ulp_c.h>
#include <soc_ulp_c.h>

// Find reg names:
// https://github.com/espressif/esp-idf/blob/master/components/soc/esp32/include/soc/rtc_io_reg.h

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

unsigned count = 0;

void entry() {
  gpio_latch_unlock();
  gpio_latch_high();
  wait(10000);
  gpio_latch_low();
  gpio_latch_lock();

  count++;
  wake_when_ready();
}

#endif // do not add anything after here
