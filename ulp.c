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
unsigned t3 = 0; // hours X10
unsigned t2 = 0; // hours  X1
unsigned t1 = 0; // mins  X10
unsigned t0 = 0; // mins   X1
unsigned counter = 0; // seconds X8

// helpers
unsigned lcdA = 0; // LCD Content A
unsigned lcdB = 0; // LCD Content B
unsigned dA = 0;   // LCD Content with polarity
unsigned dB = 0;   // LCD Content with polarity
unsigned i = 0;    // temp counter
unsigned d = 0;    // temp

unsigned DIGIT0A[10] = { 3, 0, 5, 5, 6, 7, 7, 1, 7, 7 };
unsigned DIGIT0B[10] = { 15, 3, 13, 7, 3, 6, 14, 3, 15, 7 };
unsigned DIGIT1A[10] = { 56, 8, 88, 88, 104, 112, 112, 24, 120, 120 };
unsigned DIGIT1B[10] = { 224, 32, 192, 96, 32, 96, 224, 32, 224, 96 };
unsigned DIGIT2A[10] = { 1792, 256, 2816, 2816, 3328, 3584, 3584, 768, 3840, 3840 };
unsigned DIGIT2B[10] = { 3584, 512, 3072, 1536, 512, 1536, 3584, 512, 3584, 1536 };
unsigned DIGIT3A[10] = { 28672, 4096, 45056, 45056, 53248, 57344, 57344, 12288, 61440, 61440 };
unsigned DIGIT3B[10] = { 28672, 4096, 24576, 12288, 4096, 12288, 28672, 4096, 28672, 12288 };

void entry() {
  // CALC TIME
  counter++;
  if (counter >= 480) {
    counter = 0;
    t0++;
  }
  if (t0 >= 10) {
    t0 = 0;
    t1++;
  }
  if (t1 >= 6) {
    t1 = 0;
    t2++;
  }
  if (t2 >= 10) {
    t2 = 0;
    t3++;
  }
  if (t3 >= 1 && t2 >= 2) {
    t3 = 0;
    t2 = 0;
  }
  
  // UPDATE LCD
  lcdA = DIGIT0A[t0] | DIGIT1A[t1] | DIGIT2A[t2];
  lcdB = DIGIT0B[t0] | DIGIT1B[t1] | DIGIT2B[t2];

  if (t3) {
    lcdA |= DIGIT3A[t3];
    lcdB |= DIGIT3B[t3];
  }

  if (counter & 4) {
    lcdA |= 1<<7;
  }

  // UPDATE IO
  gpio_data_unlock();
  gpio_clock_unlock();
  gpio_latch_unlock();
  gpio_lcd_com_unlock();

  short_delay();
  
  if (counter & 1) {
    dA = lcdA;
    dB = lcdB;
  } else {
    dA = ~lcdA;
    dB = ~lcdB;
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

  if (counter & 1) {
    gpio_lcd_com_low();
  } else {
    gpio_lcd_com_high();
  }

  gpio_data_lock();
  gpio_clock_lock();
  gpio_latch_lock();
  gpio_lcd_com_lock();

  // wake_when_ready();
}

#endif // do not add anything after here
