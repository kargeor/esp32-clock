#include "esp32/ulp.h"
#include "ulp_main.h"
#include "ulptool.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/soc.h"
#include "driver/rtc_io.h"

/* GPIO:
 RTC        GPIO  FUNC  IN / OUT            USE
------------------------------------------------
RTC_GPIO0  (GPIO36) VP  INPUT ONLY          N/A
RTC_GPIO3  (GPIO39) VN  INPUT ONLY          N/A
RTC_GPIO4  (GPIO34)  -  INPUT ONLY          N/A
RTC_GPIO5  (GPIO35)  -  INPUT ONLY          N/A
RTC_GPIO6  (GPIO25) D1  OK / OK             N/A
RTC_GPIO7  (GPIO26) D2  OK / OK             N/A
RTC_GPIO8  (GPIO33) T8  OK / OK             N/A
RTC_GPIO9  (GPIO32) T9  OK / OK             N/A
RTC_GPIO10 (GPIO4)  T0  OK / OK             LCD-COMMON
RTC_GPIO11 (GPIO0)  T1  PWM ON BOOT         N/A
RTC_GPIO12 (GPIO2)  T2  OK / OK             SHIFTREG-DATA [DS of 74HC595]
RTC_GPIO13 (GPIO15) T3  PWM ON BOOT         N/A
RTC_GPIO14 (GPIO13) T4  OK / OK             SHIFTREG-CLOCK [SH_CP of 74HC595]
RTC_GPIO15 (GPIO12) T5  BOOT FAIL IF HIGH   N/A
RTC_GPIO16 (GPIO14) T6  PWM ON BOOT         N/A
RTC_GPIO17 (GPIO27) T7  OK / OK             SHIFTREG-LATCH [ST_CP of 74HC595]
*/

/*
LCD MAP:
 - DIGIT 0: [ 1, 17, 18, 19, 20,  2,  3]
 - DIGIT 1: [ 5,  4, 22, 23, 24,  6,  7]
 - DIGIT 2: [10,  9, 26, 27, 28, 11, 12]
 - DIGIT 3: [14, 13, 29, 30, 31, 15, 16]
 - TWO DOTS: 8
 - OTHER DOTS: 21, 25, 32

CODE GEN:
digits = [];

rA = [];
rB = [];
[0x3F, 0x06, 0x5B, 0x4F, 0x66,
 0x6D, 0x7D, 0x07, 0x7F, 0x6F].forEach((num,i) => {
  A = 0;
  B = 0;
  ii = 0;
  while (num) {
    if (num & 1) {
      bit = digits[ii] - 1;
      if (bit >= 16) B |= 1 << (bit - 16);
      else A |= 1 << bit;
    }
    ii++;
    num >>= 1;
  }
  rA.push(A);
  rB.push(B);
});
console.log(rA, rB);
*/

gpio_num_t ulp_gpio_data  = GPIO_NUM_2;
gpio_num_t ulp_gpio_clock = GPIO_NUM_13;
gpio_num_t ulp_gpio_latch = GPIO_NUM_27;
gpio_num_t ulp_gpio_lcd_com = GPIO_NUM_4;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static void init_run_ulp(uint32_t usec) {
  // initialize ulp variable
  ulp_set_wakeup_period(0, usec);

  // use this binary loader instead
  ESP_ERROR_CHECK(ulptool_load_binary(
    0,
    ulp_main_bin_start,
    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t)
  ));

  // ulp coprocessor will run on its own now
  ESP_ERROR_CHECK( ulp_run(&ulp_entry - RTC_SLOW_MEM) );
}

void rtc_init_out(gpio_num_t gpionum) {
  rtc_gpio_init(gpionum);
  rtc_gpio_set_direction(gpionum, RTC_GPIO_MODE_OUTPUT_ONLY);
  // rtc_gpio_set_level(ulp_toggle_num, 1);
  // rtc_gpio_hold_en(ulp_toggle_num);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  Serial.println();

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_ULP) {
    Serial.println("ULP wakeup\n");
  } else {
    Serial.println("Initializing ULP\n");
    Serial.print("ULP Entry point: 0x");
    Serial.print((&ulp_entry - RTC_SLOW_MEM), HEX);
    Serial.println();

    rtc_init_out(ulp_gpio_data);
    rtc_init_out(ulp_gpio_clock);
    rtc_init_out(ulp_gpio_latch);
    rtc_init_out(ulp_gpio_lcd_com);

    init_run_ulp(125 * 1000); // 125 msec (8 Hz)
  }
}

void loop() {
  ESP_ERROR_CHECK( esp_sleep_enable_ulp_wakeup() );
  esp_deep_sleep_start();
}
