/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "driver/uart.h"
#include "driver/rmt.h"

#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "esp_system.h"
#include "math.h"
#include "esp_spi_flash.h"
#include "esp_timer.h"

// PIN used to drive NeoPixel LEDs
#define LED_PIN         6
#define NUM_LEDS        1
#include "ws2812_control.h"

#define RED             0xFF0000
#define GREEN           0x00FF00
#define BLUE            0x0000FF

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      1
#define TIMESTEPS      8 

// Statuses
#define INT_DISCONNECTED  0
#define INT_RXERROR       1
#define INT_CALIBRATE     2
#define HOST_INIT         3
#define HOST_MODE_USER    4
#define HOST_MODE_LOCAL   5
#define HOST_MODE_DISARMED   6
 
// PIN used to connect Rx receiver
#define  PWM_RC_CH5_INPUT_PIN 8
#define  PWM_RC_CH6_INPUT_PIN 10
#define  PWM_SPEEDOMETER_INPUT_PIN 12

// PWM Output
// PIN used to connect Actuators
#define  PWM_OUTPUT_FREQ  50
#define  PWM_OUTPUT_DUTY_RESOLUTION LEDC_TIMER_13_BIT 
#define  PWM_OUTPUT_STEP ((1000000000/PWM_OUTPUT_FREQ)/((2^LEDC_TIMER_13_BIT)-1)) /* ns */
#define  RMT_TICK_10_US    (80000000/RMT_CLK_DIV/100000)   /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define  rmt_item32_tIMEOUT_US  9500 

//Each 50ms, check and output value to serial link
#define OUTPUTLOOP 50000

// Global var used to capture Rx signal
unsigned int pwm_steering_value = 0;
unsigned int pwm_throttle_value = 0;
unsigned int pwm_ch5_value = 0;
unsigned int pwm_ch6_value = 0;
unsigned int pwm_speedometer_value = 0;
unsigned int freq_value = 0;
unsigned int prev_steering_time = 0;
unsigned int prev_throttle_time = 0;
unsigned int prev_ch5_time = 0;
unsigned int prev_ch6_time = 0;
unsigned int prev_speedometer_time = 0;
unsigned int prev_freq_time = 0;

// Gloval var used to detect signal activity
int steering_toggle = 0;
int throttle_toggle = 0;
int ch5_toggle = 0;
int ch6_toggle = 0;
int speedometer_toggle = 0;

// GLobal buffer for serial output
char buff [50] = {};

/*PWM based on */

#define PWM_RC_THROTTLE_OUTUT_PIN 16   //Set GPIO 15 as PWM0A
#define PWM_RC_STEERING_OUTUT_PIN 17   //Set GPIO 15 as PWM1A

#define RMT_RX_CHANNEL    0     /*!< RMT channel for receiver */
#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */
#define PWM_RC_THROTTLE_INPUT_PIN   23   //Set GPIO 25 as  CAP1
#define PWM_RC_STEERING_INPUT_PIN   25   //Set GPIO 23 as  CAP0

#define GPIO_INPUT_PIN_SEL  ((1ULL<<PWM_RC_STEERING_INPUT_PIN) | (1ULL<<PWM_RC_THROTTLE_INPUT_PIN))


static void mcpwm_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_RC_THROTTLE_OUTUT_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_RC_STEERING_OUTUT_PIN);
  }

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void mcpwm_set_throttle_pwm(int pwm_width_in_us)
{
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm_width_in_us);
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void mcpwm_set_steering_pwm(int pwm_width_in_us)
{
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, pwm_width_in_us);
}

/**
 * @brief Configure MCPWM module for brushed dc motor
 */
static void mcpwm_init_control()
{
    //1. mcpwm gpio initialization
    mcpwm_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 500Hz,
    pwm_config.cmpr_a = (float)((1500.0*100.0)/20000.0);    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}

#define ESP_INTR_FLAG_DEFAULT 0

typedef struct {
  uint32_t t0;
  uint32_t t1;
} SIGNAL_TIMING;

SIGNAL_TIMING pwm_timing[34];
uint32_t pwm_length[34];

static void IRAM_ATTR gpio_isr_handler(void* arg)
{    
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t t = esp_timer_get_time();
    if (gpio_get_level(gpio_num) == 1) {
      // rising edge
      pwm_timing[gpio_num].t0 = t;
    } else {
      //falling edge
      pwm_timing[gpio_num].t1 = t;
      if (t >= pwm_timing[gpio_num].t0)
      pwm_length[gpio_num] = pwm_timing[gpio_num].t1 - pwm_timing[gpio_num].t0;
    }
}

void init_rx_gpio (void)
{
     gpio_config_t io_conf;
     //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PWM_RC_THROTTLE_INPUT_PIN, gpio_isr_handler, (void*) PWM_RC_THROTTLE_INPUT_PIN);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(PWM_RC_STEERING_INPUT_PIN, gpio_isr_handler, (void*) PWM_RC_STEERING_INPUT_PIN);



}
void app_main()
{
  int tick=0;
  uart_set_baudrate(UART_NUM_0, 2000000); 
  ws2812_control_init();
  mcpwm_init_control();
  init_rx_gpio();
  // Set configuration of timer0 for high speed channels
  //
  while(1) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("tick %d %d %d\n", tick, pwm_length[PWM_RC_THROTTLE_INPUT_PIN], pwm_length[PWM_RC_STEERING_INPUT_PIN]);
    switch (tick%3) {
      case 0:
        mcpwm_set_throttle_pwm(1000);
      break;
      case 1:
        mcpwm_set_throttle_pwm(1500);
      break;
      case 2:
        mcpwm_set_throttle_pwm(2000);
      break;
    };
    tick++;
  };
}
