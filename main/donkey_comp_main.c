/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "esp_system.h"
#include "math.h"
#include "esp_spi_flash.h"

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
#define  PWM_RC_STEERING_INPUT_PIN 5
#define  PWM_RC_THROTTLE_INPUT_PIN 7
#define  PWM_RC_CH5_INPUT_PIN 8
#define  PWM_RC_CH6_INPUT_PIN 10
#define  PWM_SPEEDOMETER_INPUT_PIN 12

// PWM Output
// PIN used to connect Actuators
#define  PWM_RC_THROTTLE_OUTUT_PIN 18
#define  PWM_RC_STEERING_OUTUT_PIN 19
#define  PWM_OUTPUT_FREQ  50
#define  PWM_OUTPUT_DUTY_RESOLUTION LEDC_TIMER_13_BIT 
#define  PWM_OUTPUT_STEP (((1/PWM_OUTPUT_FREQ)*1000000000)/((2^LEDC_TIMER_13_BIT)-1)) /* ns */

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

#define GPIO_PWM0A_OUT 15   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B

#define GPIO_PWM1A_OUT 17   //Set GPIO 15 as PWM1A
#define GPIO_PWM1B_OUT 18   //Set GPIO 16 as PWM1B

static void mcpwm_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void mcpwm_set_throttle_pwm(int pwm_width)
{
  mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, pwm_width*1000);
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void mcpwm_set_steering_pwm(int pwm_width)
{
  mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, pwm_width*1000);
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
    pwm_config.frequency = 100;    //frequency = 500Hz,
    pwm_config.cmpr_a = (2000/1500);    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);    //Configure PWM1A & PWM1B with above settings
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
}


/* PWM based on ledc driver*/

const ledc_timer_config_t ledc_timer = {
  .duty_resolution = PWM_OUTPUT_DUTY_RESOLUTION, // resolution of PWM duty
  .freq_hz = 50,                      // frequency of PWM signal
  .speed_mode = LEDC_HIGH_SPEED_MODE,           // timer mode
  .timer_num = LEDC_TIMER_0            // timer index
};

ledc_channel_config_t ledc_throttle_channel = {
  .channel    = LEDC_CHANNEL_0,
  .duty       = 7,
  .gpio_num   = PWM_RC_THROTTLE_OUTUT_PIN,
  .speed_mode = LEDC_HIGH_SPEED_MODE,
  .hpoint     = 0,
  .timer_sel  = LEDC_TIMER_0
};

ledc_channel_config_t ledc_steering_channel = {
  .channel    = LEDC_CHANNEL_1,
  .duty       = 7,
  .gpio_num   = PWM_RC_STEERING_OUTUT_PIN,
  .speed_mode = LEDC_HIGH_SPEED_MODE,
  .hpoint     = 0,
  .timer_sel  = LEDC_TIMER_0
};

void set_pwm_length (ledc_channel_config_t * config, int length)
{
    config->duty = round(((length*1000*1000)/PWM_OUTPUT_STEP));
}

void app_main()
{
  ws2812_control_init();
  mcpwm_init_control();
  // Set configuration of timer0 for high speed channels
  ledc_timer_config(&ledc_timer);
  set_pwm_length (&ledc_throttle_channel, 1500);
  set_pwm_length (&ledc_steering_channel, 1500);
  ledc_channel_config(&ledc_throttle_channel);
  ledc_channel_config(&ledc_steering_channel);
}
