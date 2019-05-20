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
#define  PWM_RC_CH5_INPUT_PIN 8
#define  PWM_RC_CH6_INPUT_PIN 10
#define  PWM_SPEEDOMETER_INPUT_PIN 12

// PWM Output
// PIN used to connect Actuators
#define  PWM_OUTPUT_FREQ  50
#define  PWM_OUTPUT_DUTY_RESOLUTION LEDC_TIMER_13_BIT 
#define  PWM_OUTPUT_STEP ((1000000000/PWM_OUTPUT_FREQ)/((2^LEDC_TIMER_13_BIT)-1)) /* ns */

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

#define PWM_RC_THROTTLE_INPUT_PIN   23   //Set GPIO 25 as  CAP1
#define PWM_RC_STEERING_INPUT_PIN   25   //Set GPIO 23 as  CAP0

#define CAP0_INT_EN BIT(27)  //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28)  //Capture 1 interrupt bit
#define CAP_SIG_NUM 2

typedef struct {
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

xQueueHandle cap_queue;

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

static void mcpwm_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PWM_RC_THROTTLE_OUTUT_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PWM_RC_STEERING_OUTUT_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, PWM_RC_STEERING_INPUT_PIN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, PWM_RC_THROTTLE_INPUT_PIN);
    gpio_pulldown_en(PWM_RC_STEERING_INPUT_PIN);    //Enable pull down on CAP0   signal
    gpio_pulldown_en(PWM_RC_THROTTLE_INPUT_PIN);    //Enable pull down on CAP1   signal
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

static void disp_captured_signal(void *arg)
{
    uint32_t *current_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    uint32_t *previous_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    capture evt;
    while (1) {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0) {
            current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP0 : %d us\n", current_cap_value[0]);
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1) {
            current_cap_value[1] = evt.capture_signal - previous_cap_value[1];
            previous_cap_value[1] = evt.capture_signal;
            current_cap_value[1] = (current_cap_value[1] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            printf("CAP1 : %d us\n", current_cap_value[1]);
        }
    }
}

static void IRAM_ATTR isr_handler()
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    if (mcpwm_intr_status & CAP0_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    if (mcpwm_intr_status & CAP1_INT_EN) { //Check for interrupt on rising edge on CAP0 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
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

    //7. Capture configuration
    //comment if you don't want to use capture submodule, also u can comment the capture gpio signals
    //configure CAP0 and CAP1 signal to start capture counter on rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0);  //capture signal on rising edge, prescale = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = CAP0_INT_EN | CAP1_INT_EN;  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);  //Set ISR Handler

}

void app_main()
{
  int tick=0;
  uart_set_baudrate(UART_NUM_0, 2000000); 
  ws2812_control_init();
  mcpwm_init_control();
  cap_queue = xQueueCreate(1, sizeof(capture)); //comment if you don't want to use capture module
  xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 5, NULL);  //comment if you don't want to use capture module

  // Set configuration of timer0 for high speed channels
  //
  while(1) {
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    printf("tick\n");
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
