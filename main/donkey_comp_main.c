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

#include "string.h"

#define MAX_GPIO 40

// PIN used to drive NeoPixel LEDs
#include "ws2812_control.h"

#define GPIO_OUTPUT_LED    16 /*Keep it consistent with ws2812_control.h !!*/
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_LED))// How many NeoPixels are attached to the Arduino?

#define TIMESTEPS      8 

// Statuses
#define INT_DISCONNECTED  0
#define INT_RXERROR       1
#define INT_CALIBRATE     2
#define HOST_INIT         3
#define HOST_MODE_USER    4
#define HOST_MODE_LOCAL   5
#define HOST_MODE_DISARMED   6
 

//Each 50ms, check and output value to serial link
#define OUTPUTLOOP 50
#define PWM_FREQ 125
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

#define PWM_RC_THROTTLE_OUTUT_PIN 32   //Set GPIO 15 as PWM0A
#define PWM_RC_STEERING_OUTUT_PIN 33   //Set GPIO 15 as PWM1A

#define RMT_CLK_DIV      100    /*!< RMT counter clock divider */

// PIN used to connect Rx receiver
#define PWM_RC_THROTTLE_INPUT_PIN   25   
#define PWM_RC_STEERING_INPUT_PIN   26   
#define PWM_RC_CH5_INPUT_PIN        27
#define PWM_RC_CH6_INPUT_PIN        14
#define PWM_SPEEDOMETER_INPUT_PIN   17

#define GPIO_INPUT_PIN_SEL  ((1ULL<<PWM_RC_STEERING_INPUT_PIN) | (1ULL<<PWM_RC_THROTTLE_INPUT_PIN) | (1ULL<<PWM_RC_CH5_INPUT_PIN) | (1ULL<<PWM_RC_CH6_INPUT_PIN) | (1ULL<<PWM_SPEEDOMETER_INPUT_PIN))

//
// PWM Output
//

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
    pwm_config.frequency = PWM_FREQ;    //frequency = 500Hz,
    pwm_config.cmpr_a = (float)((1500.0*100.0)/(1000000/PWM_FREQ));    //duty cycle of PWMxA = 0
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

SIGNAL_TIMING pwm_timing[MAX_GPIO];
uint32_t pwm_length[MAX_GPIO];

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static void IRAM_ATTR gpio_isr_handler(void* arg)
{    
    portENTER_CRITICAL_ISR(&mux);
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
    portEXIT_CRITICAL_ISR(&mux);
}

//
// PWM INPUT
//

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
    gpio_isr_handler_add(PWM_RC_STEERING_INPUT_PIN, gpio_isr_handler, (void*) PWM_RC_STEERING_INPUT_PIN);
    gpio_isr_handler_add(PWM_RC_CH5_INPUT_PIN, gpio_isr_handler, (void*) PWM_RC_CH5_INPUT_PIN);
    gpio_isr_handler_add(PWM_RC_CH6_INPUT_PIN, gpio_isr_handler, (void*) PWM_RC_CH6_INPUT_PIN);
    gpio_isr_handler_add(PWM_SPEEDOMETER_INPUT_PIN, gpio_isr_handler, (void*) PWM_SPEEDOMETER_INPUT_PIN);
}

void init_led_gpio(void)
{
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_LED;
    //disable pull-down mode
    io_conf.pull_down_en = 1;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
// --------------------------------------
// LED Part
// --------------------------------------

#define COMPCOLOR(r, g, b) (r<<16)|(g<<8)|(b)

struct Led {
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char timing;
} ;

struct Led leds[NUM_LEDS];
struct led_state led_new_state;

void switchOffLed() {
  for(int i=0;i<NUM_LEDS;i++){
    led_new_state.leds[0]=COMPCOLOR(0,0,0);
    ws2812_write_leds(led_new_state);
  }    
}

void updateLed(void * pvParameters ) {
  static int seq = 0;

  while(1) {
    for(int i=0;i<NUM_LEDS;i++) {
      if (leds[i].timing>>seq & 0x01) {
        led_new_state.leds[0]=COMPCOLOR(leds[i].r,leds[i].g,leds[i].b);
      } else {
        led_new_state.leds[0]=COMPCOLOR(0,0,0);
      }
      ws2812_write_leds(led_new_state);
    }  
    seq=(seq+1)%TIMESTEPS;
    vTaskDelay((1000/TIMESTEPS) / portTICK_PERIOD_MS);
  }
  vTaskDelete( NULL );
}

void setLed (unsigned char lednum, unsigned char r, unsigned char g, unsigned char b, unsigned char timing) {
  if (lednum < NUM_LEDS) {
    leds[lednum].r=r;
    leds[lednum].g=g;
    leds[lednum].b=b;
    leds[lednum].timing = timing;
  }
}

void displayStatusOnLED (int status)
{
  static int _last_status = 0;
  if (status != _last_status) {
    _last_status = status;
#ifdef DEBUG
    Serial.print("New status : ");
    Serial.println(status);
#endif
    if (status==INT_CALIBRATE) {
      // fast white blink
      setLed (0,0xff,0xff,0xff,0x55);
    }
    if (status==HOST_INIT) {
      // Slow red blink
      setLed (0,0xff,0x00,0x0,0x18);
    }
    if (status==HOST_MODE_USER) {
      // Slow green blink
      setLed (0,0x00,0xff,0x00,0x18);
    }
    if (status==HOST_MODE_LOCAL) {
      // Slow blue blink
      setLed (0,0x00,0x00,0xFF,0x18);
    }
    if (status==INT_DISCONNECTED) {
      // medium red blink
      setLed (0,0xff,0x00,0x00,0x55);
    }
    if (status==INT_RXERROR) {
      // fastred blink
      setLed (0,0xff,0x00,0x00,0x33);
    }
    if (status==HOST_MODE_DISARMED) {
      // fast green blink
      setLed (0,0x00,0xff,0x00,0x55);
    }
  }
}
void processStatusFromHost (const char *status) {

  if (strcmp(status, "init")==0) {
    displayStatusOnLED(HOST_INIT);
  }
  if (strcmp(status, "disarmed")==0) {
    displayStatusOnLED(HOST_MODE_DISARMED);
  }
  if (strcmp(status, "user")==0) {
    displayStatusOnLED(HOST_MODE_USER);
  }
  if (strcmp(status, "local")==0) {
    displayStatusOnLED(HOST_MODE_LOCAL);
  }
}

void timedCheckOutput()
{
  uint32_t t = esp_timer_get_time()/1000;
  if (pwm_length[PWM_RC_THROTTLE_INPUT_PIN] == 0) {
    sprintf(buff, "%d,-1,-1,-1,-1,-1,-1\n", t);
     displayStatusOnLED(INT_RXERROR);   
  } else {
    sprintf(buff, "%d,%d,%d,%d,%d,%d\n", t, 
    pwm_length[PWM_RC_THROTTLE_INPUT_PIN], 
    pwm_length[PWM_RC_STEERING_INPUT_PIN], 
    pwm_length[PWM_RC_CH5_INPUT_PIN], 
    pwm_length[PWM_RC_CH6_INPUT_PIN],
    pwm_length[PWM_SPEEDOMETER_INPUT_PIN]);
  }
  printf(buff);    
  memset(pwm_length, 0, sizeof(pwm_length[0])*MAX_GPIO);   
}

//
// MAIN
//

void app_main()
{
  int tick=0;
  struct led_state new_state;
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;
  
  uart_set_baudrate(UART_NUM_0, 2000000); 
  memset (leds, 0, sizeof(leds));
  mcpwm_init_control();
  init_rx_gpio();
  init_led_gpio();
  ws2812_control_init();

  switchOffLed();
  displayStatusOnLED(INT_DISCONNECTED);   
  xTaskCreate( updateLed, "LED STATUS", STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle );

  mcpwm_set_throttle_pwm(1500);
  mcpwm_set_steering_pwm(1500);
  // Set configuration of timer0 for high speed channels
  //
  while(1) {
    vTaskDelay(OUTPUTLOOP / portTICK_PERIOD_MS);
    timedCheckOutput();
  };
}
