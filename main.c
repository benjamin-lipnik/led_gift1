#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "stm8s003.h"
#include "uart.h"
#include "stm8_utility.h"

//volatile uint8_t auto = 0;
volatile uint8_t char_select_index = 0;
//volatile uint8_t global_rgb[3] = {0,128,255};
volatile const uint8_t rgb_off[3] = {0,0,0};
volatile uint8_t frame_buffer[10][3];
volatile uint8_t frame_buffer_preloaded[10][3];
volatile uint8_t global_rgb[3] = {127, 0 ,0};

const volatile uint8_t char_pos_lookup[10] = {4, 8, 5, 1, 3, 8, 8, 0, 6, 2};

const int8_t sin_table[128] = {0,3,6,9,12,16,19,22,25,28,31,34,37,40,43,46,49,51,54,57,60,63,65,68,71,73,76,78,81,83,85,88,90,92,94,96,98,100,102,104,106,107,109,111,112,113,115,116,117,118,120,121,122,122,123,124,125,125,126,126,126,127,127,127,127,127,127,127,126,126,126,125,125,124,123,122,122,121,120,118,117,116,115,113,112,111,109,107,106,104,102,100,98,96,94,92,90,88,85,83,81,78,76,73,71,68,65,63,60,57,54,51,49,46,43,40,37,34,31,28,25,22,19,16,12,9,6,3};

volatile uint8_t avtomatsko = 0;
volatile uint8_t encoder_count = 8;
volatile uint8_t enc_color_select = 0;
volatile int8_t encoder_sub_count = 0;
volatile uint8_t last_ab = 0;
volatile uint32_t time_counter = 0;

int8_t get_sin_val(uint8_t pos) {
  int8_t val = sin_table[pos & 0x7F];
  if(pos & _BV(7))
    return -val;
  return val;
}


//!!!! PWM !!!!
//PORTD D
#define RGB_R_PIN 2
#define RGB_G_PIN 3
#define RGB_B_PIN 4

//PORTC
#define SW_PIN1   4 //ENC_SW
#define SW_PIN2   5
#define ENC_PIN1  6
#define ENC_PIN2  7
#define CLOCK_PIN 3

//PORTA
#define SYNC_DETECT_PIN 3

const volatile int8_t rotacijska_tabela[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};

uint8_t * hsv2rgb(uint8_t h, uint8_t s, uint8_t v)
{
  static uint8_t m_rgb[3];
  unsigned char region, remainder, p, q, t;

  if (s == 0)
  {
      m_rgb[0] = v;
      m_rgb[1] = v;
      m_rgb[2] = v;
      return m_rgb;
  }

  region = h / 43;
  remainder = (h - (region * 43)) * 6;

  p = (v * (255 - s)) >> 8;
  q = (v * (255 - ((s * remainder) >> 8))) >> 8;
  t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

  switch (region)
  {
      case 0:
          m_rgb[0] = v;
          m_rgb[1] = t;
          m_rgb[2] = p;
          break;
      case 1:
          m_rgb[0] = q;
          m_rgb[1] = v;
          m_rgb[2] = p;
          break;
      case 2:
          m_rgb[0] = p;
          m_rgb[1] = v;
          m_rgb[2] = t;
          break;
      case 3:
          m_rgb[0] = p;
          m_rgb[1] = q;
          m_rgb[2] = v;
          break;
      case 4:
          m_rgb[0] = t;
          m_rgb[1] = p;
          m_rgb[2] = v;
          break;
      default:
          m_rgb[0] = v;
          m_rgb[1] = p;
          m_rgb[2] = q;
          break;
  }
  return m_rgb;
}

int putchar(int c) {
    uart_write(c);
    return 0;
}

void set_rgb (uint8_t * rgb) {
  TIM2_CCR1H = 0;
  TIM2_CCR1L = 255-rgb[2];

  TIM2_CCR2H = 0;
  TIM2_CCR2L = 255-rgb[1];

  TIM2_CCR3H = 0;
  TIM2_CCR3L = 255-rgb[0];
}

void next_char_select() __interrupt(TIM4_ISR) {
  set_rgb(rgb_off);
  PC_ODR |= _BV(CLOCK_PIN);

  time_counter++;
  if(++char_select_index >= 10) {
    char_select_index = 0;
  }
  else if(char_select_index == 5) {
    for(uint8_t i = 0; i < 10; i++) {
      for(uint8_t j = 0; j < 3; j++) {
        frame_buffer_preloaded[i][j] = frame_buffer[i][j];
      }
    }
  }

  set_rgb(frame_buffer_preloaded[char_pos_lookup[char_select_index]]);

  PC_ODR &= ~_BV(CLOCK_PIN);
  TIM4_SR &= ~_BV(TIM4_SR_UIF); //clear ISR flag
}

void inputs_isr() __interrupt(EXTI2_ISR) {
  if(avtomatsko)
    return;
  //PC_IDR
  uint8_t ab = !(PC_IDR & _BV(ENC_PIN2)) << 1 | !(PC_IDR & _BV(ENC_PIN1));
  int8_t iz_tabele = rotacijska_tabela[last_ab << 2 | ab];
  encoder_sub_count += iz_tabele;
  last_ab = ab;

  if(!(PC_IDR & _BV(SW_PIN2))) {
    if(++enc_color_select >= 3)
      enc_color_select = 0;
      encoder_count = global_rgb[enc_color_select] / 8;
  }

  if(abs(encoder_sub_count) >= 4) {

    if(encoder_count == 0 && iz_tabele < 0)
      return;
    if(encoder_count == 31 && iz_tabele > 0)
      return;

    encoder_count += iz_tabele;
    encoder_sub_count = 0;
    global_rgb[enc_color_select] = encoder_count * 8;
  }

  //CHECK ENCODER

  //CHECK BUTTONS

}

void sync_detect_isr() __interrupt(EXTI0_ISR) {
  //PA_IDR
  //CHECK IF OUTPUT FOR LED 1 IS ACTIVE
  if(PA_IDR & _BV(SYNC_DETECT_PIN)) {
    char_select_index = 0;
  }
}

void pwm_init() {
  //Pin inits
  PD_DDR |= _BV(RGB_R_PIN) | _BV(RGB_G_PIN) | _BV(RGB_B_PIN);
  PD_CR1 |= _BV(RGB_R_PIN) | _BV(RGB_G_PIN) | _BV(RGB_B_PIN);

  //PWM Timer
  TIM2_PSCR = 0b11;

  TIM2_ARRH = 0;
  TIM2_ARRL = 254; //Max / Top

  //TIM2_IER |= _BV(TIM2_IER_UIE);
  TIM2_CR1 |= _BV(TIM2_CR1_CEN);

  TIM2_CCMR1 |=  _BV(5) | _BV(6); //pwm mode 1
  TIM2_CCMR1 &= ~(_BV(0) | _BV(1));//bita 0 in 1 moreta bit na nic ce ces met izhod
  //TIM2_CCMR1 |= _BV(3); //PRELOAD ENABLE

  TIM2_CCER1 |= _BV(0); //Output enable

  TIM2_CCMR2 |=  _BV(5) | _BV(6); //pwm mode 1
  TIM2_CCMR2 &= ~(_BV(0) | _BV(1));//bita 0 in 1 moreta bit na nic ce ces met izhod
  TIM2_CCER1 |= _BV(4);
  //TIM2_CCMR2 |= _BV(3); //PRELOAD ENABLE

  TIM2_CCMR3 |= _BV(5) | _BV(6); //pwm mode 1
  TIM2_CCMR3 &= ~(_BV(0) | _BV(1));//bita 0 in 1 moreta bit na nic ce ces met izhod
  TIM2_CCER2 |= _BV(0);
  //TIM2_CCMR3 |= _BV(3); //PRELOAD ENABLE

  TIM2_CCR1H = 0;
  TIM2_CCR1L = 255; //Duty //b

  TIM2_CCR2H = 0;
  TIM2_CCR2L = 255; //g

  TIM2_CCR3H = 0;
  TIM2_CCR3L = 255; //r
}

void sync_detect_init() {
  PA_DDR &= ~_BV(SYNC_DETECT_PIN); //INPUT
  PA_CR1 &= ~_BV(SYNC_DETECT_PIN); //FLOATING
  PA_CR2 |= _BV(SYNC_DETECT_PIN); //INTERRUPTS EXTI0
}

void inputs_init() {
  CPU_CCR |= _BV(5) | _BV(3);
  EXTI_CR1 = 0xff;
  PC_ODR &= ~(_BV(SW_PIN1) | _BV(SW_PIN2) | _BV(ENC_PIN1) | _BV(ENC_PIN2)); //INPUT
  PC_CR1 |= (_BV(SW_PIN1) | _BV(SW_PIN2) | _BV(ENC_PIN1) | _BV(ENC_PIN2)); //PP or Floating
  PC_CR2 |= _BV(SW_PIN1) | _BV(SW_PIN2) | _BV(ENC_PIN1) | _BV(ENC_PIN2); //INTERRUPTS EXTI2
}

void timer4_init() {
  TIM4_CR1 |= _BV(0);//CEN
  TIM4_IER |= _BV(0); //UIE
  TIM4_PSCR = 0b101;
  TIM4_ARR = 255;
}

void char_select_init() {
  PC_DDR |= _BV(CLOCK_PIN);
  PC_CR1 |= _BV(CLOCK_PIN);
}


void animacija1(uint8_t loopt) {
  loopt *= 4;
  for(uint8_t i = 0; i < 7; i++) {
    frame_buffer[i][0] = 0;
    frame_buffer[i][1] = 0;
    frame_buffer[i][2] = 0;

    if(i & 1) {
      frame_buffer[i][0] = 128 + get_sin_val(loopt + i*60);
    }
    else {
      frame_buffer[i][1] = 128 + get_sin_val(loopt + i*60);
    }
  }
}
void animacija2(uint8_t loopt) {
  for(uint8_t i = 0; i < 7; i++) {
    uint8_t * rgb = hsv2rgb(loopt - i * 30, 255,255);
    frame_buffer[i][0] = rgb[0];
    frame_buffer[i][1] = rgb[1];
    frame_buffer[i][2] = rgb[2];
  }
}
void animacija3(uint8_t loopt) {
  static uint8_t pos = 0;
  loopt *= 2;
  for(uint8_t i = 0; i < 7; i++) {
    if(loopt == 120) {
      pos++;
    }
    if(loopt < 120) {
      frame_buffer[i][0] = 0;
      frame_buffer[i][1] = 0;
      frame_buffer[i][2] = 0;
    }
    else {
      uint8_t * rgb = hsv2rgb(pos * 40, 255, 255);
      frame_buffer[i][0] = rgb[0];
      frame_buffer[i][1] = rgb[1];
      frame_buffer[i][2] = rgb[2];
    }
  }
}
void animacija4(uint8_t loopt) {
  for(uint8_t i = 0; i < 7; i++) {
    frame_buffer[i][0] = 0;
    frame_buffer[i][1] = 0;
    frame_buffer[i][2] = 0;

    frame_buffer[i][i%3] = 200 * (loopt > ((i+1) * 30));
  }
}
void animacija5(uint8_t loopt) {
  for(uint8_t i = 0; i < 7; i++) {
    uint8_t * rgb = hsv2rgb(loopt, 255,128);

    if(i < 5) {
      frame_buffer[i][0] = 128+get_sin_val(loopt);
      frame_buffer[i][1] = 128+get_sin_val(loopt);
      frame_buffer[i][2] = 128+get_sin_val(loopt);
    }
    else {
      frame_buffer[i][0] = 128+get_sin_val(loopt+128);
      frame_buffer[i][1] = 128+get_sin_val(loopt+128);
      frame_buffer[i][2] = 128+get_sin_val(loopt+128);
    }
  }
}

uint8_t anim_select = 0;
void (* animacije[])(uint8_t) = {animacija1, animacija2, animacija3, animacija4, animacija5};

int main () {
    CLK_CKDIVR = 0;//16mhz
    uart_init(9600);
    pwm_init();
    inputs_init();
    char_select_init();
    sync_detect_init();
    timer4_init();
    enable_interrupts();


    //Main loop
    uint16_t loopt = 0;
    while(1) {
      //racunaj funkcijo -> izbira med rocno in avtomatiko
      //printf("enc: %u, time: %lu\n\r", encoder_count, time_counter);
      util_delay_milliseconds(5);
      loopt++;

      if(loopt > 1000) {
        loopt = 0;
        if(++anim_select >= 5)
          anim_select = 0;
      }
      avtomatsko = !(PC_IDR & _BV(SW_PIN1));
      if(avtomatsko) // AUTO
        animacije[anim_select](loopt);
      else {
        for(uint8_t i = 0; i < 7; i++) {
          frame_buffer[i][0] = global_rgb[0];
          frame_buffer[i][1] = global_rgb[1];
          frame_buffer[i][2] = global_rgb[2];
        }
      }

      //animacija1(loopt);
      //animacija2(loopt);
      //animacija3(loopt);


      //for(uint8_t i = 0; i < 10; i++) {
      //  for(int8_t c = 0; c < 3; c++) {
      //    frame_buffer[i][c] = 128 + get_sin_val(loopt + 30 * i + 10 * c);
      //  }
      //}
    }
}
