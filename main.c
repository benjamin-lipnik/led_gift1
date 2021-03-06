#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "stm8s003.h"
#include "uart.h"
#include "stm8_utility.h"

//!!!! PWM !!!!
//PORTD D
#define RGB_R_PIN 2
#define RGB_G_PIN 3
#define RGB_B_PIN 4

//PORTC
#define SW_PIN1   4
#define SW_PIN2   5 //ENC_SW
#define ENC_PIN1  6
#define ENC_PIN2  7
#define CLOCK_PIN 3

//PORTA
#define SYNC_DETECT_PIN 3

//Defines
#define ST_ANIMACIJ 7
#define ANIMT_MIN 1
#define ANIMT_MAX 30

const int8_t sin_table[128] = {0,3,6,9,12,16,19,22,25,28,31,34,37,40,43,46,49,51,54,57,60,63,65,68,71,73,76,78,81,83,85,88,90,92,94,96,98,100,102,104,106,107,109,111,112,113,115,116,117,118,120,121,122,122,123,124,125,125,126,126,126,127,127,127,127,127,127,127,126,126,126,125,125,124,123,122,122,121,120,118,117,116,115,113,112,111,109,107,106,104,102,100,98,96,94,92,90,88,85,83,81,78,76,73,71,68,65,63,60,57,54,51,49,46,43,40,37,34,31,28,25,22,19,16,12,9,6,3};
const volatile int8_t rotacijska_tabela[16] = {0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0};
volatile const uint8_t char_pos_lookup[10] = {4, 8, 5, 1, 3, 8, 8, 0, 6, 2};
volatile const uint8_t char_pos_circular[7] = {0,1,2,3,4,6,5};
volatile const uint8_t rgb_off[3] = {0,0,0};

volatile uint8_t char_select_index = 0;
volatile uint8_t frame_buffer[10][3];
volatile uint8_t frame_buffer_preloaded[10][3];
volatile uint8_t global_rgb[3] = {127, 0 ,0};

volatile uint8_t avtomatsko = 0;
volatile uint8_t avtomatsko_preklop = 1;

volatile uint8_t enc_color_select = 0;
volatile uint8_t encoder_count = 8;

volatile int8_t encoder_sub_count = 0;
volatile uint8_t last_ab = 0;

volatile uint32_t time_counter = 0;
uint16_t global_loopt = 0;

volatile uint8_t anim_select = 0;
volatile uint8_t animt = 5;

//Fore debounce purposes
//bit 7 stores if switch is active or not (active high)
//bits 6:0 store debounce time
volatile uint8_t sw2_active_or_block = 0;
volatile uint8_t nacin_odtenki = 0;

int putchar(int c) {
    uart_write(c);
    return 0;
}

int8_t get_sin_val(uint8_t pos) {
  int8_t val = sin_table[pos & 0x7F];
  if(pos & _BV(7))
    return -val;
  return val;
}

uint8_t * odtenki(uint8_t pos) {
  static uint8_t m_rgb[3];
  if(pos >= 213) { // 213 -> 255, (43,42,43,43,42,43)
    m_rgb[0] = 255;
    m_rgb[1] = 0;
    m_rgb[2] = 6 * (255 - pos);
  }
  else if(pos >= 171) { //171 -> 212
    m_rgb[0] = 6 * (pos - 170);
    m_rgb[1] = 0;
    m_rgb[2] = 255;
  }
  else if(pos >= 128) { //128 -> 170
    m_rgb[0] = 0;
    m_rgb[1] = 6 * (170 - pos);
    m_rgb[2] = 255;
  }
  else if(pos >= 85) { //85 -> 127
    m_rgb[0] = 0;
    m_rgb[1] = 255;
    m_rgb[2] = 6 * (pos-85);
  }
  else if(pos >= 43) { //43 -> 84
    m_rgb[0] = 6 * (85 - pos);
    m_rgb[1] = 255;
    m_rgb[2] = 0;
  }
  else { // 0 -> 42
    m_rgb[0] = 255;
    m_rgb[1] = 6 * pos;
    m_rgb[2] = 0;
  }
  return m_rgb;
}
uint8_t * regija_odtenka(uint8_t pos) {
  static uint8_t m_rgb[3];

  m_rgb[0] = 0;
  m_rgb[1] = 0;
  m_rgb[2] = 0;

  if(pos >= 213 || pos < 43)
    m_rgb[0] = 255;
  else if(pos >= 43 &&  pos < 128)
    m_rgb[1] = 255;
  else
    m_rgb[2] = 255;
  return m_rgb;
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
  //PC_IDR
  uint8_t ab = !(PC_IDR & _BV(ENC_PIN2)) << 1 | !(PC_IDR & _BV(ENC_PIN1));
  int8_t iz_tabele = rotacijska_tabela[last_ab << 2 | ab];
  encoder_sub_count += iz_tabele;
  last_ab = ab;

  //gledanje gumba 2
  if(!sw2_active_or_block) {
    if(!(PC_IDR & _BV(SW_PIN2)))
      sw2_active_or_block = 100;
  }

  if(avtomatsko) {
    if(sw2_active_or_block) {
      if(abs(encoder_sub_count) >= 4) {
        if(iz_tabele < 0 && animt > ANIMT_MIN || iz_tabele > 0 && anim_select < (ANIMT_MAX-1))
          animt += iz_tabele;
        encoder_sub_count = 0;
        avtomatsko_preklop = 0;
      }
      else
        avtomatsko_preklop = 1;
    }else {
      if(abs(encoder_sub_count) >= 4) {
        avtomatsko_preklop = 0;
        if(iz_tabele < 0 && anim_select > 0 || iz_tabele > 0 && anim_select < (ST_ANIMACIJ-1))
          anim_select += iz_tabele;
        encoder_sub_count = 0;
      }
    }
  }
  else {
    uint8_t enc_sensitivity = 4;

    if(sw2_active_or_block) {
      if(!nacin_odtenki) {
        enc_sensitivity = 16;
        if(++enc_color_select >= 3)
          enc_color_select = 0;
        encoder_count = global_rgb[enc_color_select] / 8;
      }
      if(abs(encoder_sub_count) >= enc_sensitivity) {
        encoder_count += iz_tabele;
        encoder_sub_count = 0;
        uint8_t * rgb = odtenki(encoder_count * 4);
        memcpy(global_rgb, rgb, 3);
        nacin_odtenki = 1;
        enc_color_select = 0;
      }
    }
    else {
      if(abs(encoder_sub_count) >= 4) {
        if(encoder_count > 31)
          encoder_count = 31;
        if(encoder_count == 0 && iz_tabele < 0)
          return;
        if(encoder_count == 31 && iz_tabele > 0)
          return;

        encoder_count += iz_tabele;
        encoder_sub_count = 0;
        nacin_odtenki = 0;
        global_rgb[enc_color_select] = encoder_count * 8;
      }
    }
  }
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
    uint8_t * rgb = odtenki(loopt - i * 20);
    frame_buffer[char_pos_circular[i]][0] = rgb[0];
    frame_buffer[char_pos_circular[i]][1] = rgb[1];
    frame_buffer[char_pos_circular[i]][2] = rgb[2];
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
      uint8_t * rgb = odtenki(pos * 40);
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
    uint8_t * rgb = odtenki(loopt);

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
void animacija6(uint8_t loopt) {
  //loopt = global_loopt / 4;
  if(loopt % 28)
    return;

  uint8_t * rgb_1 = regija_odtenka(loopt);
  memcpy(frame_buffer[2], rgb_1, 3);

  uint8_t * rgb_2 = regija_odtenka(loopt - 28);
  memcpy(frame_buffer[1], rgb_2, 3);
  memcpy(frame_buffer[3], rgb_2, 3);
  memcpy(frame_buffer[5], rgb_2, 3);
  memcpy(frame_buffer[6], rgb_2, 3);

  uint8_t * rgb_3 = regija_odtenka(loopt - 56);
  memcpy(frame_buffer[0], rgb_3, 3);
  memcpy(frame_buffer[4], rgb_3, 3);
}
void animacija7(uint8_t loopt) {
  for(uint8_t i = 0; i < 7; i++) {
    memcpy(frame_buffer[i], rgb_off, 3);
  }

  loopt *= 2;
  uint8_t korak = loopt / 32;

  switch (korak) {
    case 0:
      frame_buffer[0][0] = 200;
      frame_buffer[1][0] = 20;
      frame_buffer[5][0] = 200;
    break;
    case 1:
      frame_buffer[1][0] = 200;
      frame_buffer[0][0] = 20;
      frame_buffer[5][0] = 200;
    break;
    case 2:
      frame_buffer[2][0] = 200;
      frame_buffer[1][0] = 20;
      frame_buffer[0][0] = 2;
      frame_buffer[6][0] = 200;
    break;
    case 3:
      frame_buffer[3][0] = 200;
      frame_buffer[2][0] = 20;
      frame_buffer[1][0] = 2;
      frame_buffer[6][0] = 200;
    break;
    case 4:
      frame_buffer[4][0] = 200;
      frame_buffer[3][0] = 20;
      frame_buffer[2][0] = 2;
      frame_buffer[6][0] = 200;
    break;
    case 5:
      frame_buffer[3][0] = 200;
      frame_buffer[4][0] = 20;
      frame_buffer[6][0] = 200;
    break;
    case 6:
      frame_buffer[2][0] = 200;
      frame_buffer[3][0] = 20;
      frame_buffer[4][0] = 2;
      frame_buffer[5][0] = 200;
    break;
    case 7:
      frame_buffer[1][0] = 200;
      frame_buffer[2][0] = 20;
      frame_buffer[3][0] = 2;
      frame_buffer[5][0] = 200;
    break;
  }
}

void (* animacije[])(uint8_t) = {animacija1, animacija2, animacija3, animacija4, animacija5, animacija6, animacija7};

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
    while(1) {
      //racunaj funkcijo -> izbira med rocno in avtomatiko
      //printf("enc: %u, time: %lu\n\r", encoder_count, time_counter);
      for(uint8_t i = 0; i < animt; i++) {
        util_delay_milliseconds(1);
        if(sw2_active_or_block) {
          sw2_active_or_block--;
        }
      }

      if(++global_loopt >= 1000) {
        global_loopt = 0;
        if(avtomatsko_preklop)
          if(++anim_select >= ST_ANIMACIJ) {
            anim_select = 0;
          }
      }
      avtomatsko = !(PC_IDR & _BV(SW_PIN1));
      if(avtomatsko) // AUTO
        animacije[anim_select](global_loopt);
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
