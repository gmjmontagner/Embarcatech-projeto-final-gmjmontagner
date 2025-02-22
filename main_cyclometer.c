/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/timer.h"

#define MPU9250_ADDR  0x68
#define AK8963_ADDR   0x0C
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define MAG_XOUT_L    0x03
#define CNTL1         0x0A
#define WHO_AM_I_REG  0x75
#define GYRO_XOUT_H 0x43

const uint LEDG = 11;
const uint LEDB = 12;
const uint LEDR = 13;

const uint LED_OFF = 0;
const uint LED_LOW = 60;
const uint LED_MID = 128;
const uint LED_HIGH = 255;

const uint ACCL_LOW = 5000;
const uint ACCL_MID = 10000;
const uint ACCL_HIGH = 15000;

const uint16_t PERIOD_R = 15625;
const float DIVIDER_PWM_R = 8;
const uint16_t LED_STEP_R = 781;
const uint32_t PWM_REFRESH_LEVEL_R = 10;

const uint16_t PERIOD_G = 3125;
const float DIVIDER_PWM_G = 4;
const uint16_t LED_STEP_G = 156;
const uint32_t PWM_REFRESH_LEVEL_G = 100;

const uint16_t slice_g = 5;
const uint16_t slice_r = 6;

volatile static uint led_level_g = 0;
volatile static uint led_level_r = 0;
volatile static uint led_level_b = 0;

//Frequencia desejada para o pisca do LED, em Hz
const static int frequenciaAtualiza = 10;

// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 500
#endif

void setup_pwm_r();
void setup_pwm_g();
void pwm_irq_handler();
void mpu9250_write_byte(uint8_t reg, uint8_t data);
void mpu9250_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length);
void mpu9250_init();
void read_accel_gyro(int16_t *accel, int16_t *gyro);
void read_magnetometer(int16_t *mag);
int pico_led_init(void);
void pico_set_led(bool led_on);
bool repeating_timer_callback(struct repeating_timer *t);

bool repeating_timer_callback(struct repeating_timer *t) {
    int16_t accel[3], gyro[3], mag[3];
    read_magnetometer(mag);
    read_accel_gyro(accel, gyro);
    if(abs(accel[0])>ACCL_LOW){
        led_level_r = LED_LOW;
        if(abs(accel[0])>ACCL_MID){
            led_level_r = LED_MID;
            if(abs(accel[0])>ACCL_HIGH){
                led_level_r = LED_HIGH;
            }
        }
    } else {
        led_level_r = LED_OFF;
    }
    if(abs(accel[1])>ACCL_LOW){
        led_level_g = LED_LOW;
        if(abs(accel[1])>ACCL_MID){
            led_level_g = LED_MID;
            if(abs(accel[1])>ACCL_HIGH){
                led_level_g = LED_HIGH;
            }
        }
    } else {
        led_level_g = LED_OFF;
    }
    if(abs(accel[2])>ACCL_LOW){
        led_level_b = LED_LOW;
        if(abs(accel[2])>ACCL_MID){
            led_level_b = LED_MID;
            if(abs(accel[2])>ACCL_HIGH){
                led_level_b = LED_HIGH;
            }
        }
    } else {
        led_level_b = LED_OFF;
    }
    printf("Mag X: %d, Y: %d, Z: %d\n", mag[0], mag[1], mag[2]);
    printf("Accel: X=%d Y=%d Z=%d | Gyro: X=%d Y=%d Z=%d\n", 
           accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
    return true;       // Retorna true para continuar repetindo
}

void setup_pwm_r(){
    gpio_set_function(LEDR,GPIO_FUNC_PWM);
    gpio_set_function(LEDB,GPIO_FUNC_PWM);
    uint slice = slice_r;//pwm_gpio_to_slice_num(LEDR);
    pwm_set_clkdiv(slice, DIVIDER_PWM_R);
    pwm_set_wrap(slice, PERIOD_R);
    pwm_set_gpio_level(LEDR, 1);
    pwm_set_gpio_level(LEDB, 1);
    pwm_set_enabled(slice, true);
  
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_enabled(PWM_IRQ_WRAP,true);
}
  
void setup_pwm_g(){
    gpio_set_function(LEDG,GPIO_FUNC_PWM);
    uint slice = slice_g;//pwm_gpio_to_slice_num(LEDG);
    pwm_set_clkdiv(slice, DIVIDER_PWM_G);
    pwm_set_wrap(slice, PERIOD_G);
    pwm_set_gpio_level(LEDG, 1);
    pwm_set_enabled(slice, true);
  
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_enabled(PWM_IRQ_WRAP,true);
}

void pwm_irq_handler(){
    uint this_slice = 0;
  static uint up_down_r = 1;
  static uint32_t count_r = 0;

  static uint up_down_g = 1;
  static uint32_t count_g = 0;

  uint32_t slice = pwm_get_irq_status_mask();
  if((slice & 0b100000)!= 0){
    this_slice = 5;
    pwm_clear_irq(slice_g);
  }
  if((slice & 0b1000000)!= 0){
    this_slice = 6;
    pwm_clear_irq(slice_r);
  }
  switch(this_slice){
    case slice_r:
        if( count_r++ < PWM_REFRESH_LEVEL_R) return;
        count_r = 0;
        pwm_set_gpio_level(LEDR, led_level_r);
        pwm_set_gpio_level(LEDB, led_level_b);
        break;
    case slice_g:
        if( count_g++ < PWM_REFRESH_LEVEL_G) return;
        count_g = 0;
        pwm_set_gpio_level(LEDG, led_level_g);
        break;
  }
}
  
void mpu9250_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    i2c_write_blocking(i2c1, MPU9250_ADDR, buffer, 2, false);
}

void mpu9250_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(i2c1, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU9250_ADDR, buffer, length, false);
}


void mpu9250_init() {
    uint8_t buf[2];

    // Despertar o MPU9250
    mpu9250_write_byte(PWR_MGMT_1, 0x00); // Wake up MPU9250

    // Ativar I2C pass-through para acessar o AK8963
    mpu9250_write_byte(0x37, 0x02); // Wake up AK8963

    // Configurar o AK8963 no modo contínuo (100 Hz)
    mpu9250_write_byte(CNTL1, 0x16); // 
}

void read_accel_gyro(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[12];
    mpu9250_read_bytes(ACCEL_XOUT_H, buffer, 12);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        gyro[i] = (buffer[6 + i * 2] << 8) | buffer[7 + i * 2];
    }
}

void read_magnetometer(int16_t *mag) {
    uint8_t buf[6];
    i2c_write_blocking(i2c1, AK8963_ADDR, (uint8_t[]){MAG_XOUT_L}, 1, true);
    i2c_read_blocking(i2c1, AK8963_ADDR, buf, 6, false);

    for (int i = 0; i < 3; i++) {
        mag[i] = (buf[i*2 + 1] << 8) | buf[i*2]; // AK8963 usa little-endian
    }
}

// Perform initialisation
int pico_led_init(void) {
#if defined(PICO_DEFAULT_LED_PIN)
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // For Pico W devices we need to initialise the driver etc
    return cyw43_arch_init();
#endif
}

// Turn the led on or off
void pico_set_led(bool led_on) {
#if defined(PICO_DEFAULT_LED_PIN)
    // Just set the GPIO on or off
    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
    // Ask the wifi "driver" to set the GPIO on or off
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
#endif
}

int main() {
    stdio_init_all();
    // Cria uma estrutura de timer
    struct repeating_timer meu_timer;        
    // Inicia um timer repetitivo com a frequencia desejada
    bool sucesso = add_repeating_timer_ms(1000/(frequenciaAtualiza), repeating_timer_callback, NULL, &meu_timer);
    
    //setup pwm
    setup_pwm_r();
    setup_pwm_g();  
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    i2c_init(i2c1, 100 * 1000);
    mpu9250_init();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);
    while (true) {
        pico_set_led(true);
        sleep_ms(LED_DELAY_MS);
        pico_set_led(false);
        sleep_ms(LED_DELAY_MS);
    }
}