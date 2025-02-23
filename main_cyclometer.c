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
#include "hardware/pio.h"
#include "quadrature_encoder.pio.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

// Definicoes I2C para MPU9250 e AK8963 (embarcado na mesma PCB do MPU9250)
#define MPU9250_ADDR  0x68
#define AK8963_ADDR   0x0C
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B
#define MAG_XOUT_L    0x03
#define CNTL1         0x0A
#define WHO_AM_I_REG  0x75
#define GYRO_XOUT_H 0x43

// Variaveis de estado da maquina de estados principal
#define WAIT_DATA 0
#define RECORD_DATA 1

// Definicoes para usar o LED da placa Pi Pico W
// Pico W devices use a GPIO on the WIFI chip for the LED,
// so when building for Pico W, CYW43_WL_GPIO_LED_PIN will be defined
#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

// Pinos GPIO do LED RGB
const uint LEDG = 11;
const uint LEDB = 12;
const uint LEDR = 13;

// Parametros para exibicao em intensidade de luz associada a aceleracao no LED RGB
const uint LED_OFF = 0;
const uint LED_LOW = 60;
const uint LED_MID = 128;
const uint LED_HIGH = 255;

// Parametros para classificacao da intensidade da aceleracao, de forma a permitir a exibicao no LED RGB
const uint ACCL_LOW = 5000;
const uint ACCL_MID = 10000;
const uint ACCL_HIGH = 15000;

// Parametros para operacao do PWM - slice dos leds vermelho e azul
const uint16_t PERIOD_R = 15625;
const float DIVIDER_PWM_R = 8;
const uint16_t LED_STEP_R = 781;
const uint32_t PWM_REFRESH_LEVEL_R = 10;

// Parametros para operacao do PWM - slice do led verde
const uint16_t PERIOD_G = 3125;
const float DIVIDER_PWM_G = 4;
const uint16_t LED_STEP_G = 156;
const uint32_t PWM_REFRESH_LEVEL_G = 100;

// Slices PWM definidas para os pinos do LED RGB
const uint16_t slice_g = 5;
const uint16_t slice_r = 6;

//Frequencia desejada para a aquisicao
const static int frequencia_sensor = 2;

//Selecao do formato dos dados de saida
const print_pretty = 1;

// Variaveis para configuracao do Tacometro no PIO
static PIO pio = pio0;
const uint PIN_TACHO = 16;//Pino A da quadratura no GPIO16. O proximo pino (17) é o sinal B da quadratura
const uint sm = 0;

// Variaveis para leitura dos dados do tacômetro
static int new_value, delta, old_value = 0;
static int last_value = -1, last_delta = -1;

// Variaveis para controle do brilho no LED RGB de acordo com a aceleracao
volatile static uint led_level_g = 0;
volatile static uint led_level_r = 0;
volatile static uint led_level_b = 0;

// Trava para exibir sinalizacao final uma unica vez
bool sinalizacao_final = 0;

// Variaveis para leitura dos valores de medicoes inerciais do MPU9250 e magnetometro
static int16_t accel[3], gyro[3], mag[3];

// Variavel para controle do estado na maquina de estados principal
static int16_t state = WAIT_DATA;

// Variaveis para sincronizacao e sinalizacao entre o loop principal e as interrupcoes que controlam a aquisicao de dados
static uint8_t acq_mpu_done = 0;
static uint8_t acq_tacho_done = 0;

// Variavel para controlar o comportamento do sistema apos a finalizacao da aquisicao. Definicao do tempo total de aquisicao
static uint8_t acquisition_finalized = 0;
const int16_t total_acquisition_time_s = 30;

// Buffers para as strings com informacoes de temporizacao para exibicao na tela/escrita em arquivo
char datetime_buf_mpu[256];
char *datetime_str_mpu = &datetime_buf_mpu[0];
char datetime_buf_tacho[256];
char *datetime_str_tacho = &datetime_buf_tacho[0];

// Data e hora de inicio para timestamp da aquisicao de dados da MPU
datetime_t t_mpu = {
    .year  = 2025,
    .month = 02,
    .day   = 23,
    .dotw  = 0, // 0 is Sunday, so 5 is Friday
    .hour  = 23,
    .min   = 59,
    .sec   = 59
};

// Data e hora de inicio para timestamp da aquisicao de dados do tacômetro
datetime_t t_tacho = {
    .year  = 2025,
    .month = 02,
    .day   = 23,
    .dotw  = 0, // 0 is Sunday, so 5 is Friday
    .hour  = 23,
    .min   = 59,
    .sec   = 59
};


// Prototipos das funcoes
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
int64_t acquisition_finalized_callback(alarm_id_t id, __unused void *user_data);
void execute_tacho_read();
void set_RGB_leds(); 


// Le valor do encoder de quadratura que implementa o tacômetro
void execute_tacho_read(){

    // Le do programa PIO a leitura de quadratura nos pinos GP16 e GP17
    new_value = quadrature_encoder_get_count(pio, sm);

    // Calcula o delta
    delta = new_value - old_value;
    old_value = new_value;

    // Sempre que houver alteração na leitura, registrar.
    if (new_value != last_value || delta != last_delta ) {
        last_value = new_value;
        last_delta = delta;
    }
}

// Configura valor de brilho do LED RGB de acordo com o valor e orientação da aceleração
void set_RGB_leds(){

    // Aceleracao X diferente de nível "desligado", ou seja, no mínimo no nivel "baixo"
    if(abs(accel[0])>ACCL_LOW){
        led_level_r = LED_LOW;

        // Aceleracao X no nivel "médio"
        if(abs(accel[0])>ACCL_MID){
            led_level_r = LED_MID;

            // Aceleracao X no nivel "alto"
            if(abs(accel[0])>ACCL_HIGH){
                led_level_r = LED_HIGH;
            }
        }
    } else {

        // Aceleracao no eixo X no nivel "desligado"
        led_level_r = LED_OFF;
    }

    // Aceleracao Y diferente de nível "desligado", ou seja, no mínimo no nivel "baixo"
    if(abs(accel[1])>ACCL_LOW){
        led_level_g = LED_LOW;

        // Aceleracao Y no nivel "médio"
        if(abs(accel[1])>ACCL_MID){
            led_level_g = LED_MID;

            // Aceleracao Y no nivel "alto"
            if(abs(accel[1])>ACCL_HIGH){
                led_level_g = LED_HIGH;
            }
        }
    } else {
        
        // Aceleracao no eixo Y no nivel "desligado"
        led_level_g = LED_OFF;
    }
    
    // Aceleracao Z diferente de nível "desligado", ou seja, no mínimo no nivel "baixo"
    if(abs(accel[2])>ACCL_LOW){
        led_level_b = LED_LOW;
        
        // Aceleracao Z no nivel "médio"
        if(abs(accel[2])>ACCL_MID){
            led_level_b = LED_MID;
            
            // Aceleracao Z no nivel "alto"
            if(abs(accel[2])>ACCL_HIGH){
                led_level_b = LED_HIGH;
            }
        }
    } else {

        // Aceleracao no eixo Z no nivel "desligado"
        led_level_b = LED_OFF;
    }
}

// Callback para o one-time timer que marca o fim do tempo de aquisição
int64_t acquisition_finalized_callback(alarm_id_t id, __unused void *user_data) {
    acquisition_finalized = 1;
    printf("Aquisicao finalizada!\r\n");
    return 0;
}

// Callback do timer repetitivo configurado para executar a aquisicao de dados
bool repeating_timer_callback(struct repeating_timer *t) {

    //Obtem timestamp
    rtc_get_datetime(&t_tacho);
    rtc_get_datetime(&t_mpu);

    // Executa leitura do tacômetro
    execute_tacho_read();

    // Executa leitura do MPU9250
    read_magnetometer(mag);
    read_accel_gyro(accel, gyro);

    // Configura brilho dos LEDs de acordo com orientação no MPU9250
    if((sinalizacao_final == 1) || (acquisition_finalized == 0)){
        set_RGB_leds();
    }

    // Sinaliza aquisicao completa para maquina de estados principal
    acq_mpu_done = 1;
    acq_tacho_done = 1;

    return true;       // Retorna true para continuar repetindo
}

// Configuracao inicial de PWM para slice do LED vermelho e azul
void setup_pwm_r(){

    // Configura pinos como pwm
    gpio_set_function(LEDR,GPIO_FUNC_PWM);
    gpio_set_function(LEDB,GPIO_FUNC_PWM);

    uint slice = slice_r;

    // Configura valores temporais de funcionamento do PWM
    pwm_set_clkdiv(slice, DIVIDER_PWM_R);
    pwm_set_wrap(slice, PERIOD_R);
    pwm_set_gpio_level(LEDR, 1);
    pwm_set_gpio_level(LEDB, 1);
    pwm_set_enabled(slice, true);
  
    // Configura interrupção por wrap do PWM para atualização dos valores de brilho do LED RGB
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_enabled(PWM_IRQ_WRAP,true);
}
  
// Configuracao inicial de PWM para slice do LED verde
void setup_pwm_g(){

    // Configura pino como pwm
    gpio_set_function(LEDG,GPIO_FUNC_PWM);

    uint slice = slice_g;//pwm_gpio_to_slice_num(LEDG);
    
    // Configura valores temporais de funcionamento do PWM
    pwm_set_clkdiv(slice, DIVIDER_PWM_G);
    pwm_set_wrap(slice, PERIOD_G);
    pwm_set_gpio_level(LEDG, 1);
    pwm_set_enabled(slice, true);
  
    // Configura interrupção por wrap do PWM para atualização dos valores de brilho do LED RGB
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    pwm_clear_irq(slice);
    pwm_set_irq_enabled(slice, true);
    irq_set_enabled(PWM_IRQ_WRAP,true);
}

// Funcao que lida com a interrupcao do PWM e atualiza valor de brilho do LED RGB
void pwm_irq_handler(){

    // Variavel para detectar a slice que gerou a interrupcao
    uint this_slice = 0;

    // Contagem para limitar frequencia de atualizacao do PWM de acordo com PWM_REFRESH_LEVEL
    static uint32_t count_r = 0;
    static uint32_t count_g = 0;

    // Aqui é detectado qual slice gerou a interrupcao
    uint32_t slice = pwm_get_irq_status_mask();
    if((slice & 0b100000)!= 0){
        this_slice = 5;
        pwm_clear_irq(slice_g);
    }
    if((slice & 0b1000000)!= 0){
        this_slice = 6;
        pwm_clear_irq(slice_r);
    }

    // Executa codigo para LEDs vermelho e azul ou o verde, dependendo da slice do PWM
    switch(this_slice){
        case slice_r:
            if( count_r++ < PWM_REFRESH_LEVEL_R) return;

            // Ao alcancar o tempo de atualizacao do PWM, escreve no PWM o valor vindo da funcao set_RGB_leds
            count_r = 0;
            pwm_set_gpio_level(LEDR, led_level_r);
            pwm_set_gpio_level(LEDB, led_level_b);
            break;
        case slice_g:
            if( count_g++ < PWM_REFRESH_LEVEL_G) return;

            // Ao alcancar o tempo de atualizacao do PWM, escreve no PWM o valor vindo da funcao set_RGB_leds
            count_g = 0;
            pwm_set_gpio_level(LEDG, led_level_g);
            break;
    }
}
  
// Escrita basica no periférico MPU9250
void mpu9250_write_byte(uint8_t reg, uint8_t data) {
    uint8_t buffer[2] = {reg, data};
    i2c_write_blocking(i2c1, MPU9250_ADDR, buffer, 2, false);
}

// Leitura basica do periférico MPU9250
void mpu9250_read_bytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
    i2c_write_blocking(i2c1, MPU9250_ADDR, &reg, 1, true);
    i2c_read_blocking(i2c1, MPU9250_ADDR, buffer, length, false);
}

// Incializacao do periférico MPU9250
void mpu9250_init() {
    uint8_t buf[2];

    // Despertar o MPU9250
    mpu9250_write_byte(PWR_MGMT_1, 0x00); // Wake up MPU9250

    // Ativar I2C pass-through para acessar o AK8963
    mpu9250_write_byte(0x37, 0x02); // Wake up AK8963

    // Configurar o AK8963 no modo contínuo (100 Hz)
    mpu9250_write_byte(CNTL1, 0x16); // 
}

// Leitura dos registradores com dados de medicao do periférico MPU9250
void read_accel_gyro(int16_t *accel, int16_t *gyro) {
    uint8_t buffer[12];
    mpu9250_read_bytes(ACCEL_XOUT_H, buffer, 12);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        gyro[i] = (buffer[6 + i * 2] << 8) | buffer[7 + i * 2];
    }
}

// Leitura dos registradores com dados de medicao do periférico AK8963 (magnetometro) embarcado com o MPU9250
void read_magnetometer(int16_t *mag) {
    uint8_t buf[6];
    i2c_write_blocking(i2c1, AK8963_ADDR, (uint8_t[]){MAG_XOUT_L}, 1, true);
    i2c_read_blocking(i2c1, AK8963_ADDR, buf, 6, false);

    for (int i = 0; i < 3; i++) {
        mag[i] = (buf[i*2 + 1] << 8) | buf[i*2]; // AK8963 usa little-endian
    }
}

// Inicializacao para uso do LED do Pi Pico W
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

// Escrita no LED do Pi Pico W
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

// Função principal - ponto de entrada do código
int main() {

    //Inicialização geral
    stdio_init_all();

    // Cria estruturas de timer
    struct repeating_timer meu_timer, meu_timer_2;  

    // Inicia um timer repetitivo com a frequencia desejada
    bool sucesso = add_repeating_timer_ms(1000/(frequencia_sensor), repeating_timer_callback, NULL, &meu_timer);
    

    //setup pwm para LED RGB
    setup_pwm_r();
    setup_pwm_g(); 
    
    // Configura interface I2C1
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);
    i2c_init(i2c1, 100 * 1000);

    // Inicializa corretamente o periférico MPU9250 (acelerometro, gyro e magnetometro)
    mpu9250_init();

    // Inicializa LED on-board
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    // Inicializa configuração PIO do encoder quadratura (KY040)
    pio_add_program(pio, &quadrature_encoder_program);
    quadrature_encoder_program_init(pio, sm, PIN_TACHO, 0);

    // Verifica configuração de sensor giroscópio interno ao MPU9250
    uint8_t gyro_config;
    i2c_write_blocking(i2c1, MPU9250_ADDR, (uint8_t[]){0x1B}, 1, true);
    i2c_read_blocking(i2c1, MPU9250_ADDR, &gyro_config, 1, false);
    printf("Gyro Config Register: 0x%02X\n", gyro_config);
    
    // Inicializa o RTC
    rtc_init();

    // Configura data e hora iniciais no RTC
    rtc_set_datetime(&t_mpu);
    rtc_set_datetime(&t_tacho);

    // Atraso necessário após configurar RTC
    // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_get_datetime() is called.
    // The delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
    sleep_us(64);

    // Configura timer repetitivo para aquisição
    add_alarm_in_ms(total_acquisition_time_s*1000, acquisition_finalized_callback, NULL, false);

    // Este delay possibilita abrir o terminal serial no Putty antes da primeira mensagem ser exibida
    sleep_ms(5000);

    // Mensagem inicial/Cabecalho do arquivo
    printf("Timestamp; Tacho Position; \r\n");
    printf("Timestamp; Mag X; Mag Y; Mag Z; Accel X; Accel Y; Accel Z; Gyro X; Gyro Y; Gyro Z; \r\n");
   
    // Loop principal
    while (true) {

        // Checa se já passou o tempo estipulado de aquisicao
        if(acquisition_finalized == 0){

            // Enquanto nao passou o tempo total de aquisicao, continua na maquina de estados
            switch(state){

                // Espera o proximo ciclo completo de aquisicao, com dados validos de aceleracao e tacho
                case WAIT_DATA:
                    if((acq_mpu_done == 1)&&(acq_tacho_done == 1)){
                        acq_mpu_done = 0;
                        acq_tacho_done = 0;
                        state = RECORD_DATA;
                    }
                    break;
                
                // Obtem string com a timestamp de cada sensor e exibe dados na tela no formato bonito ou para gravacao em arquivo
                case RECORD_DATA:
                    datetime_to_str(datetime_str_mpu, sizeof(datetime_buf_mpu), &t_mpu);
                    datetime_to_str(datetime_str_tacho, sizeof(datetime_buf_tacho), &t_tacho);
                    if(print_pretty == 1){
                        printf("%s      > Tacho: %6d\r\n", datetime_str_tacho, new_value);

                        // Exibindo dados de acelerometro e gyro apenas pois magnetometro nao funcionou corretamente
                        printf("%s      > Accel: X=%6d, Y=%6d, Z=%6d, Gyro: X=%6d, Y=%6d, Z=%6d\r\n", datetime_str_mpu, accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]); 
                    } else {
                        printf("%s, %d, %d, %d, %d, %d, %d, %d\r\n",datetime_str_mpu,new_value,accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
                    }

                    // Permanece ciclando
                    state = WAIT_DATA;
                    break;
            }
        } else {
            if(sinalizacao_final == 0){

                //Sinaliza no LED RGB a finalização
                led_level_r = 0;
                led_level_b = 0;
                led_level_g = 0;
                sleep_ms(2000);
                led_level_g = 250;
                sleep_ms(1000);
                led_level_g = 0;
                sleep_ms(1000);
                led_level_g = 250;
                sleep_ms(1000);
                led_level_g = 0;
                sleep_ms(1000);
                led_level_g = 250;
                sleep_ms(1000);
                led_level_g = 0;
                sleep_ms(2000);
                sinalizacao_final = 1;
            } else {
                // Maquina em Idle. Aquisicao terminou. Nao fazer nada.
                tight_loop_contents();
            }

        }
    }
}