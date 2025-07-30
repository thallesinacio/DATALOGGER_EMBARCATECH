#include <ctype.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "hardware/adc.h"
#include "hardware/rtc.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "lib/ssd1306.h"
#include "lib/font.h"
#include "hardware/pio.h"


#include "ff.h"
#include "diskio.h"
#include "f_util.h"
#include "hw_config.h"
#include "my_debug.h"
#include "rtc.h"
#include "sd_card.h"

#include "FreeRTOS.h"
#include "task.h"
#include "pio_matrix.pio.h"

#define ADC_PIN 26 // GPIO 26

static bool logger_enabled;
static const uint32_t period = 1000;
static absolute_time_t next_log_time;
static volatile uint32_t last_time = 0;

// Periféricos 
#define BOTAO_A 5
#define BOTAO_B 6
#define BUZZER_A 21
#define BUZZER_B 10
#define LED_VERMELHO 13
#define LED_VERDE 11
#define LED_AZUL 12

// Display
#define I2C_PORTD i2c1
#define I2C_SDAD 14
#define I2C_SCLD 15
#define endereco 0x3C
ssd1306_t ssd;

//static char filename[20] = "adc_data1.txt";
static char filename[20] = "imu_data1.csv";

static sd_card_t *sd_get_by_name(const char *const name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return sd_get_by_num(i);
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}
static FATFS *sd_get_fs_by_name(const char *name)
{
    for (size_t i = 0; i < sd_get_num(); ++i)
        if (0 == strcmp(sd_get_by_num(i)->pcName, name))
            return &sd_get_by_num(i)->fatfs;
    DBG_PRINTF("%s: unknown name %s\n", __func__, name);
    return NULL;
}

static void run_setrtc()
{
    const char *dateStr = strtok(NULL, " ");
    if (!dateStr)
    {
        printf("Missing argument\n");
        return;
    }
    int date = atoi(dateStr);

    const char *monthStr = strtok(NULL, " ");
    if (!monthStr)
    {
        printf("Missing argument\n");
        return;
    }
    int month = atoi(monthStr);

    const char *yearStr = strtok(NULL, " ");
    if (!yearStr)
    {
        printf("Missing argument\n");
        return;
    }
    int year = atoi(yearStr) + 2000;

    const char *hourStr = strtok(NULL, " ");
    if (!hourStr)
    {
        printf("Missing argument\n");
        return;
    }
    int hour = atoi(hourStr);

    const char *minStr = strtok(NULL, " ");
    if (!minStr)
    {
        printf("Missing argument\n");
        return;
    }
    int min = atoi(minStr);

    const char *secStr = strtok(NULL, " ");
    if (!secStr)
    {
        printf("Missing argument\n");
        return;
    }
    int sec = atoi(secStr);

    datetime_t t = {
        .year = (int16_t)year,
        .month = (int8_t)month,
        .day = (int8_t)date,
        .dotw = 0, // 0 is Sunday
        .hour = (int8_t)hour,
        .min = (int8_t)min,
        .sec = (int8_t)sec};
    rtc_set_datetime(&t);
}

static void run_format()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    /* Format the drive with default parameters */
    FRESULT fr = f_mkfs(arg1, 0, 0, FF_MAX_SS * 2);
    if (FR_OK != fr)
        printf("f_mkfs error: %s (%d)\n", FRESULT_str(fr), fr);
}

volatile bool tela_montagem = false;
volatile bool tela_desmontagem = false;
volatile bool tela_inicial = true;
volatile bool tela_captura = false;
volatile bool concluido = false;
volatile bool matriz_montagem = false;
volatile bool matriz_concluido = false;
volatile bool matriz_apaga = false;
volatile bool buzz_simples = false;
volatile bool buzz_duplo = false;

static void run_mount()
{
    tela_montagem = true;
    tela_captura = false;
    tela_inicial = false;
    tela_desmontagem = false;
    matriz_montagem = true;
    buzz_simples = true;
    gpio_put(LED_VERDE,true);
    gpio_put(LED_VERMELHO,true);
    

    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_mount(p_fs, arg1, 1);
    if (FR_OK != fr)
    {
        printf("f_mount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = true;
    printf("Processo de montagem do SD ( %s ) concluído\n", pSD->pcName);

    vTaskDelay(pdMS_TO_TICKS(2000));
    tela_montagem = false;
    concluido = true;
    matriz_montagem = false;
    matriz_concluido = true;
    buzz_duplo = true;
    gpio_put(LED_VERMELHO,false);
    vTaskDelay(pdMS_TO_TICKS(2000));
    matriz_concluido = false;
    matriz_apaga = true;
    gpio_put(LED_VERDE,false);
    concluido = false;
    tela_inicial = true;
    

}
static void run_unmount()
{
    tela_desmontagem = true;
    tela_montagem = false;
    tela_inicial = false;
    tela_captura = false;
    matriz_montagem = true;
    buzz_simples = true;
    gpio_put(LED_VERDE,true);
    gpio_put(LED_VERMELHO,true);

    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_unmount(arg1);
    if (FR_OK != fr)
    {
        printf("f_unmount error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    sd_card_t *pSD = sd_get_by_name(arg1);
    myASSERT(pSD);
    pSD->mounted = false;
    pSD->m_Status |= STA_NOINIT; // in case medium is removed
    printf("SD ( %s ) desmontado\n", pSD->pcName);

    vTaskDelay(pdMS_TO_TICKS(2000));
    tela_desmontagem = false;
    concluido = true;
    gpio_put(LED_VERMELHO,false);
    matriz_montagem = false;
    matriz_concluido = true;
    buzz_duplo = true;
    vTaskDelay(pdMS_TO_TICKS(2000));
    matriz_concluido = false;
    matriz_apaga = true;
    gpio_put(LED_VERDE,false);
    concluido = false;
    tela_inicial = true;
}
static void run_getfree()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = sd_get_by_num(0)->pcName;
    DWORD fre_clust, fre_sect, tot_sect;
    FATFS *p_fs = sd_get_fs_by_name(arg1);
    if (!p_fs)
    {
        printf("Unknown logical drive number: \"%s\"\n", arg1);
        return;
    }
    FRESULT fr = f_getfree(arg1, &fre_clust, &p_fs);
    if (FR_OK != fr)
    {
        printf("f_getfree error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    tot_sect = (p_fs->n_fatent - 2) * p_fs->csize;
    fre_sect = fre_clust * p_fs->csize;
    printf("%10lu KiB total drive space.\n%10lu KiB available.\n", tot_sect / 2, fre_sect / 2);
}
static void run_ls()
{
    const char *arg1 = strtok(NULL, " ");
    if (!arg1)
        arg1 = "";
    char cwdbuf[FF_LFN_BUF] = {0};
    FRESULT fr;
    char const *p_dir;
    if (arg1[0])
    {
        p_dir = arg1;
    }
    else
    {
        fr = f_getcwd(cwdbuf, sizeof cwdbuf);
        if (FR_OK != fr)
        {
            printf("f_getcwd error: %s (%d)\n", FRESULT_str(fr), fr);
            return;
        }
        p_dir = cwdbuf;
    }
    printf("Directory Listing: %s\n", p_dir);
    DIR dj;
    FILINFO fno;
    memset(&dj, 0, sizeof dj);
    memset(&fno, 0, sizeof fno);
    fr = f_findfirst(&dj, &fno, p_dir, "*");
    if (FR_OK != fr)
    {
        printf("f_findfirst error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    while (fr == FR_OK && fno.fname[0])
    {
        const char *pcWritableFile = "writable file",
                   *pcReadOnlyFile = "read only file",
                   *pcDirectory = "directory";
        const char *pcAttrib;
        if (fno.fattrib & AM_DIR)
            pcAttrib = pcDirectory;
        else if (fno.fattrib & AM_RDO)
            pcAttrib = pcReadOnlyFile;
        else
            pcAttrib = pcWritableFile;
        printf("%s [%s] [size=%llu]\n", fno.fname, pcAttrib, fno.fsize);

        fr = f_findnext(&dj, &fno);
    }
    f_closedir(&dj);
}
static void run_cat()
{
    char *arg1 = strtok(NULL, " ");
    if (!arg1)
    {
        printf("Missing argument\n");
        return;
    }
    FIL fil;
    FRESULT fr = f_open(&fil, arg1, FA_READ);
    if (FR_OK != fr)
    {
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
        return;
    }
    char buf[256];
    while (f_gets(buf, sizeof buf, &fil))
    {
        printf("%s", buf);
    }
    fr = f_close(&fil);
    if (FR_OK != fr)
        printf("f_open error: %s (%d)\n", FRESULT_str(fr), fr);
}

// Função para ler o conteúdo de um arquivo e exibir no terminal
void read_file(const char *filename)
{
    FIL file;
    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK)
    {
        printf("[ERRO] Não foi possível abrir o arquivo para leitura. Verifique se o Cartão está montado ou se o arquivo existe.\n");

        return;
    }
    char buffer[128];
    UINT br;
    printf("Conteúdo do arquivo %s:\n", filename);
    while (f_read(&file, buffer, sizeof(buffer) - 1, &br) == FR_OK && br > 0)
    {
        buffer[br] = '\0';
        printf("%s", buffer);
    }
    f_close(&file);
    printf("\nLeitura do arquivo %s concluída.\n\n", filename);
}

static void run_help()
{
    printf("\nComandos disponíveis:\n\n");
    printf("BOTÃO A - Capturar dados do IMU e salvar no arquivo\n");
    printf("BOTÃO B - Montar ou desmontar o cartão SD\n");
    printf("Digite 'c' para listar arquivos\n");
    printf("Digite 'd' para mostrar conteúdo do arquivo\n");
    printf("Digite 'e' para obter espaço livre no cartão SD\n");
    printf("Digite 'g' para formatar o cartão SD\n");
    printf("Digite 'h' para exibir os comandos disponíveis\n");
    printf("\nEscolha o comando:  ");
}

typedef void (*p_fn_t)();
typedef struct
{
    char const *const command;
    p_fn_t const function;
    char const *const help;
} cmd_def_t;

static cmd_def_t cmds[] = {
    {"setrtc", run_setrtc, "setrtc <DD> <MM> <YY> <hh> <mm> <ss>: Set Real Time Clock"},
    {"format", run_format, "format [<drive#:>]: Formata o cartão SD"},
    {"mount", run_mount, "mount [<drive#:>]: Monta o cartão SD"},
    {"unmount", run_unmount, "unmount <drive#:>: Desmonta o cartão SD"},
    {"getfree", run_getfree, "getfree [<drive#:>]: Espaço livre"},
    {"ls", run_ls, "ls: Lista arquivos"},
    {"cat", run_cat, "cat <filename>: Mostra conteúdo do arquivo"},
    {"help", run_help, "help: Mostra comandos disponíveis"}};

static void process_stdio(int cRxedChar)
{
    static char cmd[256];
    static size_t ix;

    if (!isprint(cRxedChar) && !isspace(cRxedChar) && '\r' != cRxedChar &&
        '\b' != cRxedChar && cRxedChar != (char)127)
        return;
    printf("%c", cRxedChar); // echo
    stdio_flush();
    if (cRxedChar == '\r')
    {
        printf("%c", '\n');
        stdio_flush();

        if (!strnlen(cmd, sizeof cmd))
        {
            printf("> ");
            stdio_flush();
            return;
        }
        char *cmdn = strtok(cmd, " ");
        if (cmdn)
        {
            size_t i;
            for (i = 0; i < count_of(cmds); ++i)
            {
                if (0 == strcmp(cmds[i].command, cmdn))
                {
                    (*cmds[i].function)();
                    break;
                }
            }
            if (count_of(cmds) == i)
                printf("Command \"%s\" not found\n", cmdn);
        }
        ix = 0;
        memset(cmd, 0, sizeof cmd);
        printf("\n> ");
        stdio_flush();
    }
    else
    {
        if (cRxedChar == '\b' || cRxedChar == (char)127)
        {
            if (ix > 0)
            {
                ix--;
                cmd[ix] = '\0';
            }
        }
        else
        {
            if (ix < sizeof cmd - 1)
            {
                cmd[ix] = cRxedChar;
                ix++;
            }
        }
    }
}

/*------------------------SETANDO SENSOR IMU------------------------*/

// MPU6050 I2C address
#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                  // 1 ou 3

 
// O endereço padrao deste IMU é o 0x68
static int addr = 0x68;

int16_t acceleration[3], gyro[3], temp;

 static void mpu6050_reset() {
     // Two byte reset. First byte register, second byte data
     // There are a load more options to set up the device in different ways that could be added here
     uint8_t buf[] = {0x6B, 0x80};
     i2c_write_blocking(I2C_PORT, addr, buf, 2, false);
     sleep_ms(100); // Allow device to reset and stabilize
 
     // Clear sleep mode (0x6B register, 0x00 value)
     buf[1] = 0x00;  // Clear sleep mode by writing 0x00 to the 0x6B register
     i2c_write_blocking(I2C_PORT, addr, buf, 2, false); 
     sleep_ms(10); // Allow stabilization after waking up
 }
 
 static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
     // For this particular device, we send the device the register we want to read
     // first, then subsequently read from the device. The register is auto incrementing
     // so we don't need to keep sending the register we want, just the first.
 
     uint8_t buffer[6];
 
     // Start reading acceleration registers from register 0x3B for 6 bytes
     uint8_t val = 0x3B;
     i2c_write_blocking(I2C_PORT, addr, &val, 1, true); // true to keep master control of bus
     i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);
 
     for (int i = 0; i < 3; i++) {
         accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
     }
 
     // Now gyro data from reg 0x43 for 6 bytes
     // The register is auto incrementing on each read
     val = 0x43;
     i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
     i2c_read_blocking(I2C_PORT, addr, buffer, 6, false);  // False - finished with bus
 
     for (int i = 0; i < 3; i++) {
         gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
     }
 
     // Now temperature from reg 0x41 for 2 bytes
     // The register is auto incrementing on each read
     val = 0x41;
     i2c_write_blocking(I2C_PORT, addr, &val, 1, true);
     i2c_read_blocking(I2C_PORT, addr, buffer, 2, false);  // False - finished with bus
 
     *temp = buffer[0] << 8 | buffer[1];
 }


/*-----------------------------------------------------FUNÇÕES AUXILIARES-------------------------------------------------------*/

// Inicialização do Display SSD1306
void init_ssd1306() {
   
    i2c_init(I2C_PORTD, 400 * 1000);

    
    gpio_set_function(I2C_SDAD, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCLD, GPIO_FUNC_I2C);

    
    gpio_pull_up(I2C_SDAD);
    gpio_pull_up(I2C_SCLD);


    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORTD);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

}


/*• Botão 1: Iniciar / Parar a captura de dados.
• Botão 2: Montar / Desmontar o cartão SD com segurança (para evitar corrupção de dados).*/

volatile bool flag_start_stop_captura = false;
volatile bool flag_monta_desmonta;
volatile bool sd_montado;
// Função chamada nas interrupções
static void gpio_irq_handler(uint gpio, uint32_t events) {
    
    uint32_t current_time = to_us_since_boot(get_absolute_time());
    if (current_time - last_time > 600000){
        last_time = current_time; 
        
        if (gpio == BOTAO_A) {
            flag_start_stop_captura = !flag_start_stop_captura;
            // se flag_start_stop_captura == false: não há captura
            // se flag_start_stop_captura == true: há captura
        } else if (gpio == BOTAO_B) {
            flag_monta_desmonta = true;
            // se flag_monta_desmonta == false: não faz nada
            // se flag_monta_desmonta == true: monta
        }
    }
}

// Matriz de led's
PIO pio;
uint sm;
uint32_t VALOR_LED;
unsigned char R, G, B;
#define NUM_PIXELS 25
#define OUT_PIN 7
double v[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0};
double apaga[] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
double x[] = {0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1};
double circle[] = {0.0, 0.1, 0.1, 0.1, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.1, 0.1, 0.1, 0.0};


//rotina para definição da intensidade de cores do led
uint32_t matrix_rgb(double b, double r, double g)
{
  //unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Desenha na matriz de leds em verde
void desenho_pio_green(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, r=0.0, desenho[24-i]);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

// Desenha na matriz de leds em vermelho
void desenho_pio_red(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, desenho[24-i], g = 0.0);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

void desenho_pio_yellow(double *desenho, uint32_t valor_led, PIO pio, uint sm, double r, double g, double b){

    for (int16_t i = 0; i < NUM_PIXELS; i++) {
            valor_led = matrix_rgb(b = 0.0, desenho[24-i], desenho[24-i]);
            pio_sm_put_blocking(pio, sm, valor_led);
    }
}

void init_button(uint gpio) {
    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_IN);
    gpio_pull_up(gpio);
}

// Inicialização de dispositivos de saída
void init_led(uint pino) {
    
    gpio_init(pino);
    gpio_set_dir(pino,GPIO_OUT);

}

void init_buzzer(uint gpio) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    // As outras configurações de PWM serão feitas nas funções de beep
}

// Função que gera um único beep curto
void beep_curto() {
    uint slice_num = pwm_gpio_to_slice_num(BUZZER_A);

    // Configura o PWM para uma frequência de ~2kHz
    pwm_set_clkdiv(slice_num, 100.0f); 
    pwm_set_wrap(slice_num, 625); // Frequência = 125MHz / (100 * 625) = 2000Hz = 2kHz

    // Define o duty cycle para 50% para gerar o som
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(BUZZER_A), 312); // Metade de 625
    pwm_set_enabled(slice_num, true); // Liga o som
    vTaskDelay(pdMS_TO_TICKS(100));    // Duração do beep (100ms)
    pwm_set_enabled(slice_num, false); // Desliga o som
}

// Função que gera um beep duplo
void beep_curto_duplo() {
    beep_curto();
    vTaskDelay(pdMS_TO_TICKS(100)); // Pausa entre os bipes
    beep_curto();
}


/*------------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------TASKS---------------------------------------------------*/

void vBuzzerTask () {
    init_buzzer(BUZZER_A);

        while(true) {
            if(buzz_simples == true) {
                beep_curto();            
                buzz_simples = false;
            } else if (buzz_duplo == true) {
                beep_curto_duplo();
                buzz_duplo = false;
            }
        }
}

void vMenuUart() {

    while (true)
    {
        int cRxedChar = getchar_timeout_us(0);
        if (PICO_ERROR_TIMEOUT != cRxedChar)
            process_stdio(cRxedChar);


        if (cRxedChar == 'c') // Lista diretórios e os arquivos se pressionar 'c'
        {
            printf("\nListagem de arquivos no cartão SD.\n");
            run_ls();
            printf("\nListagem concluída.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'd') // Exibe o conteúdo do arquivo se pressionar 'd'
        {
            read_file(filename);
            printf("Escolha o comando (h = help):  ");
        }
        if (cRxedChar == 'e') // Obtém o espaço livre no SD card se pressionar 'e'
        {
            printf("\nObtendo espaço livre no SD.\n\n");
            run_getfree();
            printf("\nEspaço livre obtido.\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        /*if (cRxedChar == 'f') // Captura dados do ADC e salva no arquivo se pressionar 'f'
        {
            capture_adc_data_and_save();
            printf("\nEscolha o comando (h = help):  ");
        }*/
        if (cRxedChar == 'g') // Formata o SD card se pressionar 'g'
        {
            printf("\nProcesso de formatação do SD iniciado. Aguarde...\n");
            run_format();
            printf("\nFormatação concluída.\n\n");
            printf("\nEscolha o comando (h = help):  ");
        }
        if (cRxedChar == 'h') // Exibe os comandos disponíveis se pressionar 'h'
        {
            run_help();
        }
        /*if (flag_start_stop_captura == true) // Botão A - Captura dados do IMU e salva no arquivo 
        {
            capture_imu_data_and_save();
            printf("\nEscolha o comando (h = help):  ");
            flag_start_stop_captura = false;
        }*/
        if(flag_monta_desmonta == true) { // Botão B - Monta/Desmonta cartão SD

            flag_monta_desmonta = false;

            if(!sd_montado) { 
                printf("\nMontando o SD...\n");
                run_mount();
                printf("\nEscolha o comando (h = help):  ");
                sd_montado = true;
            }
            else if(sd_montado) {
                printf("\nDesmontando o SD. Aguarde...\n");
                run_unmount();
                printf("\nEscolha o comando (h = help):  ");
                sd_montado = false;
            }
        }


        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void vDisplayTask() {

    // Inicializando Display
    init_ssd1306();

    while(true) {

        if(tela_inicial == true) {
            ssd1306_fill(&ssd, false);
            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
            ssd1306_draw_string_escala(&ssd, "A - Capturar IMU", 6, 16, 0.8);
            ssd1306_draw_string_escala(&ssd, "B - Montar/Desmontar", 6, 30, 0.8);
            ssd1306_send_data(&ssd); 
        }

        if(tela_captura == true) {
            ssd1306_fill(&ssd, false);
            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
            ssd1306_draw_string_escala(&ssd, "Capturando Dados", 6, 4, 0.9);
            ssd1306_draw_string_escala(&ssd, "Do Sensor...", 10, 20, 0.9);
            ssd1306_draw_string_escala(&ssd, "Aperte de novo", 6, 34, 0.9);
            ssd1306_draw_string_escala(&ssd, "para parar", 10, 48, 0.9);
            ssd1306_send_data(&ssd);
        }

        if(tela_montagem == true) {
            ssd1306_fill(&ssd, false);
            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
            ssd1306_draw_string_escala(&ssd, "Montando o SD...", 6, 20, 0.9);
            ssd1306_send_data(&ssd);
        }

        if(tela_desmontagem == true) {
            ssd1306_fill(&ssd, false);
            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
            ssd1306_draw_string_escala(&ssd, "Desmontando o SD...", 6, 20, 0.9);
            ssd1306_send_data(&ssd);
        }

        if(concluido == true) {
            ssd1306_fill(&ssd, false);
            ssd1306_rect(&ssd, 0, 0, 128, 64, true, false);
            ssd1306_draw_string_escala(&ssd, "Concluido!", 6, 20, 0.9);
            ssd1306_send_data(&ssd);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


void vMatrizTask () {
    // Setando matriz de leds
   double r = 0.0, b = 0.0 , g = 0.0;
   bool ok;
   ok = set_sys_clock_khz(128000, false);
   pio = pio0;

   uint offset = pio_add_program(pio, &ATIVIDADE_IMU_program);
   uint sm = pio_claim_unused_sm(pio, true);
   ATIVIDADE_IMU_program_init(pio, sm, offset, OUT_PIN);

   while(true) {

        if(matriz_montagem) { // Está montando 
            desenho_pio_yellow(circle, VALOR_LED, pio, sm, R, G, B);
        } else if(matriz_concluido){ // A montagem foi concluída com sucesso 
            desenho_pio_green(v, VALOR_LED, pio, sm, R, G, B);
        } else if(matriz_apaga) { // Apagar matriz
            desenho_pio_green(apaga, VALOR_LED, pio, sm, R, G, B);
            matriz_apaga = false;
        }
    vTaskDelay(pdMS_TO_TICKS(500));
    
   }
}

void vImuCaptureTask() {

    FIL file;
    FRESULT res;
    bool isCapturing = false; // Estado local para controlar a lógica da tarefa
    uint index = 0;
    

    while (true) {
        // VERIFICA SE O ESTADO GLOBAL (CONTROLADO PELO BOTÃO) MUDOU
        if (flag_start_stop_captura != isCapturing) {
            isCapturing = flag_start_stop_captura; // Sincroniza o estado local

            if (isCapturing) {
                // ESTADO MUDOU PARA 'INICIAR CAPTURA'
                printf("\nIniciando captura de dados do IMU...\n");
            
                // Atualiza a tela
                tela_captura = true;
                tela_inicial = false;
                matriz_montagem = true;
                buzz_simples = true;

                // Abre o arquivo para escrita
                res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
                if (res != FR_OK) {
                    printf("\n[ERRO] Não foi possível abrir o arquivo. Monte o Cartão SD.\n");
                    isCapturing = false; // Falhou, então não estamos capturando
                    flag_start_stop_captura = false; // Reseta a flag global
                    
                    // Volta para a tela inicial
                    tela_captura = false;
                    tela_inicial = true;
                }
            } else {
                // ESTADO MUDOU PARA 'PARAR CAPTURA'
                printf("\nCaptura de dados do IMU finalizada.\n");
                printf("Dados salvos em %s.\n\n", filename);
                f_close(&file); // Fecha o arquivo com segurança
                
                // Lógica de feedback na tela
                index = 0;
                tela_captura = false;
                concluido = true;
                matriz_montagem = false;
                matriz_concluido = true;
                buzz_duplo = true;
                vTaskDelay(pdMS_TO_TICKS(2000)); // Mostra "Concluído" por 2s
                concluido = false;
                tela_inicial = true;
                matriz_concluido = false;
                matriz_apaga = true;
                
            }
        }

        // SE ESTIVER NO MODO DE CAPTURA, COLETA E SALVA UM DADO
        if (isCapturing) {
            mpu6050_read_raw(acceleration, gyro, &temp);
            char buffer[128]; // Buffer maior para segurança
            index++;
            UINT bw;

                if(index == 1) {
                    f_write(&file, "amostra,accel_x_g,accel_y_g,accel_z_g,gyro_x,gyro_y,gyro_z\n", strlen("amostra,accel_x_g,accel_y_g,accel_z_g,gyro_x,gyro_y,gyro_z\n"), &bw);
                }

            sprintf(buffer, "%d,%.2f,%.2f,%.2f,%d,%d,%d\n", index, (float)acceleration[0]/16384, (float)acceleration[1]/16384, (float)acceleration[2]/16384, gyro[0], gyro[1], gyro[2]);
            
           
        
            res = f_write(&file, buffer, strlen(buffer), &bw);
            if (res != FR_OK) {
                printf("[ERRO] Falha ao escrever no arquivo. Parando a captura.\n");
                f_close(&file);
                isCapturing = false; // Para a captura
                flag_start_stop_captura = false; // Reseta a flag global
                
                // Volta para a tela inicial
                tela_captura = false;
                tela_inicial = true;
            }
        }


        // Atraso para controlar a taxa de amostragem e ceder tempo de CPU
        vTaskDelay(pdMS_TO_TICKS(100)); // Captura dados a cada 100ms (10Hz)
    }
}
/*------------------------------------------------------------------------------------------------------------*/

int main()
{

    stdio_init_all();
    sleep_ms(5000);

    time_init();

    printf("FatFS SPI example\n");
    printf("\033[2J\033[H"); // Limpa tela
    printf("\n> ");
    stdio_flush();
    //    printf("A tela foi limpa...\n");
    //    printf("Depois do Flush\n");
    run_help();

    // Inicializazndo botões
    init_button(BOTAO_A);
    init_button(BOTAO_B);

    // Inicializando Leds
    init_led(LED_AZUL);
    init_led(LED_VERDE);
    init_led(LED_VERMELHO);

    // Habilitando interrupções
    gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Make the I2C pins available to picotool
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    //printf("Antes do bi_decl...\n");
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    //printf("Antes do reset MPU...\n");
    mpu6050_reset();
    

    xTaskCreate(vMenuUart, "Menu Task", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vDisplayTask, "Display Task", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vMatrizTask, "Matriz Task", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    xTaskCreate(vImuCaptureTask, "IMU Capture Task", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 2, NULL); // Prioridade maior para garantir a captura
    xTaskCreate(vBuzzerTask, "Buzzer Task", configMINIMAL_STACK_SIZE + 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
    vTaskStartScheduler();
    
    panic_unsupported();

    return 0;
}