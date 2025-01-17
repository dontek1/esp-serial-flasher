/* Flash multiple partitions example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sys/param.h>
#include <string.h>
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp32_port.h"
#include "esp_loader.h"
#include "example_common.h"
#include "led_strip.h"

static const char *TAG = "serial_flasher";

//#define ENABLE_UART_CHECK

static loader_esp32_config_t config =
{
    .baud_rate = 115200
};

static uint32_t higher_baudrate = 115200;

static led_strip_handle_t led_strip;

static uint8_t led_r = 0x10;
static uint8_t led_g = 0x10;
static uint8_t led_b = 0x10;

static bool led_state = false;
static bool led_toggle = false;
static uint32_t led_intv_ms = 1000;

static void led_configure(void)
{
    led_strip_config_t strip_config =
    {
        .strip_gpio_num = GPIO_NUM_38,
        .max_leds = 1,
    };
    led_strip_rmt_config_t rmt_config =
    {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    led_strip_clear(led_strip);
}

static void led_update(uint8_t r, uint8_t g, uint8_t b)
{
    led_r = r;
    led_g = g;
    led_b = b;
}

#define LED_SET_ESP1()      led_update(0x00, 0x00, 0x80)
#define LED_SET_ESP2()      led_update(0xFF, 0x45, 0x00)
#define LED_SET_ERROR()     led_update(0xFF, 0x00, 0x00)

void led_blink(void *arg)
{
    while(1)
    {
        if(led_state)
        {
            led_strip_set_pixel(led_strip, 0, led_r, led_g, led_b);
            led_strip_refresh(led_strip);
        }
        else
        {
            led_strip_clear(led_strip);
        }
        if(led_toggle)
        {
            led_state = !led_state;
        }
        vTaskDelay(pdMS_TO_TICKS(led_intv_ms));
    }
}

// Max line size
#define BUF_LEN 128
static uint8_t buf[BUF_LEN] = {0};

void slave_monitor(void *arg)
{
    if(higher_baudrate != 115200)
    {
        uart_flush_input(config.uart_port);
        uart_flush(config.uart_port);
        uart_set_baudrate(config.uart_port, 115200);
    }

    while(1)
    {
        int rxBytes = uart_read_bytes(config.uart_port, buf, BUF_LEN, 100 / portTICK_PERIOD_MS);
        buf[rxBytes] = '\0';
        printf("%s", buf);
    }
}

#define GPIO_INPUT_ESPn     GPIO_NUM_14

#define GPIO_INPUT_BAUD1    GPIO_NUM_45
#define GPIO_INPUT_BAUD2    GPIO_NUM_48
#define GPIO_INPUT_BAUD3    GPIO_NUM_47
#define GPIO_INPUT_BAUD4    GPIO_NUM_21

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_ESPn)|(1ULL<<GPIO_INPUT_BAUD1)|(1ULL<<GPIO_INPUT_BAUD2)|(1ULL<<GPIO_INPUT_BAUD3)|(1ULL<<GPIO_INPUT_BAUD4))

static void gpio_setup(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

static int gpio_select_esp(void)
{
    // GND --- IN.ESPn => select ESP
    int inEsp = gpio_get_level(GPIO_INPUT_ESPn);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_ESPn, inEsp);
    return inEsp;
}

static void gpio_select_baud(void)
{
    // GND -- IN.Baud1/IN.Baud2/IN.Baud3 => select baud
    int inBaud1 = gpio_get_level(GPIO_INPUT_BAUD1);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_BAUD1, inBaud1);

    int inBaud2 = gpio_get_level(GPIO_INPUT_BAUD2);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_BAUD2, inBaud2);

    int inBaud3 = gpio_get_level(GPIO_INPUT_BAUD3);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_BAUD3, inBaud3);

    int inBaud4 = gpio_get_level(GPIO_INPUT_BAUD4);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_BAUD4, inBaud4);
    
    if(inBaud4 == 0)
    {
        higher_baudrate = 3686400;
        led_intv_ms = 25;
    }
    else if(inBaud3 == 0)
    {
        higher_baudrate = 921600;
        led_intv_ms = 125;
    }
    else if(inBaud2 == 0)
    {
        higher_baudrate = 460800;
        led_intv_ms = 250;
    }
    else if(inBaud1 == 0)
    {
        higher_baudrate = 230400;
        led_intv_ms = 500;
    }
    else
    {
        higher_baudrate = 115200;
        led_intv_ms = 1000;
    }
}

#ifdef ENABLE_UART_CHECK
static void uart_check(uint32_t uart_port, uint32_t tx_pin, uint32_t rx_pin)
{
    uart_config_t uart_config =
    {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_port, BUF_LEN * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%lu opened, baud %dbps", uart_port, 115200);

    for(int i = 0; i < 30; i++)
    {
        // Read data from the UART
        int len = uart_read_bytes(uart_port, buf, (BUF_LEN - 1), 20 / portTICK_PERIOD_MS);
        if(len)
        {
            buf[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *) buf);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    ESP_ERROR_CHECK(uart_driver_delete(uart_port));
}
#endif

#ifdef SINGLE_TARGET_SUPPORT

// For esp32s3 and later chips
#define BOOTLOADER_ADDRESS_V1       0x0
#define PARTITION_ADDRESS           0x8000
#define APPLICATION_ADDRESS         0x10000

extern const unsigned char bootloader_start[] asm("_binary_bootloader_bin_start");
extern const unsigned char bootloader_end[]   asm("_binary_bootloader_bin_end");

extern const unsigned char partition_table_start[] asm("_binary_partition_table_bin_start");
extern const unsigned char partition_table_end[]   asm("_binary_partition_table_bin_end");

extern const unsigned char https_mbedtls_start[] asm("_binary_https_mbedtls_bin_start");
extern const unsigned char https_mbedtls_end[]   asm("_binary_https_mbedtls_bin_end");

void get_example_binaries_esp32s3(example_binaries_t *bins)
{
    bins->boot.data = (const uint8_t *) &bootloader_start;
    bins->boot.size = bootloader_end - bootloader_start;
    bins->boot.addr = BOOTLOADER_ADDRESS_V1;

    bins->part.data = (const uint8_t *) &partition_table_start;
    bins->part.size = partition_table_end - partition_table_start;
    bins->part.addr = PARTITION_ADDRESS;

    bins->app.data  = (const uint8_t *) &https_mbedtls_start;
    bins->app.size  = https_mbedtls_end - https_mbedtls_start;
    bins->app.addr  = APPLICATION_ADDRESS;
}

#endif

static bool do_flash(const char *name, const uint8_t *bin, size_t size, size_t address)
{
    bool result = false;

    ESP_LOGI(TAG, "Loading %s from %p to 0x%08x...", name, bin, address);
    ESP_LOGI(TAG, "Baud: %lu", higher_baudrate);
    ESP_LOGI(TAG, "Size: %d bytes", size);

    int64_t t0_us = esp_timer_get_time();
    esp_loader_error_t err = flash_binary(bin, size, address);
    int64_t t1_us = esp_timer_get_time();
    
    if(err == ESP_LOADER_SUCCESS)
    {
        int64_t tm_us = t1_us - t0_us;    
        double tm_sec = (double) (tm_us / 1000000.0);
        double size_kb = (double) (size / 1024.0);
        double kb_s = size_kb / tm_sec;

        ESP_LOGI(TAG, "Time: %.1f secs", tm_sec);
        ESP_LOGI(TAG, "Rate: %.1f kB/s\n", kb_s);
        result = true;
    }

    return result;
}

void app_main(void)
{
    led_configure();
    xTaskCreate(led_blink, "led_blink", 2048, NULL, configMAX_PRIORITIES - 1, NULL);

#ifdef ENABLE_UART_CHECK
    uart_check(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18);
#endif

    gpio_setup();

    if(gpio_select_esp() == 1)
    {
        config.uart_port = UART_NUM_1;
        config.uart_rx_pin = GPIO_NUM_11;
        config.uart_tx_pin = GPIO_NUM_12;
        config.reset_trigger_pin = GPIO_NUM_6;
        config.gpio0_trigger_pin = GPIO_NUM_7;
        LED_SET_ESP1();
    }
    else
    {
        config.uart_port = UART_NUM_2;
        config.uart_rx_pin = GPIO_NUM_17;
        config.uart_tx_pin = GPIO_NUM_18;
        config.reset_trigger_pin = GPIO_NUM_4;
        config.gpio0_trigger_pin = GPIO_NUM_5;
        LED_SET_ESP2();
    }

    if(loader_port_esp32_init(&config) != ESP_LOADER_SUCCESS)
    {
        ESP_LOGE(TAG, "serial initialization failed.");
        LED_SET_ERROR();
        return;
    }

    gpio_select_baud();

    ESP_LOGI(TAG, "Connecting on UART%lu", config.uart_port);

    bool result = (connect_to_target(higher_baudrate) == ESP_LOADER_SUCCESS);
    if(result)
    {
        example_binaries_t bin;

#ifdef SINGLE_TARGET_SUPPORT        
        get_example_binaries_esp32s3(&bin);
#else
        get_example_binaries(esp_loader_get_target(), &bin);
#endif
        
        led_toggle = true;

        result &= do_flash("bootloader", bin.boot.data, bin.boot.size, bin.boot.addr);
        result &= do_flash("partition table", bin.part.data, bin.part.size, bin.part.addr);
        result &= do_flash("app", bin.app.data,  bin.app.size,  bin.app.addr);
        
        ESP_LOGI(TAG, "Done!");

        esp_loader_reset_target();

        // Delay for skipping the boot message of the targets
        vTaskDelay(pdMS_TO_TICKS(500));

        // Forward slave's serial output
        ESP_LOGI(TAG, "********************************************");
        ESP_LOGI(TAG, "*** Logs below are print from slave .... ***");
        ESP_LOGI(TAG, "********************************************");
        xTaskCreate(slave_monitor, "slave_monitor", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    }

    led_toggle = false;
    led_state = true;

    if(!result)
    {
        LED_SET_ERROR();
    }

    vTaskDelete(NULL);
}
