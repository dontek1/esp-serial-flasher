/* Flash multiple partitions example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <sys/param.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "esp32_port.h"
#include "esp_loader.h"

#include "esp_app_format.h"
#include "esp_ota_ops.h"

#include "esp_flash.h"

#include "esp_http_client.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "example_common.h"

#include "led_strip.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "protocol_examples_common.h"

static const char *TAG = "serial_flasher";

//#define ENABLE_UART_CHECK

// Flashing bootloader/partition table from HTTP server not supported/TBD.
#ifndef ENABLE_HTTP_DOWNLOAD
#define ENABLE_FLASH_BOOTLOADER
#define ENABLE_FLASH_PARTTABLE
#endif

static loader_esp32_config_t config =
{
    .baud_rate = 115200
};

static uint32_t higher_baudrate = 115200;

static led_strip_handle_t led_strip;

static uint8_t R = 0x10;
static uint8_t G = 0x10;
static uint8_t B = 0x10;

enum blink_rate_e
{
    BLINK_RATE_DEF=0,
    BLINK_RATE_368M,
    BLINK_RATE_921K,
    BLINK_RATE_460K,
    BLINK_RATE_230K,
    BLINK_RATE_115K,
    BLINK_RATE_ERR
};

static uint8_t blink_rate = BLINK_RATE_DEF;

#define LED_SET_ESP1()          do { R=0x00; G=0x00; B=0x80; } while(0)
#define LED_SET_ESP2()          do { R=0xFF; G=0x45; B=0x00; } while(0)

#define LED_SET_CONNECTED()     do { R=0x00; G=0x80; B=0x00; } while(0)
#define LED_SET_ERROR(r,g,b)    do { R=r; G=g; B=b; blink_rate=BLINK_RATE_ERR; } while(0)

#define LED_FILE_ERROR()        LED_SET_ERROR(0x80,0x80,0x00)
#define LED_TARGET_ERROR()      LED_SET_ERROR(0x80,0x00,0x80)
#define LED_FLASH_ERROR()       LED_SET_ERROR(0x80,0x00,0x00)

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

void led_blink(void *arg)
{
    bool led_state = true;
    uint32_t led_intv_ms = 1000;

    while(1)
    {
        if(led_state)
        {
            led_strip_set_pixel(led_strip, 0, R, G, B);
            led_strip_refresh(led_strip);
            led_state = false;
        }
        else
        {
            led_strip_clear(led_strip);
            led_state = true;
        }

        switch(blink_rate)
        {
            case BLINK_RATE_368M: led_intv_ms = 25; break;
            case BLINK_RATE_921K: led_intv_ms = 125; break;
            case BLINK_RATE_460K: led_intv_ms = 250; break;
            case BLINK_RATE_230K: led_intv_ms = 500; break;
            case BLINK_RATE_115K: led_intv_ms = 1000; break;
            case BLINK_RATE_ERR: led_intv_ms = 5000; break;
            default: led_intv_ms = 2000; break; // BLINK_RATE_DEF
        }

        vTaskDelay(pdMS_TO_TICKS(led_intv_ms));
    }
}

// Max line size
#define SLV_BUF_LEN 128
static uint8_t slv_buf[SLV_BUF_LEN] = {0};

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
        int rxBytes = uart_read_bytes(config.uart_port, slv_buf, SLV_BUF_LEN, 100 / portTICK_PERIOD_MS);
        slv_buf[rxBytes] = '\0';
        printf("%s", slv_buf);
    }
}

#define GPIO_INPUT_ESPn     GPIO_NUM_14

#define GPIO_INPUT_BAUD1    GPIO_NUM_45
#define GPIO_INPUT_BAUD2    GPIO_NUM_48
#define GPIO_INPUT_BAUD3    GPIO_NUM_47
#define GPIO_INPUT_BAUD4    GPIO_NUM_21

#define GPIO_INPUT_URL      GPIO_NUM_1

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_ESPn)\
                            |(1ULL<<GPIO_INPUT_BAUD1)\
                            |(1ULL<<GPIO_INPUT_BAUD2)\
                            |(1ULL<<GPIO_INPUT_BAUD3)\
                            |(1ULL<<GPIO_INPUT_BAUD4)\
                            |(1ULL<<GPIO_INPUT_URL))

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

static void gpio_select_esp(void)
{
    // GND --- IN.ESPn => select ESP
    int inEsp = gpio_get_level(GPIO_INPUT_ESPn);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_ESPn, inEsp);

    if(inEsp == 1)
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
}

static void gpio_select_baud(void)
{
    // GND --- IN.Baud1-4 => select baud
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
        blink_rate = BLINK_RATE_368M;
    }
    else if(inBaud3 == 0)
    {
        higher_baudrate = 921600;
        blink_rate = BLINK_RATE_921K;
    }
    else if(inBaud2 == 0)
    {
        higher_baudrate = 460800;
        blink_rate = BLINK_RATE_460K;
    }
    else if(inBaud1 == 0)
    {
        higher_baudrate = 230400;
        blink_rate = BLINK_RATE_230K;
    }
    else
    {
        higher_baudrate = 115200;
        blink_rate = BLINK_RATE_115K;
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

    ESP_ERROR_CHECK(uart_driver_install(uart_port, SLV_BUF_LEN * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART%lu opened, baud %dbps", uart_port, 115200);

    for(int i = 0; i < 30; i++)
    {
        // Read data from the UART
        int len = uart_read_bytes(uart_port, slv_buf, (SLV_BUF_LEN - 1), 20 / portTICK_PERIOD_MS);
        if(len)
        {
            slv_buf[len] = '\0';
            ESP_LOGI(TAG, "Recv str: %s", (char *) slv_buf);
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

void init_binaries_esp32s3(example_binaries_t *bins)
{
    bins->boot.data = NULL;
    bins->boot.size = 0;
    bins->boot.addr = BOOTLOADER_ADDRESS_V1;

    bins->part.data = NULL;
    bins->part.size = 0;
    bins->part.addr = PARTITION_ADDRESS;

    bins->app.data  = NULL;
    bins->app.size  = 0;
    bins->app.addr  = APPLICATION_ADDRESS;
}

#ifdef ENABLE_HTTP_DOWNLOAD

#define DOWNLOAD_ADDRESS    (0x00281000)
#define DOWNLOAD_PROGRESS   (10)

esp_partition_mmap_handle_t download_map_handle;
const void *download_map_ptr;
bool download_mapped = false;

#define DLD_BUF_SIZE 8192
static char download_buf[DLD_BUF_SIZE+1] = {0};

static void http_cleanup(esp_http_client_handle_t client)
{
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
}

char *http_url = NULL;
static void gpio_select_url(void)
{
    // GND --- IN.URL => select url
    int inUrl = gpio_get_level(GPIO_INPUT_URL);
    ESP_LOGI(TAG, "Get GPIO%d = %d", GPIO_INPUT_URL, inUrl);

    if(inUrl == 1)
    {
        http_url = "http://example.com/file1.bin";
    }
    else
    {
        http_url = "http://example.com/file2.bin";
    }
}

bool get_binaries_esp32s3(example_binaries_t *bins)
{
    bool result = false;

    init_binaries_esp32s3(bins);    
    gpio_select_url();

    int64_t t0_us = esp_timer_get_time();

    esp_http_client_config_t config =
    {
        .url = http_url,
        .cert_pem = NULL,
        .timeout_ms = 3000,
        .keep_alive_enable = true,
        .skip_cert_common_name_check = true
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if(client == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialise HTTP connection");
        return result;
    }
    ESP_LOGI(TAG, "HTTP connection initialised");

    esp_err_t err = esp_http_client_open(client, 0);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open HTTP connection: %s", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return result;
    }
    ESP_LOGI(TAG, "HTTP connection opened, URL: %s", config.url);

    int64_t cont_len = esp_http_client_fetch_headers(client);
    if(cont_len > 0)
    {
        ESP_LOGI(TAG, "content-length is %lld", cont_len);
    }

    int bin_len = 0;
    int pct_read = 0;
    int pct_old = -1;    

    bool image_header_was_checked = false;
    
    uint32_t write_address = DOWNLOAD_ADDRESS;

    while(1)
    {
        int data_read = esp_http_client_read(client, download_buf, DLD_BUF_SIZE);
        if(data_read < 0)
        {
            ESP_LOGE(TAG, "Error: SSL data read error");
            http_cleanup(client);
            break;
        }
        else if(data_read > 0)
        {
            if(image_header_was_checked == false)
            {
                esp_app_desc_t new_app_info;
                if(data_read > sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t))
                {
                    memcpy(&new_app_info, &download_buf[sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t)], sizeof(esp_app_desc_t));
                    if(new_app_info.magic_word == ESP_APP_DESC_MAGIC_WORD)
                    {
                        ESP_LOGI(TAG, "Firmware info");
                        ESP_LOGI(TAG, "- project name: %s", new_app_info.project_name);
                        ESP_LOGI(TAG, "- firmware version: %s", new_app_info.version);
                        ESP_LOGI(TAG, "- IDF version: %s", new_app_info.idf_ver);
                        ESP_LOGI(TAG, "- compiled date/time: %s %s", new_app_info.date, new_app_info.time);
                        
                        image_header_was_checked = true;
                    }
                    else
                    {
                        ESP_LOGE(TAG, "Invalid magic word: %lx", new_app_info.magic_word);
                        http_cleanup(client);
                        break;
                    }
                }
                else
                {
                    ESP_LOGE(TAG, "Received package is not fit len");
                    http_cleanup(client);
                    break;
                }
            }
            
	        err = esp_flash_erase_region(NULL, write_address, DLD_BUF_SIZE);
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error erasing flash address at 0x%lu, size=%d", write_address, DLD_BUF_SIZE);
                break;
            }

            err = esp_flash_write(NULL, (void *) download_buf, write_address, data_read);
            if(err != ESP_OK)
            {
                ESP_LOGE(TAG, "Error writing %d bytes to flash address 0x%lu", data_read, write_address);
                break;
            }

            write_address += data_read;
            bin_len += data_read;
            ESP_LOGD(TAG, "Written image length %d", bin_len);

            if(cont_len > 0)
            {
                pct_read = (bin_len * 100) / cont_len;
                pct_read = pct_read - (pct_read % DOWNLOAD_PROGRESS);

                if(pct_read != pct_old)
                {
                    ESP_LOGI(TAG, "Downloading & writing to flash %d%%", pct_read);
                    pct_old = pct_read;
                }
            }
        }
        else if(data_read == 0)
        {
           /*
            * As esp_http_client_read never returns negative error code, we rely on
            * `errno` to check for underlying transport connectivity closure if any
            */
            if(errno == ECONNRESET || errno == ENOTCONN)
            {
                ESP_LOGE(TAG, "Connection closed, errno = %d", errno);
                break;
            }
            if(esp_http_client_is_complete_data_received(client) == true)
            {
                ESP_LOGI(TAG, "Connection closed");
                break;
            }
        }
    }

    result = esp_http_client_is_complete_data_received(client);
    if(result)
    {
        if(bin_len > 0)
        {
            int64_t t1_us = esp_timer_get_time();
            int64_t tm_us = t1_us - t0_us;    
            double tm_sec = (double) (tm_us / 1000000.0);
            ESP_LOGI(TAG, "Downloaded & wrote %d bytes to flash in %.1f secs", bin_len, tm_sec);

            const esp_partition_t *partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "download");
            if(partition != NULL)
            {
                err = esp_partition_mmap(partition, 0, partition->size, ESP_PARTITION_MMAP_DATA, &download_map_ptr, &download_map_handle);
                if(err == ESP_OK)
                {
                    ESP_LOGI(TAG, "'download' partition mapped to data memory address %p", download_map_ptr);
                    bins->app.data = (const uint8_t *) download_map_ptr;
                    bins->app.size = bin_len;
                    download_mapped = true;
                }
                else
                {
                    ESP_LOGE(TAG, "esp_partition_mmap returned %d", err);
                    result = false;
                }
            }
            else
            {
                ESP_LOGE(TAG, "'download' partition is NULL");
                result = false;
            }
        }
        else
        {
            ESP_LOGW(TAG, "No data downloaded/flashed");
        }
    }
    else
    {
        ESP_LOGE(TAG, "Error in receiving complete file");
    }

    return result;
}

#else // !ENABLE_HTTP_DOWNLOAD

extern const unsigned char bootloader_start[] asm("_binary_bootloader_bin_start");
extern const unsigned char bootloader_end[]   asm("_binary_bootloader_bin_end");

extern const unsigned char partition_table_start[] asm("_binary_partition_table_bin_start");
extern const unsigned char partition_table_end[]   asm("_binary_partition_table_bin_end");

extern const unsigned char https_mbedtls_start[] asm("_binary_https_mbedtls_bin_start");
extern const unsigned char https_mbedtls_end[]   asm("_binary_https_mbedtls_bin_end");

bool get_binaries_esp32s3(example_binaries_t *bins)
{
    init_binaries_esp32s3(bins);
    
    bins->boot.data = (const uint8_t *) &bootloader_start;
    bins->boot.size = bootloader_end - bootloader_start;

    bins->part.data = (const uint8_t *) &partition_table_start;
    bins->part.size = partition_table_end - partition_table_start;

    bins->app.data  = (const uint8_t *) &https_mbedtls_start;
    bins->app.size  = https_mbedtls_end - https_mbedtls_start;

    return true;
}

#endif // ENABLE_HTTP_DOWNLOAD

#endif // SINGLE_TARGET_SUPPORT

static bool do_flash(const char *name, const uint8_t *bin, size_t size, size_t address)
{
    bool result = false;

    if(bin == NULL)
    {
        ESP_LOGE(TAG, "Invalid bin data for %s", name);
    }
    else if(size == 0)
    {
        ESP_LOGE(TAG, "Invalid bin size for %s", name);
    }
    else
    {
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
        else
        {
            ESP_LOGE(TAG, "Error flashing %s: %d", name, err);
        }
    }

    return result;
}

void app_main(void)
{
    bool result = false;

    gpio_setup();

    led_configure();
    xTaskCreate(led_blink, "led_blink", 1024, NULL, configMAX_PRIORITIES - 1, NULL);

#ifdef ENABLE_HTTP_DOWNLOAD
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // OTA app partition table has a smaller NVS partition size than the non-OTA
        // partition table. This size mismatch may cause NVS initialization to fail.
        // If this happens, we erase NVS partition and initialize NVS again.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    LED_SET_CONNECTED();
#endif

    example_binaries_t bin;
#ifdef SINGLE_TARGET_SUPPORT        
    result = get_binaries_esp32s3(&bin);
    if(!result)
    {
        ESP_LOGE(TAG, "error getting binaries.");
        LED_FILE_ERROR();
        return;
    }    
#else
    get_example_binaries(esp_loader_get_target(), &bin);
#endif

    gpio_select_esp();

#ifdef ENABLE_UART_CHECK
    uart_check(UART_NUM_1, GPIO_NUM_17, GPIO_NUM_18);
#endif

    if(loader_port_esp32_init(&config) != ESP_LOADER_SUCCESS)
    {
        ESP_LOGE(TAG, "serial initialization failed.");
        LED_TARGET_ERROR();
        return;
    }

    gpio_select_baud();

    ESP_LOGI(TAG, "Connecting on UART%lu", config.uart_port);

    result = (connect_to_target(higher_baudrate) == ESP_LOADER_SUCCESS);
    if(result)
    {
#ifdef ENABLE_FLASH_BOOTLOADER
        result &= do_flash("bootloader", bin.boot.data, bin.boot.size, bin.boot.addr);
#endif

#ifdef ENABLE_FLASH_PARTTABLE
        result &= do_flash("partition table", bin.part.data, bin.part.size, bin.part.addr);
#endif

        result &= do_flash("app", bin.app.data,  bin.app.size,  bin.app.addr);
        if(result)
        {
            ESP_LOGI(TAG, "Success!");
            blink_rate = BLINK_RATE_DEF;
        }
        else
        {
            LED_FLASH_ERROR();
        }

#ifdef ENABLE_HTTP_DOWNLOAD
        if(download_mapped)
        {
            esp_partition_munmap(download_map_handle);
        }
#endif

        esp_loader_reset_target();

        // Delay for skipping the boot message of the targets
        vTaskDelay(pdMS_TO_TICKS(500));

        // Forward slave's serial output
        ESP_LOGI(TAG, "********************************************");
        ESP_LOGI(TAG, "*** Logs below are print from slave .... ***");
        ESP_LOGI(TAG, "********************************************");
        xTaskCreate(slave_monitor, "slave_monitor", 2048, NULL, configMAX_PRIORITIES - 1, NULL);
    }
    else
    {
        LED_TARGET_ERROR();
    }

    vTaskDelete(NULL);
}
