/* Copyright 2020-2023 Espressif Systems (Shanghai) CO LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "esp32_port.h"

#include <stdio.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/uart.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"

#define PIN_SCL (45)
#define PIN_SDA (3)

#define PI4IO_ADDR (0x20)

#define PI4IO_REG_IO_IN_0   (0x00)
#define PI4IO_REG_IO_OUT_0  (0x02)
#define PI4IO_REG_POL_INV_1 (0x05)
#define PI4IO_REG_IO_CONF_0 (0x06)
#define PI4IO_REG_PULL_EN_1 (0x47)

#define BJTS 1

#define I2C_MASTER_SCL_IO           PIN_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           PIN_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define TOUCH_I2C_PORT              I2C_MASTER_NUM
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000
#define TOUCH_I2C_TIMEOUT_MS        I2C_MASTER_TIMEOUT_MS

static const char *TAG = "i2c-simple-example";

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static uint16_t iox_output_state = 0xFFFF;

static void iox_pin_write(uint8_t pin, uint8_t state)
{
    iox_output_state = state ? iox_output_state | (uint16_t)(0x1 << pin): iox_output_state & ~((uint16_t)(0x1 << pin));
    const uint8_t reg_io_lo = (uint8_t)(iox_output_state & 0xFF);
    const uint8_t reg_io_hi = (uint8_t)((iox_output_state & 0xFF00) >> 8);
    uint8_t buf_out[3] = {PI4IO_REG_IO_OUT_0, reg_io_lo, reg_io_hi};
    ESP_LOGI(TAG, "Writing buf_out values %02x %02x %02x", buf_out[0], buf_out[1], buf_out[2]);
    esp_err_t ret = i2c_master_write_to_device(TOUCH_I2C_PORT, PI4IO_ADDR, 
                                                 buf_out, sizeof(buf_out), 
                                                 pdMS_TO_TICKS(TOUCH_I2C_TIMEOUT_MS));
    if (ret != ESP_OK) ESP_LOGE(TAG, "Error writing IO expander");
}

static void init_i2c(uint8_t reset_pin, uint8_t boot0_pin){
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Configure IO expander
    uint8_t buf[2] = {PI4IO_REG_PULL_EN_1, 0x1E}; // pull up button pins
    esp_err_t ret = i2c_master_write_to_device(TOUCH_I2C_PORT, PI4IO_ADDR, 
                                                 buf, sizeof(buf), 
                                                 pdMS_TO_TICKS(TOUCH_I2C_TIMEOUT_MS));

    uint8_t buf2[2] = {PI4IO_REG_POL_INV_1, 0x1E}; // invert button inputs
    ret = i2c_master_write_to_device(TOUCH_I2C_PORT, PI4IO_ADDR, 
                                                 buf2, sizeof(buf2), 
                                                 pdMS_TO_TICKS(TOUCH_I2C_TIMEOUT_MS));

    // ESP_LOGI(TAG, "pins %d %d", 0x01 << reset_pin, boot0_pin);
    const uint8_t reg_io_lo = 0xFF & ~((uint8_t)(0x1 << reset_pin)) & ~((uint8_t)(0x1 << boot0_pin));
    const uint8_t reg_io_hi = 0xFF & ~((uint8_t)(0x1 << (reset_pin-8))) & ~((uint8_t)(0x1 << (boot0_pin-8)));

    ESP_LOGI(TAG, "Writing config values %02x %02x", reg_io_lo, reg_io_hi);

    uint8_t buf21[3] = {PI4IO_REG_IO_CONF_0, reg_io_lo, reg_io_hi}; // set all pins to inputs, except for network PCBA reset and BOOT0
    ESP_LOGI(TAG, "Writing buf21 values %02x %02x %02x", buf21[0], buf21[1], buf21[2]);
    ret = i2c_master_write_to_device(TOUCH_I2C_PORT, PI4IO_ADDR, 
                                                 buf21, sizeof(buf21), 
                                                 pdMS_TO_TICKS(TOUCH_I2C_TIMEOUT_MS));

    if (ret == ESP_OK){
        ESP_LOGI(TAG, "Configured IO expander");
    } else {
        ESP_LOGE(TAG, "Error configuring IO expander");
    }     
}

#if SERIAL_FLASHER_DEBUG_TRACE
static void transfer_debug_print(const uint8_t *data, uint16_t size, bool write)
{
    static bool write_prev = false;

    if (write_prev != write) {
        write_prev = write;
        printf("\n--- %s ---\n", write ? "WRITE" : "READ");
    }

    for (uint32_t i = 0; i < size; i++) {
        printf("%02x ", data[i]);
    }
}
#endif

static int64_t s_time_end;
static int32_t s_uart_port;
static int32_t s_reset_trigger_pin;
static int32_t s_gpio0_trigger_pin;
static bool s_peripheral_needs_deinit;

esp_loader_error_t loader_port_esp32_init(const loader_esp32_config_t *config)
{
    s_uart_port = config->uart_port;
    s_reset_trigger_pin = config->reset_trigger_pin;
    s_gpio0_trigger_pin = config->gpio0_trigger_pin;

    init_i2c((uint8_t) s_reset_trigger_pin, (uint8_t) s_gpio0_trigger_pin);

    // Initialize UART
    if (!config->dont_initialize_peripheral) {
        uart_config_t uart_config = {
            .baud_rate = config->baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
            .source_clk = UART_SCLK_DEFAULT,
#endif
        };

        int rx_buffer_size = config->rx_buffer_size ? config->rx_buffer_size : 400;
        int tx_buffer_size = config->tx_buffer_size ? config->tx_buffer_size : 400;
        QueueHandle_t *uart_queue = config->uart_queue ? config->uart_queue : NULL;
        int queue_size = config->queue_size ? config->queue_size : 0;

        if ( uart_param_config(s_uart_port, &uart_config) != ESP_OK ) {
            return ESP_LOADER_ERROR_FAIL;
        }
        if ( uart_set_pin(s_uart_port, config->uart_tx_pin, config->uart_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK ) {
            return ESP_LOADER_ERROR_FAIL;
        }
        if ( uart_driver_install(s_uart_port, rx_buffer_size, tx_buffer_size, queue_size, uart_queue, 0) != ESP_OK ) {
            return ESP_LOADER_ERROR_FAIL;
        }

        s_peripheral_needs_deinit = true;
    }

    return ESP_LOADER_SUCCESS;
}

void loader_port_esp32_deinit(void)
{
    if (s_peripheral_needs_deinit) {
        uart_driver_delete(s_uart_port);
    }
}


esp_loader_error_t loader_port_write(const uint8_t *data, uint16_t size, uint32_t timeout)
{
    uart_write_bytes(s_uart_port, (const char *)data, size);
    esp_err_t err = uart_wait_tx_done(s_uart_port, pdMS_TO_TICKS(timeout));

    if (err == ESP_OK) {
#if SERIAL_FLASHER_DEBUG_TRACE
        transfer_debug_print(data, size, true);
#endif
        return ESP_LOADER_SUCCESS;
    } else if (err == ESP_ERR_TIMEOUT) {
        return ESP_LOADER_ERROR_TIMEOUT;
    } else {
        return ESP_LOADER_ERROR_FAIL;
    }
}


esp_loader_error_t loader_port_read(uint8_t *data, uint16_t size, uint32_t timeout)
{
    int read = uart_read_bytes(s_uart_port, data, size, pdMS_TO_TICKS(timeout));

    if (read < 0) {
        return ESP_LOADER_ERROR_FAIL;
    } else if (read < size) {
#if SERIAL_FLASHER_DEBUG_TRACE
        transfer_debug_print(data, read, false);
#endif
        return ESP_LOADER_ERROR_TIMEOUT;
    } else {
#if SERIAL_FLASHER_DEBUG_TRACE
        transfer_debug_print(data, read, false);
#endif
        return ESP_LOADER_SUCCESS;
    }
}


// Set GPIO0 LOW, then
// assert reset pin for 50 milliseconds.
void loader_port_enter_bootloader(void)
{
    if(!BJTS){
        iox_pin_write(s_gpio0_trigger_pin, 0);
        loader_port_reset_target();
        loader_port_delay_ms(SERIAL_FLASHER_BOOT_HOLD_TIME_MS);
        iox_pin_write(s_gpio0_trigger_pin, 1);
    }else{
        loader_port_reset_target();
        iox_pin_write(s_gpio0_trigger_pin, 0);
        loader_port_delay_ms(SERIAL_FLASHER_BOOT_HOLD_TIME_MS);
        iox_pin_write(s_gpio0_trigger_pin, 1);
    }

}

void loader_port_reset_target(void)
{
    iox_pin_write(s_reset_trigger_pin, 0);
    loader_port_delay_ms(SERIAL_FLASHER_RESET_HOLD_TIME_MS);
    iox_pin_write(s_reset_trigger_pin, 1);
}


void loader_port_delay_ms(uint32_t ms)
{
    usleep(ms * 1000);
}


void loader_port_start_timer(uint32_t ms)
{
    s_time_end = esp_timer_get_time() + ms * 1000;
}


uint32_t loader_port_remaining_time(void)
{
    int64_t remaining = (s_time_end - esp_timer_get_time()) / 1000;
    return (remaining > 0) ? (uint32_t)remaining : 0;
}


void loader_port_debug_print(const char *str)
{
    printf("DEBUG: %s\n", str);
}

esp_loader_error_t loader_port_change_transmission_rate(uint32_t baudrate)
{
    esp_err_t err = uart_set_baudrate(s_uart_port, baudrate);
    return (err == ESP_OK) ? ESP_LOADER_SUCCESS : ESP_LOADER_ERROR_FAIL;
}
