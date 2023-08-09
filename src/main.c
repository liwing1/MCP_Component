#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "MCP3564.h"

MCP356x_t MCP_instance = {
    .gpio_num_miso = GPIO_NUM_13,
    .gpio_num_mosi = GPIO_NUM_11,
    .gpio_num_clk = GPIO_NUM_12,
    .gpio_num_cs = GPIO_NUM_10,
    .gpio_num_pwm = GPIO_NUM_18,
    .gpio_num_nsync = GPIO_NUM_16,
    .gpio_num_ndrdy = GPIO_NUM_1,
    .flag_drdy = 0,
};

void app_main() 
{
    mcpStartup(&MCP_instance);

    ESP_ERROR_CHECK(MCP3564_startConversion(&MCP_instance, 30));
    vTaskDelay(pdMS_TO_TICKS(31000));
    vTaskDelay(pdMS_TO_TICKS(31000));
    ESP_ERROR_CHECK(MCP3564_startConversion(&MCP_instance, 20));
    vTaskDelay(pdMS_TO_TICKS(21000));
    vTaskDelay(pdMS_TO_TICKS(21000));
    ESP_ERROR_CHECK(MCP3564_startConversion(&MCP_instance, 10));
    vTaskDelay(pdMS_TO_TICKS(11000));
    vTaskDelay(pdMS_TO_TICKS(11000));
    ESP_ERROR_CHECK(MCP3564_startConversion(&MCP_instance, 5));
}