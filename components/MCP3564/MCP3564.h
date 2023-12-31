#ifndef __MCP3564_H__
#define __MCP3564_H__

#include "driver/gpio.h"

typedef struct{
    // Pin mapping
    gpio_num_t gpio_num_miso;
    gpio_num_t gpio_num_mosi;
    gpio_num_t gpio_num_clk;
    gpio_num_t gpio_num_cs;
    gpio_num_t gpio_num_pwm;
    gpio_num_t gpio_num_nsync;
    gpio_num_t gpio_num_ndrdy;

    // External flag 
    uint32_t flag_drdy;
} MCP356x_t;

esp_err_t MCP3564_startConversion(MCP356x_t* mcp_obj, size_t duration_seconds);
void mcpStartup(MCP356x_t* mcp_obj);

#endif