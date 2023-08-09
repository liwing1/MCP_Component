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

#endif