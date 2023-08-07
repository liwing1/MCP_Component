#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rom/gpio.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_log.h"

/************************* MCP3564 REGISTER DEFS ******************************/

#define _ADCDATA_ 0x00                      //MCP3564 ADCDATA Register Address.        
#define _CONFIG0_ 0x01                      //MCP3564 CONFIG0 Register Address.
#define _CONFIG1_ 0x02                      //MCP3564 CONFIG1 Register Address.
#define _CONFIG2_ 0x03                      //MCP3564 CONFIG2 Register Address.
#define _CONFIG3_ 0x04                      //MCP3564 CONFIF3 Register Address.
#define _IRQ_ 0x05                          //MCP3564 IRQ Register Address.
#define _MUX_ 0x06                          //MCP3564 MUX Register Address.
#define _SCAN_ 0x07                         //MCP3564 SCAN Register Address.
#define _TIMER_ 0x08                        //MCP3564 TIMER Register Address.
#define _OFFSETCAL_ 0x09                    //MCP3564 OFFSETCAL Register Address.
#define _GAINCAL_ 0x0A                      //MCP3564 GAINCAL Register Address.
#define _RESERVED_B_ 0x0B                   //MCP3564 Reserved B Register Address.
#define _RESERVED_C_ 0x0C                   //MCP3564 Reserved C Register Address.
#define _LOCK_ 0x0D                         //MCP3564 LOCK Register Address.
#define _RESERVED_E_ 0x0E                   //MCP3564 Reserved E Register Address.
#define _CRCCFG_ 0x0F                       //MCP3564 CRCCFG Register Address.

#define _WRT_CTRL_ 0b01000010               //MCP3564 Write-CMD Command-Byte.
#define _RD_CTRL_  0b01000001               //MCP3564 Read-CMD Command-Byte.

// Custom macros
#define SPI_CLOCK_SPEED 1000000 // 8.192 MHz
#define BUFFER_SIZE 1

static uint8_t tx_data[BUFFER_SIZE] = {0x00}; // Data to be transmitted
static uint8_t rx_data[BUFFER_SIZE] = {0x00}; // Buffer to store received data

static spi_device_handle_t spi;
static spi_transaction_t trans = {
    .length = 8 * BUFFER_SIZE,
    .tx_buffer = tx_data,
    .rx_buffer = rx_data
};

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

    // Data access
    uint32_t channel_data;
} MCP356x_t;


static void IRAM_ATTR GPIO_DRDY_IRQHandler(void* arg)
{
    uint32_t gpio_num = ((MCP356x_t*)arg)->gpio_num_ndrdy;
    if(gpio_num) ((MCP356x_t*)arg)->flag_drdy++;
    // readData(&channel_data);
}


static void init_spi(MCP356x_t* mcp_obj)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = mcp_obj->gpio_num_mosi,
        .miso_io_num = mcp_obj->gpio_num_miso,
        .sclk_io_num = mcp_obj->gpio_num_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = BUFFER_SIZE
    };

    spi_device_interface_config_t dev_config = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,  // can either be spi mode 0,0 or 1,1
        .duty_cycle_pos = 0,
        .cs_ena_posttrans = 0,
        .cs_ena_pretrans = 5,
        .clock_speed_hz = SPI_CLOCK_SPEED,  // 1 MHz clock speed
        .spics_io_num = -1,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL
    };
 
    spi_bus_initialize(SPI2_HOST, &bus_config, 3);
    spi_bus_add_device(SPI2_HOST, &dev_config, &spi);    
}

static uint8_t spiSendByte(uint8_t byte)
{
    tx_data[0] = byte;
    spi_device_transmit(spi, &trans);
    // printf("sendByte: %x\n", byte);   //Debug
    return rx_data[0];
}

static void writeSingleRegister(MCP356x_t* mcp_obj, uint8_t WRT_CMD, uint32_t WRT_DATA)
{
    uint8_t WRT_DATA_LOW;                                  //Register-Data Low-Byte.
    uint8_t WRT_DATA_HIGH;                                 //Register-Data High-Byte.
    uint8_t WRT_DATA_UPPER;                                //Register-Data Upper-Byte.
    uint8_t REG_ADDR;
    uint8_t STATUS;

    WRT_DATA_LOW = (uint8_t) (WRT_DATA & 0x0000FF);              //Extract Low-Byte from 24-bit Write-Data.
    WRT_DATA_HIGH = (uint8_t) ((WRT_DATA & 0x00FF00) >> 8);      //Extract High-Byte from 24-bit Write-Data.
    WRT_DATA_UPPER = (uint8_t) ((WRT_DATA & 0xFF0000) >> 16);    //Extract Upper-Byte from 24-bit Write-Data.

    REG_ADDR = (WRT_CMD & 0b00111100) >> 2;                 //Extract MCP3564 Register-Address for Write-CMD

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _LOCK_) 
    {
        gpio_set_level(mcp_obj->gpio_num_cs, 0);            //Assert CS Low on 8-bit HPC Curiosity --> Enable MCP3564 SPI Interface.

        //Transmit Write-CMD as one 8-bit packet.
        //Wait for SPI Shift Register to complete TX of Write-CMD.
        //Read SSP2BUF to retrieve MCP3564 STATUS-Byte.
        STATUS = spiSendByte(WRT_CMD);

        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete TX of Register-Data Low-Byte.
        //Read SSP2BUF to clear buffer. 
        spiSendByte(WRT_DATA_LOW);                   

        gpio_set_level(mcp_obj->gpio_num_cs, 1);            //Raise CS High on 8-bit HPC Curiosity --> Reset MCP3564 SPI Interface.
    } 
    else if (REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_) 
    {
        gpio_set_level(mcp_obj->gpio_num_cs, 0);            //Assert CS Low on 8-bit HPC Curiosity --> Enable MCP3564 SPI Interface.

        //Transmit Write-CMD as one 8-bit packet.
        //Wait for SPI Shift Register to complete TX of Write-CMD.
        //Read SSP2BUF to retrieve MCP3564 STATUS-Byte.
        STATUS = spiSendByte(WRT_CMD);

        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete TX of Register-Data Low-Byte.
        //Read SSP2BUF to clear buffer. 
        spiSendByte(WRT_DATA_UPPER);
        
        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete TX of Register-Data Low-Byte.
        //Read SSP2BUF to clear buffer. 
        spiSendByte(WRT_DATA_HIGH);
        
        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete TX of Register-Data Low-Byte.
        //Read SSP2BUF to clear buffer. 
        spiSendByte(WRT_DATA_LOW);

        gpio_set_level(mcp_obj->gpio_num_cs, 1);            //Raise CS High on 8-bit HPC Curiosity --> Reset MCP3564 SPI Interface.
    }
}

static uint32_t readSingleRegister(MCP356x_t* mcp_obj, uint8_t RD_CMD)
{
    uint32_t READ_VALUE = 0;
    uint8_t REG_ADDR;
    uint8_t STATUS;

    REG_ADDR = (RD_CMD & 0b00111100) >> 2;                  //Extract MCP3564 Register-Address for Write-CMD

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _RESERVED_C_ || REG_ADDR == _LOCK_) 
    {
        gpio_set_level(mcp_obj->gpio_num_cs, 0);            //Assert CS Low on 8-bit HPC Curiosity --> Enable MCP3564 SPI Interface.

        //Transmit Read-CMD as one 8-bit packet.
        //Wait for SPI Shift Register to complete TX of Read-CMD.
        //Read SSP2BUF to retrieve MCP3564 STATUS-Byte.
        STATUS = spiSendByte(RD_CMD);

        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data Low-Byte.
        //Read SSP2BUF to clear buffer. 
        READ_VALUE = (uint32_t)spiSendByte(0x00);                   

        gpio_set_level(mcp_obj->gpio_num_cs, 1);            //Raise CS High on 8-bit HPC Curiosity --> Reset MCP3564 SPI Interface.
    } 
    else if (REG_ADDR == _RESERVED_E_ || REG_ADDR == _CRCCFG_) 
    {
        gpio_set_level(mcp_obj->gpio_num_cs, 0);            //Assert CS Low on 8-bit HPC Curiosity --> Enable MCP3564 SPI Interface.

        //Transmit Read-CMD as one 8-bit packet.
        //Wait for SPI Shift Register to complete TX of Read-CMD.
        //Read SSP2BUF to retrieve MCP3564 STATUS-Byte.
        STATUS = spiSendByte(RD_CMD);

        //Transmit Register-Data High-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data High-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE = ((uint32_t)spiSendByte(0x00)) << 8;

        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data High-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE |= (uint32_t)spiSendByte(0x00);

        gpio_set_level(mcp_obj->gpio_num_cs, 1);            //Raise CS High on 8-bit HPC Curiosity --> Reset MCP3564 SPI Interface.
    }
    else if (REG_ADDR == _ADCDATA_ || REG_ADDR == _SCAN_ || REG_ADDR == _TIMER_ || REG_ADDR == _OFFSETCAL_ || REG_ADDR == _GAINCAL_ || REG_ADDR == _RESERVED_B_) 
    {
        gpio_set_level(mcp_obj->gpio_num_cs, 0);            //Assert CS Low on 8-bit HPC Curiosity --> Enable MCP3564 SPI Interface.

        //Transmit Read-CMD as one 8-bit packet.
        //Wait for SPI Shift Register to complete TX of Read-CMD.
        //Read SSP2BUF to retrieve MCP3564 STATUS-Byte.
        STATUS = spiSendByte(RD_CMD);

        //Transmit Register-Data High-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data Upper-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE = ((uint32_t)spiSendByte(0x00)) << 16;

        //Transmit Register-Data High-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data High-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE |= ((uint32_t)spiSendByte(0x00)) << 8;

        //Transmit Register-Data Low-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data High-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE |= (uint32_t)spiSendByte(0x00);

        gpio_set_level(mcp_obj->gpio_num_cs, 1);            //Raise CS High on 8-bit HPC Curiosity --> Reset MCP3564 SPI Interface.
    }
    return READ_VALUE;
}

static void init_clkout(MCP356x_t* mcp_obj){
    // Configure PWM settings
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_3_BIT,
        .freq_hz = 8192000, /*1000000*/
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0
    };

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 4,
        .gpio_num = mcp_obj->gpio_num_pwm,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    
    ledc_timer_config(&ledc_timer);
    ledc_channel_config(&ledc_channel);
}

void mcpStartup(MCP356x_t* mcp_obj) 
{
    // Set CS as Output
    esp_rom_gpio_pad_select_gpio(mcp_obj->gpio_num_cs);
    gpio_set_direction(mcp_obj->gpio_num_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(mcp_obj->gpio_num_cs, 1);

    // First enable interrupts
    esp_rom_gpio_pad_select_gpio(mcp_obj->gpio_num_ndrdy);
    gpio_set_direction(mcp_obj->gpio_num_ndrdy, GPIO_MODE_INPUT);
    gpio_set_pull_mode(mcp_obj->gpio_num_ndrdy, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(mcp_obj->gpio_num_ndrdy, GPIO_INTR_NEGEDGE);
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(mcp_obj->gpio_num_ndrdy, GPIO_DRDY_IRQHandler, (void*)mcp_obj);

    init_clkout(mcp_obj);

    //GAINCAL --> (8,253,056 / 8,388,607 = 1.615894%) Gain Error w/2.048V Input
    writeSingleRegister(mcp_obj, ((_GAINCAL_ << 2) | _WRT_CTRL_), 0x7DEE80);     

    //OFFSETCAL --> +62 Counts of Offset Cancellation (Measured Offset is Negative).
    writeSingleRegister(mcp_obj, ((_OFFSETCAL_ << 2) | _WRT_CTRL_), 0x00003E);

    //TIMER --> Disabled.
    writeSingleRegister(mcp_obj, ((_TIMER_ << 2) | _WRT_CTRL_), 0x000000);

    //SCAN --> Disabled.
    writeSingleRegister(mcp_obj, ((_SCAN_ << 2) | _WRT_CTRL_), 0x000000);

    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
    writeSingleRegister(mcp_obj, ((_MUX_ << 2) | _WRT_CTRL_), 0x01);

    //IRQ --> IRQ Mode = Low level active IRQ Output  --> (0b00000000).
    writeSingleRegister(mcp_obj, ((_IRQ_ << 2) | _WRT_CTRL_), 0x04);

    //CONFIG3 --> Conv. Mod = One-Shot Conv. Mode, FORMAT = 24b,
    //CRC_FORMAT = 16b, CRC-COM = Disabled,
    //OFFSETCAL = Enabled, GAINCAL = Enabled --> (0b10000011).
    writeSingleRegister(mcp_obj, ((_CONFIG3_ << 2) | _WRT_CTRL_), 0x83);
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 1 --> (0b10001111).
    writeSingleRegister(mcp_obj, ((_CONFIG2_ << 2) | _WRT_CTRL_), 0x8F);

    //CONFIG1 --> AMCLK = MCLK, OSR = 98304 --> (0b00111100).      
    writeSingleRegister(mcp_obj, ((_CONFIG1_ << 2) | _WRT_CTRL_), 0x3C);

    //CONFIG0 --> CLK_SEL = extCLK, CS_SEL = No Bias, ADC_MODE = Standby Mode --> (0b11000010).
    writeSingleRegister(mcp_obj, ((_CONFIG0_ << 2) | _WRT_CTRL_), 0xC2);
}


MCP356x_t MCP_instance = {
    .gpio_num_miso = GPIO_NUM_13,
    .gpio_num_mosi = GPIO_NUM_11,
    .gpio_num_clk = GPIO_NUM_12,
    .gpio_num_cs = GPIO_NUM_10,
    .gpio_num_pwm = GPIO_NUM_18,
    .gpio_num_nsync = GPIO_NUM_16,
    .gpio_num_ndrdy = GPIO_NUM_1,
    .flag_drdy = 0,
    .channel_data = 0
};


void mcp_task(void* p)
{
    uint32_t data_read;
    while(1) 
    {
        //Conversion-Start via Write-CMD to CONFIG0[1:0] ADC_MODE[1:0]] = 11
        writeSingleRegister(&MCP_instance, ((_CONFIG0_ << 2) | _WRT_CTRL_), 0xC3);

        esp_rom_delay_us(10);
        
        if(MCP_instance.flag_drdy == 1)
        {
            data_read = readSingleRegister(&MCP_instance, (_ADCDATA_ << 2) | _RD_CTRL_);

            printf("%ld, %ld\r\n", esp_log_timestamp(), data_read);
            
            MCP_instance.flag_drdy = 0;             //Data-Ready Flag via MCP3564 IRQ Alert.
        }

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

// #define __TESTE__

void app_main() 
{
    init_spi(&MCP_instance);
    mcpStartup(&MCP_instance);

#ifdef  __TESTE__
    xTaskCreate(mcp_task, "mcp_task", 2048 * 8, NULL, configMAX_PRIORITIES - 1, NULL);
#else
    while(1){
        uint32_t rx_value = readSingleRegister(&MCP_instance, ((_CONFIG0_ << 2) | _RD_CTRL_));
        printf("read register-> %x: %lx\r\n", ((_CONFIG0_ << 2) | _RD_CTRL_), rx_value);
        vTaskDelay(pdMS_TO_TICKS(2500));
    }
#endif  // __TESTE__
}