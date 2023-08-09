#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "MCP3564.h"

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
#define SPI_CLOCK_SPEED 20000000 // 20 MHz
#define SAMPLING_FREQ   100 // Hz
#define SAMPLING_PERIOD_MS (1000/SAMPLING_FREQ)
#define NUM_CHANNELS    8 // ch7 to ch0

#define nPOR_STATUS     BIT0
#define nCRCCFG_STATUS  BIT1

typedef struct{
    MCP356x_t* device;
    size_t window_size_sec;
} adc_request_t;


// Global variables
static uint8_t tx_data = 0; // Data to be transmitted
static uint8_t rx_data = 0; // Buffer to store received data
static uint8_t g_status = 0;

static spi_device_handle_t spi;
static spi_transaction_t trans = {
    .length = 8,
    .tx_buffer = &tx_data,
    .rx_buffer = &rx_data
};

static TaskHandle_t mcp_task_handler = NULL;

static void IRAM_ATTR GPIO_DRDY_IRQHandler(void* arg)
{
    uint32_t gpio_num = ((MCP356x_t*)arg)->gpio_num_ndrdy;
    if(gpio_num) ((MCP356x_t*)arg)->flag_drdy++;
}


static void init_spi(MCP356x_t* mcp_obj)
{
    spi_bus_config_t bus_config = {
        .mosi_io_num = mcp_obj->gpio_num_mosi,
        .miso_io_num = mcp_obj->gpio_num_miso,
        .sclk_io_num = mcp_obj->gpio_num_clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 1
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
    tx_data = byte;
    spi_device_transmit(spi, &trans);
    // printf("sendByte: %x\n", byte);   //Debug
    return rx_data;
}

static void writeSingleRegister(MCP356x_t* mcp_obj, uint8_t WRT_CMD, uint32_t WRT_DATA)
{
    uint8_t WRT_DATA_LOW;                                  //Register-Data Low-Byte.
    uint8_t WRT_DATA_HIGH;                                 //Register-Data High-Byte.
    uint8_t WRT_DATA_UPPER;                                //Register-Data Upper-Byte.
    uint8_t REG_ADDR;

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
        g_status = spiSendByte(WRT_CMD);

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
        g_status = spiSendByte(WRT_CMD);

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

    REG_ADDR = (RD_CMD & 0b00111100) >> 2;                  //Extract MCP3564 Register-Address for Write-CMD

    if (REG_ADDR == _CONFIG0_ || REG_ADDR == _CONFIG1_ || REG_ADDR == _CONFIG2_ || REG_ADDR == _CONFIG3_ || REG_ADDR == _IRQ_ || REG_ADDR == _MUX_ || REG_ADDR == _RESERVED_C_ || REG_ADDR == _LOCK_) 
    {
        gpio_set_level(mcp_obj->gpio_num_cs, 0);            //Assert CS Low on 8-bit HPC Curiosity --> Enable MCP3564 SPI Interface.

        //Transmit Read-CMD as one 8-bit packet.
        //Wait for SPI Shift Register to complete TX of Read-CMD.
        //Read SSP2BUF to retrieve MCP3564 STATUS-Byte.
        g_status = spiSendByte(RD_CMD);

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
        g_status = spiSendByte(RD_CMD);

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
        g_status = spiSendByte(RD_CMD);

        //Transmit Register-Data High-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data Upper-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE = ((uint32_t)spiSendByte(0x00)) << 24;

        //Transmit Register-Data High-Byte as one 8-bit packet. 
        //Wait for SPI Shift Register to complete RX of Register-Data Upper-Byte.
        //Read SSP2BUF to clear buffer 
        READ_VALUE |= ((uint32_t)spiSendByte(0x00)) << 16;

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

void mcpStartup(MCP356x_t* mcp_obj) 
{
    init_spi(mcp_obj);

    // Set CS as Output
    esp_rom_gpio_pad_select_gpio(mcp_obj->gpio_num_cs);
    gpio_set_direction(mcp_obj->gpio_num_cs, GPIO_MODE_OUTPUT);
    gpio_set_level(mcp_obj->gpio_num_cs, 1);

    // First enable interrupts
    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 1 << mcp_obj->gpio_num_ndrdy,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&gpio_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(mcp_obj->gpio_num_ndrdy, GPIO_DRDY_IRQHandler, (void*)mcp_obj);

    //GAINCAL --> (8,253,056 / 8,388,607 = 1.615894%) Gain Error w/2.048V Input
    writeSingleRegister(mcp_obj, ((_GAINCAL_ << 2) | _WRT_CTRL_), 0x7DEE80);     

    //OFFSETCAL --> +62 Counts of Offset Cancellation (Measured Offset is Negative).
    writeSingleRegister(mcp_obj, ((_OFFSETCAL_ << 2) | _WRT_CTRL_), 0x00003E);

    //TIMER --> Disabled.
    writeSingleRegister(mcp_obj, ((_TIMER_ << 2) | _WRT_CTRL_), 0x000000);

    //SCAN --> Disabled.
    writeSingleRegister(mcp_obj, ((_SCAN_ << 2) | _WRT_CTRL_), 0x0000FF);

    //MUX --> VIN+ = CH0, VIN- = CH1 --> (0b00000001).
    writeSingleRegister(mcp_obj, ((_MUX_ << 2) | _WRT_CTRL_), 0xBC);

    //IRQ --> IRQ Mode = Low level active IRQ Output  --> (0b00000000).
    writeSingleRegister(mcp_obj, ((_IRQ_ << 2) | _WRT_CTRL_), 0x04);

    //CONFIG3 --> Conv. Mod = One-Shot Conv. Mode, FORMAT = 32b + chid,
    //CRC_FORMAT = 16b, CRC-COM = Disabled,
    //OFFSETCAL = Enabled, GAINCAL = Enabled --> (0b10110011).
    writeSingleRegister(mcp_obj, ((_CONFIG3_ << 2) | _WRT_CTRL_), 0xB3);
    
    //CONFIG2 --> BOOST = 1x, GAIN = 1x, AZ_MUX = 0 --> (0b10001011).
    writeSingleRegister(mcp_obj, ((_CONFIG2_ << 2) | _WRT_CTRL_), 0x8B);

    //CONFIG1 --> AMCLK = MCLK, OSR = 256 --> (0b00001100).      
    writeSingleRegister(mcp_obj, ((_CONFIG1_ << 2) | _WRT_CTRL_), 0x0C);

    //CONFIG0 --> CLK_SEL = extCLK, CS_SEL = No Bias, ADC_MODE = Standby Mode --> (0b11000010).
    writeSingleRegister(mcp_obj, ((_CONFIG0_ << 2) | _WRT_CTRL_), 0xE2);
}

static inline uint32_t** mallocMeasureArray(uint16_t num_rows, uint8_t num_columns)
{
    uint32_t** array = (uint32_t**)calloc(1, num_rows * sizeof(uint32_t));

    if(array == NULL)
    {
        return NULL;
    } 

    for(uint16_t i = 0; i < num_rows; i++)
    {
        array[i] = (uint32_t*)calloc(1, num_columns * sizeof(uint32_t));

        if(array[i] == NULL)
        {
            return NULL;
        }
    }

    return array;
}

static inline void freeMeasureArray(uint32_t** measures, uint16_t num_rows)
{
    for(uint16_t i = 0; i < num_rows; i++)
    {
        free(measures[i]);
    }
    free(measures);
}

static int32_t signExtend(uint32_t Bytes)
{
    int32_t signByte    = ((int32_t) (Bytes & 0x0F000000));
    int32_t dataBytes   = ((int32_t) (Bytes & 0x00FFFFFF));

    return ((signByte<<4) | signByte | dataBytes);
}

static inline void printMeasures(uint32_t** measures, uint16_t num_rows, uint8_t num_columns)
{
    printf("timestamp,ch7,ch6,ch5,ch4,ch3,ch2,ch1,ch0\n");
    for(uint16_t row = 0; row < num_rows; row++)
    {
        printf("%ld", measures[row][NUM_CHANNELS]);
        for(uint8_t column = 0; column < NUM_CHANNELS; column++)
        {
            printf(",%08ld", signExtend(measures[row][column]));
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void mcp_task(void* p)
{
    MCP356x_t* mcp_obj = (MCP356x_t*)((adc_request_t*)p)->device;
    uint16_t num_rows = ((adc_request_t*)p)->window_size_sec * SAMPLING_FREQ;
    uint8_t num_columns = NUM_CHANNELS + 1; //timevector + channels

    /* INITIALIZE MEMORY SPACE */
    uint32_t** measures = mallocMeasureArray(num_rows, num_columns);
    if(measures == NULL){
        ESP_LOGE("MCP", "memory allocation failed");
        while(1);
    }

    /* SAMPLE CHANNELS */
    printf("Start conversion \r\n");

    for(uint16_t row = 0; row < num_rows; row++)
    {
        measures[row][NUM_CHANNELS] = row*SAMPLING_PERIOD_MS;   // Timevector

        //Conversion-Start via Write-CMD to CONFIG0[1:0] ADC_MODE[1:0]] = 11
        writeSingleRegister(mcp_obj, ((_CONFIG0_ << 2) | _WRT_CTRL_), 0xE3);

        //Read conversion. Store CH7 in column 0, CH6 in column 1...
        for(uint8_t column = 0; column < NUM_CHANNELS; column++)
        {
            while(!mcp_obj->flag_drdy) esp_rom_delay_us(50);//vTaskDelay(1); // To do: add timeout
            
            measures[row][column] = readSingleRegister(mcp_obj, (_ADCDATA_ << 2) | _RD_CTRL_);
            
            mcp_obj->flag_drdy = 0;             //Data-Ready Flag via MCP3564 IRQ Alert.
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLING_PERIOD_MS));
    }

    /* OUTPUT DATA */
    printMeasures(measures, num_rows, num_columns);

    /* CLEAR ALLOCATED MEMORY */
    freeMeasureArray(measures, num_rows);

    vTaskDelete(mcp_task_handler);
}

esp_err_t MCP3564_startConversion(MCP356x_t* mcp_obj, size_t duration_seconds)
{
    BaseType_t status = 0;
    static adc_request_t adc_request = {0};

    adc_request.device = mcp_obj;
    adc_request.window_size_sec = duration_seconds;

    // TODO: add verify to prevent 2 instances
    // eTaskState taskState = eTaskGetState(mcp_task_handler);

    // if(taskState == eRunning)
    // {
    //     ESP_LOGE("MCP", "STILL EXECUTING");
    //     return ESP_FAIL;
    // }

    status = xTaskCreate(
        mcp_task,
        "mcp_task", 
        2048 * 8, 
        &adc_request, 
        configMAX_PRIORITIES - 1, 
        &mcp_task_handler
    );

    if(errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY == status)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}