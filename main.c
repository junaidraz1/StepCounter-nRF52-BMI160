/**
 * Copyright (c) 2009 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
* @brief Example template project.
* @defgroup nrf_templates_example Example Template
*
*/

#include <stdbool.h>
#include <stdint.h>
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "bmi160.h"
#include "nrf_drv_gpiote.h"
#include "app_uart.h" 
#include "nrf_uart.h" 
#include "arm_math.h" 
#include "fdacoefs.h"


#define SPI_INSTANCE 0 // SPI instance index. We use SPI master 0
#define SPI_SS_PIN 26
#define SPI_MISO_PIN 23
#define SPI_MOSI_PIN 24
#define SPI_SCK_PIN 22
#define BUTTON_PIN 27

// UART Pin Definitions
#define RX_PIN_NUMBER  8
#define TX_PIN_NUMBER  6
#define RTS_PIN_NUMBER 5
#define CTS_PIN_NUMBER 4

// UART buffer sizes
#define UART_TX_BUF_SIZE 256
#define UART_RX_BUF_SIZE 256

// Hardware flow control off
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED

#define NUM_TAPS 58
#define BLOCK_SIZE 28

// SPI instance
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);
// Flag used to indicate that SPI instance completed the transfer
static volatile bool spi_xfer_done;

static uint8_t SPI_RX_Buffer[100]; // Allocate a buffer for SPI reads
struct bmi160_dev sensor;          // An instance of bmi160 sensor

// Declare memory to store the raw FIFO buffer information
uint8_t fifo_buff[200];

// Variable to hold error data
uint32_t err_code;
uint32_t gpio_config_err;

// Modify the FIFO buffer instance and link to the device instance
struct bmi160_fifo_frame fifo_frame;

// 200 bytes -> ~7bytes per frame -> ~28 data frames
struct bmi160_sensor_data acc_data[28];

// Declare a state array
static int16_t firStateQ15[BLOCK_SIZE + NUM_TAPS - 1];

static int previous_step_index;

// Declare an instance for the low-pass FIR filter
arm_fir_instance_q15 fir_lpf;

int16_t acc_z_in_buf[BLOCK_SIZE]; 
int16_t acc_z_out_buf[BLOCK_SIZE];

int8_t block_cnt = 0;

// Add a variable for counting steps
volatile int step_count = 0;


/**
 * SPI user event handler.
 */
void spi_event_handler(nrf_drv_spi_evt_t const *p_event, void *p_context)
{
    spi_xfer_done = true; // Set a flag when transfer is done
}

/**
 * Function for setting up the SPI communication.
 */
uint32_t spi_config()
{
    uint32_t err_code;

    // Use nRF's default configurations
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;

    // Define each GPIO pin
    spi_config.ss_pin = SPI_SS_PIN;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin = SPI_SCK_PIN;

    // Initialize the SPI peripheral and give it a function pointer to
    // its event handler
    err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);

    return err_code;
}

/**
 * Function for writing to the BMI160 via SPI.
 */
int8_t bmi160_spi_bus_write(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t cnt)
{
    spi_xfer_done = false; // Set the flag down during transfer

    int32_t error = 0;
    uint8_t tx_buff[cnt + 1];

    uint16_t stringpos;
    // AND address with 0111 1111; set msb to '0' (write operation)
    tx_buff[0] = reg_addr & 0x7F;

    for (stringpos = 0; stringpos < cnt; stringpos++)
    {
        tx_buff[stringpos + 1] = *(reg_data + stringpos);
    }
    // Do the actual SPI transfer
    nrf_drv_spi_transfer(&spi, tx_buff, cnt + 1, NULL, 0);

    while (!spi_xfer_done)
    {
    }; // Loop until the transfer is complete

    return (int8_t)error;
}

/**
 * Function for reading from the BMI160 via SPI.
 */
int8_t bmi160_spi_bus_read(uint8_t hw_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    spi_xfer_done = false; // Set the flag down during transfer
    int32_t error = 0;
    uint8_t tx_buff = reg_addr | 0x80; // OR address with 1000 0000; Read -> set msb to '1'
    uint8_t *rx_buff_pointer = SPI_RX_Buffer;
    uint16_t stringpos;

    // Do the actual SPI transfer
    nrf_drv_spi_transfer(&spi, &tx_buff, 1, rx_buff_pointer, len + 1);

    while (!spi_xfer_done)
    {
    }; // Loop until the transfer is complete
    // Copy received bytes to reg_data
    for (stringpos = 0; stringpos < len; stringpos++)
        *(reg_data + stringpos) = SPI_RX_Buffer[stringpos + 1];

    return (int8_t)error;
}

/**
 * Function for configuring the sensor
 */
int8_t sensor_config()
{
    int8_t rslt = BMI160_OK;
    sensor.id = 0; // We use SPI so id == 0

    sensor.interface = BMI160_SPI_INTF;

    // Give the driver the correct interfacing functions
    sensor.read = bmi160_spi_bus_read;
    sensor.write = bmi160_spi_bus_write;
    sensor.delay_ms = nrf_delay_ms;

    // Initialize the sensor and check if everything went ok
    rslt = bmi160_init(&sensor);

  // Configure the accelerometer's sampling freq, range and modes
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_50HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    // Set the configurations
    rslt = bmi160_set_sens_conf(&sensor);

    // Some fifo settings
    fifo_frame.data = fifo_buff;
    fifo_frame.length = 200;
    sensor.fifo = &fifo_frame;


    // Configure the sensor's FIFO settings
    rslt = bmi160_set_fifo_config(BMI160_FIFO_ACCEL, BMI160_ENABLE, &sensor);
    
    // Create an instance for interrupt settings
    struct bmi160_int_settg int_config;

    // Interrupt channel/pin 1
    int_config.int_channel = BMI160_INT_CHANNEL_1;
    
    // Choosing fifo watermark interrupt
    int_config.int_type = BMI160_ACC_GYRO_FIFO_WATERMARK_INT;

    // Set fifo watermark level to 180
    rslt = bmi160_set_fifo_wm((uint8_t) 180, &sensor);

    // Enabling interrupt pins to act as output pin
    int_config.int_pin_settg.output_en = BMI160_ENABLE;

    // Choosing push-pull mode for interrupt pin
    int_config.int_pin_settg.output_mode = BMI160_DISABLE;
    
    // Choosing active high output
    int_config.int_pin_settg.output_type = BMI160_ENABLE;
    
    // Choosing edge triggered output
    int_config.int_pin_settg.edge_ctrl = BMI160_ENABLE;
    
    // Disabling interrupt pin to act as input
    int_config.int_pin_settg.input_en = BMI160_DISABLE;
    
    // Non-latched output
    int_config.int_pin_settg.latch_dur = BMI160_LATCH_DUR_NONE;
    
    // Enabling FIFO watermark interrupt
    int_config.fifo_WTM_int_en = BMI160_ENABLE;

    // Set interrupt configurations
    rslt = bmi160_set_int_config(&int_config, &sensor);

    return rslt;
}

/**
 * Function for reading FIFO data
 */
 int8_t get_bmi160_fifo_data()
{
    printf("Inside get_bmi160_fifo_data\r\n");
    int8_t rslt = BMI160_OK;

    uint8_t acc_frames_req = 28;

    // Read the fifo buffer using SPI
    rslt = bmi160_get_fifo_data(&sensor);

    // Parse the data and extract 28 accelerometer frames
    rslt = bmi160_extract_accel(acc_data, &acc_frames_req, &sensor);

    // Copy the contents of each axis to a FIR input buffer
    for (uint8_t i = 0; i < acc_frames_req; i++) {
        acc_z_in_buf[acc_frames_req * block_cnt + i] = acc_data[i].z;
    }

    // Apply FIR filter to the Z-axis data
    arm_fir_q15(&fir_lpf, &acc_z_in_buf[0], &acc_z_out_buf[0], BLOCK_SIZE);

    // New Step Detection Logic
    int step_threshold = 3000;  // Increase threshold to reduce noise
    int step_min_distance = 10; // Minimum number of samples between steps to avoid false positives
    previous_step_index = -step_min_distance; // Prevent multiple detections for the same step

    for (uint8_t i = 1; i < BLOCK_SIZE; i++) {
        // Detect peaks and valleys in the Z-axis data
        if (abs(acc_z_out_buf[i] - acc_z_out_buf[i - 1]) > step_threshold &&
            (i - previous_step_index) > step_min_distance) {
            // Count a step
            step_count++;
            previous_step_index = i; // Update the last step index
        }
    }

    // Print step count for debugging
    printf("Step Count: %d\r\n", step_count);

    return rslt;
}

/**
 * Interrupt handler for button press
 */
void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    printf("Inside Button Handlere");
    get_bmi160_fifo_data();
}

/**
 * UART error handler
 */
void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

/**
 * GPIO configuration function
 */
uint32_t config_gpio() {
    uint32_t err_code = NRF_SUCCESS;

    if (!nrf_drv_gpiote_is_init()) {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    //config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(BUTTON_PIN, &config, button_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(BUTTON_PIN, true);
    return err_code;
}

/**
 * Function for initializing the FIR filter instance
 */
void dsp_config() {
    // Note that the init function requires the address of the coefficient table B as an input
    arm_fir_init_q15(&fir_lpf, NUM_TAPS, B, firStateQ15, BLOCK_SIZE);
}

int main(void)
{
    uint32_t err_code;

    // Configure SPI
    err_code = spi_config();
    if (err_code != NRF_SUCCESS)
    {
        // Error check failed!
        printf("SPI configuration failed\r\n");
        return -1;
    }

    // Sensor configuration
    int8_t sensor_status = sensor_config();
    if (sensor_status != BMI160_OK)
    {
        // Sensor configuration failed
        printf("Sensor configuration failed\r\n");
        return -1;
    }

   //call to dsp config method
   dsp_config();

   //call to config gpio method
   config_gpio();

    // UART parameter configuration
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        UART_HWFC,
        false,
        NRF_UART_BAUDRATE_115200
    };

    // UART initialization macro
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_error_handle,
                       APP_IRQ_PRIORITY_HIGHEST,
                       err_code);

    if (err_code != NRF_SUCCESS)
    {
        printf("UART initialization failed\r\n");
        return -1;
    }

    printf("System initialized successfully\r\n");

    // Main loop
    while (true)
    {
        __WFI();
    }
}
