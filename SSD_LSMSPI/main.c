/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
* @defgroup spi_master_example_main main.c
* @{
* @ingroup spi_master_example
*
* @brief SPI Master Loopback Example Application main file.
*
* This file contains the source code for a sample application using SPI.
*
*/

#include "nrf_delay.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "nrf_drv_spi.h"
#include "bsp.h"
#include "app_timer.h"
#include "nordic_common.h"

#include "string.h"

#include "PinDef.h"
#include "SparkFunLSM9DS1.h"
#include "SFE_MicroOLED.h"

#define APP_TIMER_PRESCALER      0                      ///< Value of the RTC1 PRESCALER register.
#define APP_TIMER_OP_QUEUE_SIZE  2                      ///< Size of timer operation queues.

#define DELAY_MS                 1000                   ///< Timer Delay in milli-seconds.

/** @def  TX_RX_MSG_LENGTH
 * number of bytes to transmit and receive. This amount of bytes will also be tested to see that
 * the received bytes from slave are the same as the transmitted bytes from the master */
#define TX_RX_MSG_LENGTH         100


static uint8_t m_tx_data_spi[TX_RX_MSG_LENGTH]; ///< SPI master TX buffer.
static uint8_t m_rx_data_spi[TX_RX_MSG_LENGTH]; ///< SPI master RX buffer.

static volatile bool m_transfer_completed = true;

// 
static const nrf_drv_spi_t m_spi_master_0 = NRF_DRV_SPI_INSTANCE(0);
static const nrf_drv_spi_t m_spi_master_1 = NRF_DRV_SPI_INSTANCE(1);

/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    UNUSED_VARIABLE(bsp_indication_set(BSP_INDICATE_FATAL_ERROR));

    for (;;)
    {
        // No implementation needed.
    }
}

/**@brief The function initializes TX buffer to values to be sent and clears RX buffer.
 *
 * @note Function initializes TX buffer to values from 0 to (len - 1).
 *       and clears RX buffer (fill by 0).
 *
 * @param[out] p_tx_data    A pointer to a buffer TX.
 * @param[out] p_rx_data    A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 */
static void init_buf(uint8_t * const p_tx_buf,
                     uint8_t * const p_rx_buf,
                     const uint16_t  len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        p_tx_buf[i] = i;
        p_rx_buf[i] = 0;
    }
}


/**@brief Function for checking if buffers are equal.
 *
 * @note Function compares each element of p_tx_buf with p_rx_buf.
 *
 * @param[in] p_tx_data     A pointer to a buffer TX.
 * @param[in] p_rx_data     A pointer to a buffer RX.
 * @param[in] len           A length of the data buffers.
 *
 * @retval true     Buffers are equal.
 * @retval false    Buffers are different.
 */
static bool check_buf_equal(const uint8_t * const p_tx_buf,
                            const uint8_t * const p_rx_buf,
                            const uint16_t        len)
{
    uint16_t i;

    for (i = 0; i < len; i++)
    {
        if (p_tx_buf[i] != p_rx_buf[i])
        {
            return false;
        }
    }
    return true;
}


#if (SPI0_ENABLED == 1)
/**@brief Handler for SPI0 master events.
 *
 * @param[in] event SPI master event.
 */
void spi_master_0_event_handler(nrf_drv_spi_event_t event)
{
    uint32_t err_code = NRF_SUCCESS;
    bool result = false;

    switch (event)
    {
        case NRF_DRV_SPI_EVENT_DONE:
            // Check if received data is correct.
            result = check_buf_equal(m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
            APP_ERROR_CHECK_BOOL(result);

            nrf_drv_spi_uninit(&m_spi_master_0);

            err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
            APP_ERROR_CHECK(err_code);

            m_transfer_completed = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif // (SPI0_ENABLED == 1)


#if (SPI1_ENABLED == 1)
/**@brief Handler for SPI1 master events.
 *
 * @param[in] event SPI master event.
 */
void spi_master_1_event_handler(nrf_drv_spi_event_t event)
{
    uint32_t err_code = NRF_SUCCESS;
    bool result = false;

    switch (event)
    {
        case NRF_DRV_SPI_EVENT_DONE:
            // Check if received data is correct.
            result = check_buf_equal(m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
            APP_ERROR_CHECK_BOOL(result);

            nrf_drv_spi_uninit(&m_spi_master_1);

            err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
            APP_ERROR_CHECK(err_code);

            m_transfer_completed = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif // (SPI1_ENABLED == 1)


#if (SPI2_ENABLED == 1)
/**@brief Handler for SPI2 master events.
 *
 * @param[in] event SPI master event.
 */
void spi_master_2_event_handler(nrf_drv_spi_event_t event)
{
    uint32_t err_code = NRF_SUCCESS;
    bool result = false;

    switch (event)
    {
        case NRF_DRV_SPI_EVENT_DONE:
            // Check if received data is correct.
            result = check_buf_equal(m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
            APP_ERROR_CHECK_BOOL(result);

            nrf_drv_spi_uninit(&m_spi_master_2);

            err_code = bsp_indication_set(BSP_INDICATE_RCV_OK);
            APP_ERROR_CHECK(err_code);

            m_transfer_completed = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}
#endif // (SPI2_ENABLED == 1)

//static void spi_master_init(nrf_drv_spi_t const * p_instance, bool lsb)



/**@brief Function for initializing a SPI master driver.
 *
 * @param[in] p_instance    Pointer to SPI master driver instance.
 * @param[in] lsb           Bits order LSB if true, MSB if false.
 */
static void spi_master_init(nrf_drv_spi_t const * p_instance, bool lsb)
{
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_spi_config_t config =
    {
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_1M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = (lsb ?
            NRF_DRV_SPI_BIT_ORDER_LSB_FIRST : NRF_DRV_SPI_BIT_ORDER_MSB_FIRST),
    };

    #if (SPI0_ENABLED == 1)
    if (p_instance == &m_spi_master_0)
    {
        config.sck_pin  = SPIM0_SCK_PIN;
        config.mosi_pin = SPIM0_MOSI_PIN;
        config.miso_pin = SPIM0_MISO_PIN;
				config.frequency = NRF_DRV_SPI_FREQ_4M;
        err_code = nrf_drv_spi_init(p_instance, &config,
            spi_master_0_event_handler);
    }
    else
    #endif // (SPI0_ENABLED == 1)

    #if (SPI1_ENABLED == 1)
    if (p_instance == &m_spi_master_1)
    {
        config.sck_pin  = SPIM1_SCK_PIN;
        config.mosi_pin = SPIM1_MOSI_PIN;
        config.miso_pin = SPIM1_MISO_PIN;
        err_code = nrf_drv_spi_init(p_instance, &config,
            spi_master_1_event_handler);
    }
    else
    #endif // (SPI1_ENABLED == 1)

    #if (SPI2_ENABLED == 1)
    if (p_instance == &m_spi_master_2)
    {
        config.sck_pin  = SPIM2_SCK_PIN;
        config.mosi_pin = SPIM2_MOSI_PIN;
        config.miso_pin = SPIM2_MISO_PIN;
        err_code = nrf_drv_spi_init(p_instance, &config,
            spi_master_2_event_handler);
    }
    else
    #endif // (SPI2_ENABLED == 1)

    {}

    APP_ERROR_CHECK(err_code);
}


/**@brief Function for sending and receiving data.
 *
 * @param[in]   p_instance   Pointer to SPI master driver instance.
 * @param[in]   p_tx_data    A pointer to a buffer TX.
 * @param[out]  p_rx_data    A pointer to a buffer RX.
 * @param[in]   len          A length of the data buffers.
 */
static void spi_send_recv(nrf_drv_spi_t const * p_instance,
                          uint8_t * p_tx_data,
                          uint8_t * p_rx_data,
                          uint16_t  len)
{
    // Initalize buffers.
    init_buf(p_tx_data, p_rx_data, len);

    uint32_t err_code = nrf_drv_spi_transfer(p_instance,
        p_tx_data, len, p_rx_data, len);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for executing and switching state.
 *
 */
static void switch_state(void)
{
		/*
    nrf_drv_spi_t const * p_instance;

    switch (m_spi_master_ex_state)
    {
        #if (SPI0_ENABLED == 1)
        case TEST_STATE_SPI0_LSB:
            p_instance = &m_spi_master_0;
            spi_master_init(p_instance, true);
            break;

        case TEST_STATE_SPI0_MSB:
            p_instance = &m_spi_master_0;
            spi_master_init(p_instance, false);
            break;
        #endif // (SPI0_ENABLED == 1)

        #if (SPI1_ENABLED == 1)
        case TEST_STATE_SPI1_LSB:
            p_instance = &m_spi_master_1;
            spi_master_init(p_instance, true);
            break;

        case TEST_STATE_SPI1_MSB:
            p_instance = &m_spi_master_1;
            spi_master_init(p_instance, false);
            break;
        #endif // (SPI1_ENABLED == 1)

        #if (SPI2_ENABLED == 1)
        case TEST_STATE_SPI2_LSB:
            p_instance = &m_spi_master_2;
            spi_master_init(p_instance, true);
            break;

        case TEST_STATE_SPI2_MSB:
            p_instance = &m_spi_master_2;
            spi_master_init(p_instance, false);
            break;
        #endif // (SPI2_ENABLED == 1)

        default:
            return;
    }
    if (++m_spi_master_ex_state >= END_OF_TEST_SEQUENCE)
    {
        m_spi_master_ex_state = (spi_master_ex_state_t)0;
    }

    spi_send_recv(p_instance, m_tx_data_spi, m_rx_data_spi, TX_RX_MSG_LENGTH);
		*/
}


/**@brief Function for initializing bsp module.
 */
void bsp_configuration()
{
    uint32_t err_code = NRF_SUCCESS;

    NRF_CLOCK->LFCLKSRC            = (CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos);
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
        // Do nothing.
    }

    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);

    err_code = bsp_init(BSP_INIT_LED, APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), NULL);
    APP_ERROR_CHECK(err_code);
}


/**

*/
void SPI0MaterInit_forSSD1306()
{
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_spi_config_t config =
    {
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_8M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
		
		config.sck_pin  = SSDSCKPIN;
		config.mosi_pin = SSDMISO;
		config.miso_pin = SSDMOSI;

		err_code = nrf_drv_spi_init(&m_spi_master_0, &config,
				spi_master_0_event_handler);
		
		b_SSDSPI = &m_spi_master_0;
		
    APP_ERROR_CHECK(err_code);	
}

/**

*/
void SPI1MaterInit_forLSM9DS1()
{
    uint32_t err_code = NRF_SUCCESS;

    nrf_drv_spi_config_t config =
    {
        .ss_pin       = NRF_DRV_SPI_PIN_NOT_USED,
        .irq_priority = APP_IRQ_PRIORITY_LOW,
        .orc          = 0xCC,
        .frequency    = NRF_DRV_SPI_FREQ_8M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
		
		config.sck_pin  = LSMSCKPIN;
		config.mosi_pin = LSMSDIPIN;
		config.miso_pin = LSSDO_M;

		err_code = nrf_drv_spi_init(&m_spi_master_1, &config,
				spi_master_0_event_handler);
		
		b_LSMSPI = &m_spi_master_1;
		
    APP_ERROR_CHECK(err_code);	
}


void SSDprint(char* buff, uint8_t nLen)
{
		uint8_t idx = 0;
		for(idx = 0; idx < nLen; idx++)
			write(buff[idx]);
}


/** @brief Function for main application entry.
 */
int main(void)
{
		static uint8_t str_buffer[20];
	
    // Setup bsp module.
    bsp_configuration();
	
		settings.device.mAddress = LSMCS_M;
		settings.device.agAddress = LSSCS_AG;
		
	
		SPI0MaterInit_forSSD1306();
		SPI1MaterInit_forLSM9DS1();
	
		// SSD OLED Init
		MicroOLED(SSDRESET, SSDCSPIN, SSDSCKPIN);
		Oledbegin();
		clear(ALL);
		display();   // Logo Screen on SSD
		
		// 9 DOF device Begin
		if(IMUbegin() == 0)
		{
				// Fail to Communication with IMU device
				
		}
		else
		{
			
		}
		while(1)
		{
			nrf_delay_ms(1000);
			// Read data from LSM9DS1
			readGyro();
			readAccel();
			readMag();

			//
			setFontType(0);
			setCursor(0, 0);

			// RAW data Display, we can test calc function, too
			// Gyro Data 
			sprintf((char*)str_buffer, "GyroData: %u,", gx); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			sprintf((char*)str_buffer, "%u,", gy); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			sprintf((char*)str_buffer, "%u,", gz); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			// Accer Data 
			sprintf((char*)str_buffer, "AccerData: %u,", ax); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			sprintf((char*)str_buffer, "%u,", ay); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			sprintf((char*)str_buffer, "%u,", az); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			// Mag Data 
			sprintf((char*)str_buffer, "MagData: %u,", mx); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			sprintf((char*)str_buffer, "%u,", my); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			sprintf((char*)str_buffer, "%u,", mz); 
			SSDprint((char*)str_buffer, strlen((const char*)str_buffer));

			display();   // Logo Screen on SSD
				
		};

}

/** @} */
