/**
 ****************************************************************************************
 *
 * @file user_periph_setup.c
 *
 * @brief Peripherals setup and initialization.
 *
 * Copyright (C) 2015-2019 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "user_periph_setup.h"
#include "datasheet.h"
#include "system_library.h"
#include "rwip_config.h"
#include "gpio.h"
#include "uart.h"
#include "syscntl.h"

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 ****************************************************************************************
 */

#if DEVELOPMENT_DEBUG

void GPIO_reservations(void)
{
/*
    i.e. to reserve P0_1 as Generic Purpose I/O:
    RESERVE_GPIO(DESCRIPTIVE_NAME, GPIO_PORT_0, GPIO_PIN_1, PID_GPIO);
*/

#if defined (CFG_PRINTF_UART2)
    //RESERVE_GPIO(UART2_TX, UART2_TX_PORT, UART2_TX_PIN, PID_UART2_TX);
#endif

		RESERVE_GPIO(M_EN, GPIO_EN_PORT, GPIO_EN, PID_GPIO);
	
    RESERVE_GPIO(M1, GPIO_M1_PORT, GPIO_M1_PIN, PID_GPIO);
		RESERVE_GPIO(M2, GPIO_M2_PORT, GPIO_M2_PIN, PID_GPIO);
		RESERVE_GPIO(HANDLE, GPIO_HANDLE_PORT, GPIO_HANDLE_PIN, PID_GPIO);
		RESERVE_GPIO(BOLT, GPIO_BOLT_PORT, GPIO_BOLT_PIN, PID_GPIO);
#if defined (CFG_SPI_FLASH_ENABLE) && !defined (__DA14586__)
    // SPI Flash
		RESERVE_GPIO(SPI_EN, SPI_EN_PORT, SPI_EN_PIN, PID_SPI_EN);
		RESERVE_GPIO(SPI_CLK, SPI_EN_PORT, SPI_CLK_PIN, PID_SPI_CLK);
		RESERVE_GPIO(SPI_MOSI, SPI_EN_PORT, SPI_DO_PIN, PID_SPI_DO);
		RESERVE_GPIO(SPI_MISO, SPI_EN_PORT, SPI_DI_PIN, PID_SPI_DI);
#endif
		RESERVE_GPIO(ADC_INPUT, ADC_INPUT_PORT, ADC_INPUT_PIN, PID_ADC);



}

#endif

void set_pad_functions(void)
{
/*
    i.e. to set P0_1 as Generic purpose Output:
    GPIO_ConfigurePin(GPIO_PORT_0, GPIO_PIN_1, OUTPUT, PID_GPIO, false);
*/
#if defined (CFG_SPI_FLASH_ENABLE)
    // SPI Flash
    GPIO_ConfigurePin(SPI_EN_PORT, SPI_EN_PIN, OUTPUT, PID_SPI_EN, true);
    GPIO_ConfigurePin(SPI_CLK_PORT, SPI_CLK_PIN, OUTPUT, PID_SPI_CLK, false);
    GPIO_ConfigurePin(SPI_DO_PORT, SPI_DO_PIN, OUTPUT, PID_SPI_DO, false);
    GPIO_ConfigurePin(SPI_DI_PORT, SPI_DI_PIN, INPUT, PID_SPI_DI, false);
#endif

#if defined (CFG_PRINTF_UART2)
    // Configure UART2 TX Pad
    //GPIO_ConfigurePin(UART2_TX_PORT, UART2_TX_PIN, OUTPUT, PID_UART2_TX, false);
#endif
		extern bool my_m1_state;
		extern bool my_m2_state;
		extern bool my_en_state;
		extern bool my_button;
    GPIO_ConfigurePin(GPIO_M1_PORT, GPIO_M1_PIN, OUTPUT, PID_GPIO, my_m1_state);
		GPIO_ConfigurePin(GPIO_M2_PORT, GPIO_M2_PIN, OUTPUT, PID_GPIO, my_m2_state);
		GPIO_ConfigurePin(GPIO_EN_PORT, GPIO_EN, OUTPUT, PID_GPIO, my_en_state);
		GPIO_ConfigurePin(GPIO_HANDLE_PORT, GPIO_HANDLE_PIN, INPUT_PULLUP, PID_GPIO, my_m2_state);
		GPIO_ConfigurePin(GPIO_BOLT_PORT, GPIO_BOLT_PIN, INPUT_PULLUP, PID_GPIO, false);
		GPIO_ConfigurePin(ADC_INPUT_PORT, ADC_INPUT_PIN, INPUT, PID_ADC, false);

}

#if defined (CFG_PRINTF_UART2)
// Configuration struct for UART2
static const uart_cfg_t uart_cfg = {
    .baud_rate = UART2_BAUDRATE,
    .data_bits = UART2_DATABITS,
    .parity = UART2_PARITY,
    .stop_bits = UART2_STOPBITS,
    .auto_flow_control = UART2_AFCE,
    .use_fifo = UART2_FIFO,
    .tx_fifo_tr_lvl = UART2_TX_FIFO_LEVEL,
    .rx_fifo_tr_lvl = UART2_RX_FIFO_LEVEL,
    .intr_priority = 2,
};
#endif
#if defined (CFG_SPI_FLASH_ENABLE)
    #include "spi.h"
    #include "spi_flash.h"
#endif

#if defined (CFG_SPI_FLASH_ENABLE)
// Configuration struct for SPI
static const spi_cfg_t spi_cfg = {
    .spi_ms = SPI_MS_MODE,
    .spi_cp = SPI_CP_MODE,
    .spi_speed = SPI_SPEED_MODE,
    .spi_wsz = SPI_WSZ,
    .spi_cs = SPI_CS,
    .cs_pad.port = SPI_EN_PORT,
    .cs_pad.pin = SPI_EN_PIN,
#if defined (__DA14531__)
    .spi_capture = SPI_EDGE_CAPTURE,
#endif
};

#define P25Q11U_MAN_DEV_ID                      0x8510
#define P25Q11U_JEDEC_ID                        0x854011
#define P25Q11U_CHIP_SIZE                       0x20000

static const spi_flash_cfg_t spi_flash_cfg = {
    .dev_index = P25Q11U_MAN_DEV_ID,//MX25R2035F_DEV_INDEX,
    .jedec_id  = P25Q11U_JEDEC_ID,//MX25V2035F_JEDEC_ID,
    //.chip_size = P25Q11U_CHIP_SIZE,//MX25V2035F_CHIP_SIZE,
		.chip_size = SPI_FLASH_DEV_SIZE,
};
#endif

void periph_init(void)
{
#if defined (__DA14531__)
	   // Disable HW Reset functionality of P0_0
    GPIO_Disable_HW_Reset();
    // In Boost mode enable the DCDC converter to supply VBAT_HIGH for the used GPIOs
    syscntl_dcdc_turn_on_in_boost(SYSCNTL_DCDC_LEVEL_3V0);
#else
    // Power up peripherals' power domain
    SetBits16(PMU_CTRL_REG, PERIPH_SLEEP, 0);
    while (!(GetWord16(SYS_STAT_REG) & PER_IS_UP));
    SetBits16(CLK_16M_REG, XTAL16_BIAS_SH_ENABLE, 1);
#endif

    // ROM patch
    patch_func();

    // Initialize peripherals
#if defined (CFG_PRINTF_UART2)
    // Initialize UART2
    uart_initialize(UART2, &uart_cfg);
#endif
		    // Initialize SPI
    spi_initialize(&spi_cfg);
			    // Configure SPI Flash environment
    spi_flash_configure_env(&spi_flash_cfg);
#if defined (CFG_SPI_FLASH_ENABLE)
    // Configure SPI Flash environment
    spi_flash_configure_env(&spi_flash_cfg);

    // Initialize SPI
    spi_initialize(&spi_cfg);
#endif
    // Set pad functionality
    set_pad_functions();

    // Enable the pads
    GPIO_set_pad_latch_en(true);
}
