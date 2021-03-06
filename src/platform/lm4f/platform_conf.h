// eLua platform configuration

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "auxmods.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "stacks.h"
#include "driverlib/sysctl.h"
#include "elua_int.h"
#include "flash_conf.h"
#include "rom_map.h"

// *****************************************************************************
// Define here what components you want for this platform
#define BUILD_XMODEM
#define BUILD_TERM

// defining processor part
//TODO: make this code more reliable
#define PART_LM4F120B2QR

#define BUILD_SHELL
#define BUILD_ROMFS
#define BUILD_MMCFS

// TODO: I'll leave USB CDC disabled. This will change in future.
//#define BUILD_USB_CDC

#define BUILD_LINENOISE

#define BUILD_ADC
#define BUILD_RPC

#define BUILD_CON_GENERIC

#define BUILD_C_INT_HANDLERS

// TODO: discover if the defines below are necessary
//#define BUILD_LUA_INT_HANDLERS
//#define PLATFORM_INT_QUEUE_LOG_SIZE 5

#define PLATFORM_HAS_SYSTIMER
#define PLATFORM_TMR_COUNTS_DOWN

#ifdef INTERNAL_FLASH_CONFIGURED // this comes from flash_conf.h
#define BUILD_WOFS
#endif

#define ENABLE_LM4F_GPIO

#define LINENOISE_HISTORY_SIZE_LUA    30
#define LINENOISE_HISTORY_SIZE_SHELL  10

// *****************************************************************************
// UART/Timer IDs configuration data (used in main.c)

#define CON_UART_ID           0

#define CON_UART_SPEED        115200
#define TERM_LINES            25
#define TERM_COLS             80

// *****************************************************************************
// Auxiliary libraries that will be compiled for this platform

// The name of the platform specific libs table
// FIXME: should handle partial or no inclusion of platform specific modules per conf.py
#if defined( ENABLE_LM4F_GPIO )
#define PS_LIB_TABLE_NAME   "lm4f"
#endif

// TODO: Adapt this CANLINE define
#if defined( FORLM3S8962 ) || defined( FORLM3S9B92 ) || defined( FORLM3S9D92 )
#define CANLINE  _ROM( AUXLIB_CAN, luaopen_can, can_map )
#define BUILD_CAN
#else
#define CANLINE
#endif

// TODO: Adapt this PWMLINE define
#ifdef FORLM3S6918
#define PWMLINE
#else
#define PWMLINE  _ROM( AUXLIB_PWM, luaopen_pwm, pwm_map )
#endif

#ifdef BUILD_UIP
#define NETLINE  _ROM( AUXLIB_NET, luaopen_net, net_map )
#else
#define NETLINE
#endif

#ifdef BUILD_ADC
#define ADCLINE _ROM( AUXLIB_ADC, luaopen_adc, adc_map )
#else
#define ADCLINE
#endif

#ifdef BUILD_TERM
#define TERMLINE _ROM( AUXLIB_TERM, luaopen_term, term_map )
#else
#define TERMLINE
#endif

#if defined( ELUA_BOOT_RPC ) && !defined( BUILD_RPC )
#define BUILD_RPC
#endif

#if defined( BUILD_RPC ) 
#define RPCLINE _ROM( AUXLIB_RPC, luaopen_rpc, rpc_map )
#else
#define RPCLINE
#endif

#ifdef PS_LIB_TABLE_NAME
#define PLATLINE _ROM( PS_LIB_TABLE_NAME, luaopen_platform, platform_map )
#else
#define PLATLINE
#endif



#define LUA_PLATFORM_LIBS_ROM\
  _ROM( AUXLIB_PIO, luaopen_pio, pio_map )\
  _ROM( AUXLIB_SPI, luaopen_spi, spi_map )\
  _ROM( AUXLIB_TMR, luaopen_tmr, tmr_map )\
  _ROM( AUXLIB_PD, luaopen_pd, pd_map )\
  _ROM( AUXLIB_UART, luaopen_uart, uart_map )\
  PWMLINE\
  TERMLINE\
  _ROM( AUXLIB_PACK, luaopen_pack, pack_map )\
  _ROM( AUXLIB_BIT, luaopen_bit, bit_map )\
  _ROM( AUXLIB_BITARRAY, luaopen_bitarray, bitarray_map )\
  NETLINE\
  _ROM( AUXLIB_CPU, luaopen_cpu, cpu_map )\
  _ROM( AUXLIB_ELUA, luaopen_elua, elua_map )\
  ADCLINE\
  CANLINE\
  RPCLINE\
  _ROM( LUA_MATHLIBNAME, luaopen_math, math_map )\
  PLATLINE

// *****************************************************************************
// Configuration data

// Virtual timers (0 if not used)
#define VTMR_NUM_TIMERS		0
#define VTMR_FREQ_HZ		5

// Number of resources (0 if not available/not implemented)
// TODO: Study the amount of resources of LM4F120H5QR and verify the code
//		 written below (until the end of file)

#define NUM_PIO				6
#define NUM_SPI				1

// TODO: Only UART0 is configured at this moment, so left 1 below.
#define NUM_UART			1

#define NUM_TIMER			1
#define NUM_PWM				1
#define NUM_ADC             1
#define NUM_CAN				1

// Enable RX buffering on UART
#define BUF_ENABLE_UART
#define CON_BUF_SIZE          BUF_SIZE_128

// ADC Configuration Params
#define ADC_BIT_RESOLUTION    10
#define BUF_ENABLE_ADC
#define ADC_BUF_SIZE          BUF_SIZE_2

// These should be adjusted to support multiple ADC devices
#define ADC_TIMER_FIRST_ID    0
#define ADC_NUM_TIMERS        NUM_TIMER  

// RPC boot options
#define RPC_UART_ID           CON_UART_ID
#define RPC_UART_SPEED        CON_UART_SPEED

// TODO: MMCFS not supported yet
//#define MMCFS_CS_PORT                6
//#define MMCFS_CS_PIN                 7
//#define MMCFS_SPI_NUM                1
#if defined( BUILD_MMCFS ) && !defined( MMCFS_SPI_NUM )
  #warning "MMCFS was enabled, but required SPI & CS data are undefined, disabling MMCFS"
  #undef BUILD_MMCFS
#endif


// CPU frequency (needed by the CPU module and MMCFS code, 0 if not used)
#define CPU_FREQUENCY         MAP_SysCtlClockGet()

// PIO prefix ('0' for P0, P1, ... or 'A' for PA, PB, ...)
#define PIO_PREFIX            'A'
// Pins per port configuration:
// #define PIO_PINS_PER_PORT (n) if each port has the same number of pins, or
// #define PIO_PIN_ARRAY { n1, n2, ... } to define pins per port in an array
// Use #define PIO_PINS_PER_PORT 0 if this isn't needed
// TODO: must review this PIO_PIN_ARRAY. I changed him based on LM4F_ALTERNATE_FUNCTIONS.
#define PIO_PIN_ARRAY         { 8, 8, 8, 8, 6, 5 }
//                              A, B, C, D, E, F

#define SRAM_SIZE ( 0x08000 )

// Flash data
// TODO: Verify this for LM4F120H5QR
#define INTERNAL_FLASH_SIZE             ( 256 * 1024 )
#define INTERNAL_FLASH_WRITE_UNIT_SIZE  4
#define INTERNAL_FLASH_SECTOR_SIZE      1024
#define INTERNAL_FLASH_START_ADDRESS    0
#define BUILD_WOFS

// Allocator data: define your free memory zones here in two arrays
// (start address and end address)
#define MEM_START_ADDRESS     { ( void* )end }
#define MEM_END_ADDRESS       { ( void* )( SRAM_BASE + SRAM_SIZE - STACK_SIZE_TOTAL - 1 ) }

// Interrupt list
#define INT_UART_RX           ELUA_INT_FIRST_ID
#define INT_GPIO_POSEDGE      ( ELUA_INT_FIRST_ID + 1 )
#define INT_GPIO_NEGEDGE      ( ELUA_INT_FIRST_ID + 2 )
#define INT_TMR_MATCH         ( ELUA_INT_FIRST_ID + 3 )
#define INT_ELUA_LAST         INT_TMR_MATCH

// *****************************************************************************
// CPU constants that should be exposed to the eLua "cpu" module

#include "hw_ints.h"

#define PLATFORM_CPU_CONSTANTS\
  _C( INT_GPIOA ),\
  _C( INT_GPIOB ),\
  _C( INT_GPIOC ),\
  _C( INT_GPIOD ),\
  _C( INT_GPIOE ),\
  _C( INT_UART0 ),\
  _C( INT_UART1 ),\
  _C( INT_SSI0 ),\
  _C( INT_I2C0 ),\
  _C( INT_PWM_FAULT ),\
  _C( INT_PWM0 ),\
  _C( INT_PWM1 ),\
  _C( INT_PWM2 ),\
  _C( INT_QEI0 ),\
  _C( INT_ADC0 ),\
  _C( INT_ADC1 ),\
  _C( INT_ADC2 ),\
  _C( INT_ADC3 ),\
  _C( INT_WATCHDOG ),\
  _C( INT_TIMER0A ),\
  _C( INT_TIMER0B ),\
  _C( INT_TIMER1A ),\
  _C( INT_TIMER1B ),\
  _C( INT_TIMER2A ),\
  _C( INT_TIMER2B ),\
  _C( INT_COMP0 ),\
  _C( INT_COMP1 ),\
  _C( INT_COMP2 ),\
  _C( INT_SYSCTL ),\
  _C( INT_FLASH ),\
  _C( INT_GPIOF ),\
  _C( INT_GPIOG ),\
  _C( INT_GPIOH ),\
  _C( INT_UART2 ),\
  _C( INT_SSI1 ),\
  _C( INT_TIMER3A ),\
  _C( INT_TIMER3B ),\
  _C( INT_I2C1 ),\
  _C( INT_QEI1 ),\
  _C( INT_CAN0 ),\
  _C( INT_CAN1 ),\
  _C( INT_CAN2 ),\
  _C( INT_ETH ),\
  _C( INT_HIBERNATE ),\
  _C( INT_USB0 ),\
  _C( INT_PWM3 ),\
  _C( INT_UDMA ),\
  _C( INT_UDMAERR ),\
  _C( INT_UART_RX ),\
  _C( INT_GPIO_POSEDGE ),\
  _C( INT_GPIO_NEGEDGE ),\
  _C( INT_TMR_MATCH )

#endif // #ifndef __PLATFORM_CONF_H__
