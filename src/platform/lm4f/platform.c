// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "elua_adc.h"
#include "platform_conf.h"
#include "common.h"
#include "math.h"
#include "diskio.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"
#include "elua_int.h" 

// Platform specific includes

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/pwm.h"
#include "driverlib/adc.h"
#include "driverlib/systick.h"
#include "driverlib/flash.h"
#include "driverlib/interrupt.h"
#include "elua_net.h"
#include "buf.h"
#include "utils.h"

// Target is LM4F120H5QR
//TODO should be protected by ifdef
#include "lm4f120h5qr.h"

#include "driverlib/rom.h"
#include "driverlib/rom_map.h"

// USB CDC Stuff
// TODO: USB CDC will be left disabled until it be implemented and tested
#include "driverlib/usb.h"
//#include "usblib/usblib.h"
//#include "usblib/usbcdc.h"
//#include "usblib/device/usbdevice.h"
//#include "usblib/device/usbdcdc.h"
//#include "usb_serial_structs.h"

// TODO: Verify the defines below (SYSTICK defines)
// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
#define SYSTICKHZ               5
#define SYSTICKMS               (1000 / SYSTICKHZ)

// ****************************************************************************
// Platform initialization

// forward
static void timers_init();
static void uarts_init();
static void spis_init();
static void pios_init();
static void pwms_init();
static void adcs_init();
static void cans_init();
static void usb_init();

int platform_init()
{
  // Set the clocking to run from PLL
  MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

  // Setup PIO
  pios_init();

  // Setup SSIs
  // TODO: SPIs not implemented yet.
  //spis_init();

  // Setup UARTs
  uarts_init();

  // Setup timers
  // TODO: The timers will be left disabled until implemented.
  //timers_init();

  // Setup PWMs
  // TODO: PWN not implemented yet.
  //pwms_init();

#ifdef BUILD_ADC
  // Setup ADCs
  // TODO: ADCs not implemented yet.
  //adcs_init();
#endif

#ifdef BUILD_CAN
  // Setup CANs
  // TODO: CANs not implemented yet.
  //cans_init();
#endif

  // Setup USB
  // TODO: The USB will be left disabled until implemented.
  //usb_init();

  // Setup system timer
  // TODO: Verify this for LM4F120H5QR.  
  cmn_systimer_set_base_freq( MAP_SysCtlClockGet() );
  cmn_systimer_set_interrupt_freq( SYSTICKHZ );

  // Common platform initialization code
  cmn_platform_init();

  // Virtual timers
#if VTMR_NUM_TIMERS > 0
  // Configure SysTick for a periodic interrupt.
  // TODO: Virtual timers not implemented yet.
  //MAP_SysTickPeriodSet( MAP_SysCtlClockGet() / SYSTICKHZ );
  //MAP_SysTickEnable();
  //MAP_SysTickIntEnable();
  //MAP_IntMasterEnable();
#endif

  // TODO: Verify this for LM4F120H5QR.
  MAP_FlashUsecSet( SysCtlClockGet() );

  // All done
  return PLATFORM_OK;
}

// ****************************************************************************
// PIO
// The LM4F120H5QR has 6 ports.

  const u32 pio_base[] = { GPIO_PORTA_BASE, GPIO_PORTB_BASE, GPIO_PORTC_BASE, GPIO_PORTD_BASE,
                                  GPIO_PORTE_BASE, GPIO_PORTF_BASE };
                                  
  const u32 pio_sysctl[] = { SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOC, SYSCTL_PERIPH_GPIOD,
                                    SYSCTL_PERIPH_GPIOE, SYSCTL_PERIPH_GPIOF };

static void pios_init()
{
  unsigned i;

  for( i = 0; i < NUM_PIO; i ++ )
    MAP_SysCtlPeripheralEnable(pio_sysctl[ i ]);
}

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
  pio_type retval = 1, base = pio_base[ port ];

  switch( op )
  {
    case PLATFORM_IO_PORT_SET_VALUE:
      MAP_GPIOPinWrite( base, 0xFF, pinmask );
      break;

    case PLATFORM_IO_PIN_SET:
      MAP_GPIOPinWrite( base, pinmask, pinmask );
      break;

    case PLATFORM_IO_PIN_CLEAR:
      MAP_GPIOPinWrite( base, pinmask, 0 );
      break;

    case PLATFORM_IO_PORT_DIR_INPUT:
      pinmask = 0xFF;
    case PLATFORM_IO_PIN_DIR_INPUT:
      MAP_GPIOPinTypeGPIOInput( base, pinmask );
      break;

    case PLATFORM_IO_PORT_DIR_OUTPUT:
      pinmask = 0xFF;
    case PLATFORM_IO_PIN_DIR_OUTPUT:
      MAP_GPIOPinTypeGPIOOutput( base, pinmask );
      break;

    case PLATFORM_IO_PORT_GET_VALUE:
      retval = MAP_GPIOPinRead( base, 0xFF );
      break;

    case PLATFORM_IO_PIN_GET:
      retval = MAP_GPIOPinRead( base, pinmask ) ? 1 : 0;
      break;

    case PLATFORM_IO_PIN_PULLUP:
    case PLATFORM_IO_PIN_PULLDOWN:
      MAP_GPIOPadConfigSet( base, pinmask, GPIO_STRENGTH_8MA, op == PLATFORM_IO_PIN_PULLUP ? GPIO_PIN_TYPE_STD_WPU : GPIO_PIN_TYPE_STD_WPD );
      break;

    case PLATFORM_IO_PIN_NOPULL:
      MAP_GPIOPadConfigSet( base, pinmask, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD );
      break;

    default:
      retval = 0;
      break;
  }
  return retval;
}

// ****************************************************************************
// CAN
// TODO: Must implement CAN support. LM4F120H5QR has 3 CAN ports.

#if defined( BUILD_CAN )

void CANIntHandler(void)
{

}


void cans_init( void )
{

}


u32 platform_can_setup( unsigned id, u32 clock )
{ 
	return clock;
}

void platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{

}

int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
    return PLATFORM_OK;
}

#endif

// ****************************************************************************
// SPI
// TODO: Must implement SPI support.

static void spis_init()
{

}

u32 platform_spi_setup( unsigned id, int mode, u32 clock, unsigned cpol, unsigned cpha, unsigned databits )
{
	return clock;
}

spi_data_type platform_spi_send_recv( unsigned id, spi_data_type data )
{
	return data;
}

void platform_spi_select( unsigned id, int is_select )
{

}

// ****************************************************************************
// UART
// Configured for LM4F120H5QR

// TODO: Only UART0 was configured. Need to configure other UARTs.
const u32 uart_base[] = { UART0_BASE };
static const u32 uart_sysctl[] = { SYSCTL_PERIPH_UART0 };
static const u32 uart_gpio_base[] = { GPIO_PORTA_BASE };
static const u8 uart_gpio_pins[] = { GPIO_PIN_0 | GPIO_PIN_1 };
static const u32 uart_gpiofunc[] = { GPIO_PA0_U0RX, GPIO_PA1_U0TX };

static void uarts_init()
{
  unsigned i;
  for( i = 0; i < NUM_UART; i ++ )
    MAP_SysCtlPeripheralEnable(uart_sysctl[ i ]);
}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
  u32 config;

  if( id < NUM_UART )
  {
    MAP_GPIOPinConfigure( uart_gpiofunc[ id << 1 ] );
    MAP_GPIOPinConfigure( uart_gpiofunc[ ( id << 1 ) + 1 ] );
    MAP_GPIOPinTypeUART( uart_gpio_base[ id ], uart_gpio_pins[ id ] );

    switch( databits )
    {
      case 5:
        config = UART_CONFIG_WLEN_5;
        break;
      case 6:
        config = UART_CONFIG_WLEN_6;
        break;
      case 7:
        config = UART_CONFIG_WLEN_7;
        break;
      default:
        config = UART_CONFIG_WLEN_8;
        break;
    }
    config |= ( stopbits == PLATFORM_UART_STOPBITS_1 ) ? UART_CONFIG_STOP_ONE : UART_CONFIG_STOP_TWO;
    if( parity == PLATFORM_UART_PARITY_EVEN )
      config |= UART_CONFIG_PAR_EVEN;
    else if( parity == PLATFORM_UART_PARITY_ODD )
      config |= UART_CONFIG_PAR_ODD;
    else
      config |= UART_CONFIG_PAR_NONE;

    MAP_UARTConfigSetExpClk( uart_base[ id ], MAP_SysCtlClockGet(), baud, config );
    MAP_UARTConfigGetExpClk( uart_base[ id ], MAP_SysCtlClockGet(), &baud, &config );

    MAP_UARTEnable( uart_base[ id ] );
  }
  return baud;
}

void platform_s_uart_send( unsigned id, u8 data )
{
  MAP_UARTCharPut( uart_base[ id ], data );
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
  u32 base = uart_base[ id ];

  if( timeout == 0 )
    return MAP_UARTCharGetNonBlocking( base );

  return MAP_UARTCharGet( base );
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
  return PLATFORM_ERR;
}

// ****************************************************************************
// Timers
// TODO: Must implement support for timers.

// This variable was left because is referred in other files.
const u32 timer_base[];

static void timers_init()
{

}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{

}

timer_data_type platform_s_timer_op( unsigned id, int op,timer_data_type data )
{
  return 0;
}

u64 platform_timer_sys_raw_read()
{

}

void platform_timer_sys_disable_int()
{

}

void platform_timer_sys_enable_int()
{

}

timer_data_type platform_timer_read_sys()
{

}

u8 lm4f_timer_int_periodic_flag[ NUM_TIMER ];
int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
  return PLATFORM_TIMER_INT_OK;
}

// ****************************************************************************
// PWMs
// TODO: Must implement PWM support.

static void pwms_init()
{

}

// Return the PWM clock
u32 platform_pwm_get_clock( unsigned id )
{
  return 0;
}

// Set the PWM clock
u32 platform_pwm_set_clock( unsigned id, u32 clock )
{
  return 0;
}

u32 platform_pwm_setup( unsigned id, u32 frequency, unsigned duty )
{
  return 0;
}

void platform_pwm_start( unsigned id )
{

}

void platform_pwm_stop( unsigned id )
{

}

// *****************************************************************************
// ADC specific functions and variables

#ifdef BUILD_ADC

int platform_adc_check_timer_id( unsigned id, unsigned timer_id )
{
  return 0;
}

void platform_adc_stop( unsigned id )
{

}

// Handle ADC interrupts
void ADCIntHandler( void )
{

}

static void adcs_init()
{

}

u32 platform_adc_set_clock( unsigned id, u32 frequency )
{
  return frequency;
}


int platform_adc_update_sequence( )
{  
  return PLATFORM_OK;
}


int platform_adc_start_sequence()
{ 
  return PLATFORM_OK;
}

#endif // ifdef BUILD_ADC


// ****************************************************************************
// USB functions

#if defined( BUILD_USB_CDC )

static void usb_init()
{

}

void platform_usb_cdc_send( u8 data )
{

}

int platform_usb_cdc_recv( s32 timeout )
{

}

unsigned long TxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
  return(0);
}

unsigned long RxHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void *pvMsgData)
{
  return(0);
}

unsigned long
ControlHandler(void *pvCBData, unsigned long ulEvent, unsigned long ulMsgValue,
               void *pvMsgData)
{
  return(0);
}

#endif // BUILD_USB_CDC

// ****************************************************************************
// Flash access functions

#ifdef BUILD_WOFS
u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  return MAP_FlashProgram( ( unsigned long * )from, toaddr, size );
}

int platform_flash_erase_sector( u32 sector_id )
{
  return FlashErase( sector_id * INTERNAL_FLASH_SECTOR_SIZE ) == 0 ? PLATFORM_OK : PLATFORM_ERR;
}
#endif // #ifdef BUILD_WOFS

// ****************************************************************************
// Platform specific modules go here

#if defined( ENABLE_LM4F_GPIO )

#define MIN_OPT_LEVEL 2
#include "lrodefs.h"

extern const LUA_REG_TYPE lm4f_pio_map[];

const LUA_REG_TYPE platform_map[] =
{
#if LUA_OPTIMIZE_MEMORY > 0
#ifdef ENABLE_LM4F_GPIO
  { LSTRKEY( "pio" ), LROVAL( lm4f_pio_map ) },
#endif
#endif
  { LNILKEY, LNILVAL }
};

LUALIB_API int luaopen_platform( lua_State *L )
{
#if LUA_OPTIMIZE_MEMORY > 0
  return 0;
#else // #if LUA_OPTIMIZE_MEMORY > 0
  luaL_register( L, PS_LIB_TABLE_NAME, platform_map );

  // Setup the new tables inside platform table
  lua_newtable( L );
  luaL_register( L, NULL, lm4f_pio_map );
  lua_setfield( L, -2, "pio" );

  return 1;
#endif // #if LUA_OPTIMIZE_MEMORY > 0
}

#else // #if defined( ENABLE_LM4F_GPIO )

LUALIB_API int luaopen_platform( lua_State *L )
{
  return 0;
}

#endif // #if defined( ENABLE_LM4F_GPIO )
